#![no_std]
#![no_main]
#![feature(naked_functions, asm_const)]

use core::mem::transmute;

#[naked]
#[no_mangle]
#[link_section = ".text.entry"]
unsafe extern "C" fn _start() -> ! {
    const LEN_STACK_M0: usize = 1024;
    #[link_section = ".bss.uninit"]
    static mut STACK: [u8; LEN_STACK_M0] = [0; LEN_STACK_M0];
    core::arch::asm!(
        "   la   t0, 0f
            csrw mtvec, t0
            la   sp, {stack} + {size}
            j    {main}

         0: j    {light_on}
        ",
        size     = const LEN_STACK_M0,
        stack    =   sym STACK,
        light_on =   sym light_on,
        main     =   sym main,
        options(noreturn),
    )
}

extern "C" fn main() -> ! {
    bl_uart_init(0, 14, 15, 255, 255, 2_000_000);

    let p = unsafe { bl808_pac::Peripherals::steal() };

    for _ in 0..1000 {
        p.UART0.data_write.write(|w| w.value().variant(42));
    }

    p.GLB.gpio_config[8].modify(|_, w| {
        w.pin_mode()
            .output()
            .output_function()
            .set_bit()
            .input_function()
            .clear_bit()
            .pull_up()
            .set_bit()
    });
    loop {
        p.GLB.gpio_config[8].modify(|_, w| w.output_set().clear_bit());
        for _ in 0..100_000_000 {
            core::hint::spin_loop();
        }
        p.GLB.gpio_config[8].modify(|_, w| w.output_set().set_bit());
        for _ in 0..100_000_000 {
            core::hint::spin_loop();
        }
    }
}

extern "C" fn light_on() -> ! {
    let p = unsafe { bl808_pac::Peripherals::steal() };
    p.GLB.gpio_config[8].modify(|_, w| {
        w.pin_mode()
            .output()
            .output_function()
            .set_bit()
            .input_function()
            .clear_bit()
            .pull_up()
            .set_bit()
    });
    p.GLB.gpio_config[8].modify(|_, w| w.output_set().clear_bit());
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

fn bl_uart_init(_id: u8, tx_pin: u8, rx_pin: u8, _cts_pin: u8, _rts_pin: u8, baudrate: u32) {
    unsafe {
        let ptr = bl808_pac::GLB::PTR
            .cast::<u8>()
            .add(0x510)
            .cast::<u32>()
            .cast_mut();
        ptr.write_volatile(ptr.read_volatile() | (1 << 3) | (1 << 5));
    };

    let fun = GlbUartSigFun::Uart0Txd;
    let sig = (tx_pin as u32 + 6) % 12;
    unsafe {
        glb_uart_fun_sel(transmute(sig), transmute(fun));
        glb_uart_fun_sel(transmute(fun), transmute(sig));
    }

    let fun = GlbUartSigFun::Uart0Rxd;
    let sig = (rx_pin as u32 + 6) % 12;
    unsafe {
        glb_uart_fun_sel(transmute(sig), transmute(fun));
        glb_uart_fun_sel(transmute(fun), transmute(sig));
    }

    glb_set_uart_clk(true, HbnUartClk::ClkMcuPbclk, 0);

    let dev = unsafe { &(*bl808_pac::UART0::PTR) };
    // uart_disable
    dev.transmit_config.modify(|_, w| w.function().clear_bit());
    dev.receive_config.modify(|_, w| w.function().clear_bit());

    let clk = 80_000_000;
    let fraction = clk * 10 / baudrate % 10;
    let mut divisor = clk / baudrate;
    if fraction >= 5 {
        divisor += 1;
    }
    dev.bit_period.write(|w| {
        w.receive().variant((divisor - 1) as _);
        w.transmit().variant((divisor - 1) as _)
    });
    dev.transmit_config.write(|w| {
        w.parity_enable().clear_bit();
        w.word_length().eight();
        w.stop_bits().one();
        w.cts().disable();
        w.lin_transmit().disable();
        w.break_bits().variant(0)
    });
    dev.receive_config.write(|w| {
        w.parity_enable().clear_bit();
        w.word_length().eight();
        w.deglitch_enable().disable();
        w.lin_receive().disable()
    });
    dev.data_config.write(|w| w.bit_order().clear_bit());
    dev.signal_override.write(|w| {
        w.rts_signal().disable();
        w.transmit_signal().disable()
    });

    // uart_enable
    dev.transmit_config.modify(|_, w| w.function().set_bit());
    dev.receive_config.modify(|_, w| w.function().set_bit());
}

fn glb_uart_fun_sel(sig: GlbUartSig, fun: GlbUartSigFun) {
    let (ptr, sig_pos) = if (sig as u32) < (GlbUartSig::Eight as u32) {
        (
            unsafe { (*bl808_pac::GLB::PTR).uart_signal_0.as_ptr() },
            sig as u32 * 4,
        )
    } else {
        (
            unsafe { (*bl808_pac::GLB::PTR).uart_signal_1.as_ptr() },
            (sig as u32 - 8) * 4,
        )
    };
    let mut val = unsafe { ptr.read_volatile() };
    val &= !(0xf << sig_pos);
    val |= (fun as u32) << sig_pos;
    unsafe { ptr.write_volatile(val) };
}

fn glb_set_uart_clk(enable: bool, clk_sel: HbnUartClk, div: u8) {
    let reg = unsafe { &(*bl808_pac::GLB::PTR).uart_config };
    reg.modify(|_, w| w.clock_enable().clear_bit());
    reg.modify(|_, w| w.clock_divide().variant(div));
    hbn_set_uart_clk_sel(clk_sel);
    if enable {
        reg.modify(|_, w| w.clock_enable().set_bit());
    }
}

fn hbn_set_uart_clk_sel(clk_sel: HbnUartClk) {
    let ptr = unsafe { (*bl808_pac::HBN::PTR).global.as_ptr() };
    let mut val = unsafe { ptr.read_volatile() };
    match clk_sel {
        HbnUartClk::ClkMcuPbclk => {
            val &= !(1 << 15);
            val &= !(1 << 2);
        }
        HbnUartClk::Clk160m => {
            val &= !(1 << 15);
            val |= 1 << 2;
        }
        HbnUartClk::ClkXclk => {
            val |= 1 << 15;
            val &= !(1 << 2);
        }
    }
    unsafe { ptr.write_volatile(val) };
}

#[derive(Clone, Copy)]
#[repr(u32)]
enum GlbUartSigFun {
    Uart0Rts = 0,
    Uart0Cts,
    Uart0Txd,
    Uart0Rxd,
    Uart1Rts,
    Uart1Cts,
    Uart1Txd,
    Uart1Rxd,
    Uart2Rts,
    Uart2Cts,
    Uart2Txd,
    Uart2Rxd,
}

#[derive(Clone, Copy)]
#[repr(u32)]
enum GlbUartSig {
    Zero = 0,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
    Nine,
    Ten,
    Eleven,
}

enum HbnUartClk {
    ClkMcuPbclk,
    Clk160m,
    ClkXclk,
}
