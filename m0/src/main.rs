#![no_std]
#![no_main]
#![feature(naked_functions, asm_const)]

use core::mem::transmute;

use bl808_pac::{glb::gpio_config, GLB};

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
    let p = unsafe { bl808_pac::Peripherals::steal() };

    // p.GLB.

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

fn bl_uart_init(id: u8, tx_pin: u8, rx_pin: u8, cts_pin: u8, rts_pin: u8, baudrate: u32) {
    const UART_CFG: UartCfg = UartCfg {
        clk: 80_000_000,
        baud_rate: 2_000_000,
        data_bits: UartDataBits::Eight,
        stop_bits: UartStopBits::One,
        parity: UartParity::None,
        cts_flow_control: false,
        rx_deglitch: false,
        rts_software_control: false,
        tx_software_control: false,
        tx_lin_mode: false,
        rx_lin_mode: false,
        tx_break_bit_cnt: 0,
        byte_bit_inverse: ByteBitInverse::LsbFirst,
    };
    const FIFO_CFG: UartFifoCfg = UartFifoCfg {
        tx_fifo_dma_threshold: 16,
        rx_fifo_dma_threshold: 16,
        rx_fifo_dma_enable: false,
        tx_fifo_dma_enable: false,
    };
}

fn uart_gpio_demo(id: u8, tx_pin: u8, rx_pin: u8, cts_pin: u8, rts_pin: u8) {
    let mut gpio_config = GlbGpioCfgType {
        gpio_pin: tx_pin,
        gpio_fun: 7,
        gpio_mode: 2,
        pull_type: 0,
        drive: 0,
        smt_ctrl: 1,
        output_mode: 0,
    };
    glb_uart_sig_swap_set(GlbUartSigSwapGrp::Gpio12Gpio23, 1);
    glb_uart_sig_swap_set(GlbUartSigSwapGrp::Gpio36Gpio45, 1);
    let fun = GlbUartSigFun::Uart0Txd;
    let sig = uart_signal_get(gpio_config.gpio_pin);
    glb_uart_fun_sel(sig, fun)
}

fn glb_uart_sig_swap_set(group: GlbUartSigSwapGrp, swap: u8) {
    const GLB_PARM_CFG0_OFFSET: usize = 0x510;
    let ptr = unsafe {
        bl808_pac::GLB::PTR
            .cast::<u8>()
            .add(GLB_PARM_CFG0_OFFSET)
            .cast::<u32>()
    };
    let mut cfg_reg = unsafe { ptr.read_volatile() };
    if swap != 0 {
        cfg_reg |= 1 << (group as u32 + 2);
    } else {
        cfg_reg &= !(1 << (group as u32 + 2));
    }
    unsafe { ptr.cast_mut().write_volatile(cfg_reg) }
}

fn uart_signal_get(mut pin: u8) -> u8 {
    //TODO no magic number is allowed here
    if (12..=23).contains(&pin) || (36..=45).contains(&pin) {
        pin += 6;
    }
    pin % 12
}

fn glb_uart_fun_sel(sig: GlbUartSig, fun: GlbUartSigFun) {
    if (sig as u32) < (GlbUartSig::Eight as u32) {
        let ptr = unsafe { (*bl808_pac::GLB::PTR).uart_signal_0.as_ptr() };
        let mut val = unsafe { ptr.read_volatile() };
        let sig_pos = sig as u32 * 4;
        val &= !(0xf << sig_pos);
        val |= (fun as u32) << sig_pos;
        unsafe { ptr.write_volatile(val) };
    } else {
        let ptr = unsafe { (*bl808_pac::GLB::PTR).uart_signal_1.as_ptr() };
        let mut val = unsafe { ptr.read_volatile() };
        let sig_pos = (sig as u32 - 8) * 4;
        val &= !(0xf << sig_pos);
        val |= (fun as u32) << sig_pos;
        unsafe { ptr.write_volatile(val) };
    }
}

struct UartCfg {
    clk: u32,
    baud_rate: u32,
    data_bits: UartDataBits,
    stop_bits: UartStopBits,
    parity: UartParity,
    cts_flow_control: bool,
    rx_deglitch: bool,
    rts_software_control: bool,
    tx_software_control: bool,
    tx_lin_mode: bool,
    rx_lin_mode: bool,
    tx_break_bit_cnt: u8,
    byte_bit_inverse: ByteBitInverse,
}

enum UartDataBits {
    Five,
    Six,
    Seven,
    Eight,
}

enum UartStopBits {
    Half,
    One,
    OneAndHalf,
    Two,
}

enum UartParity {
    None,
    Odd,
    Even,
}

enum ByteBitInverse {
    LsbFirst,
    MsbFirst,
}

struct UartFifoCfg {
    tx_fifo_dma_threshold: u8,
    rx_fifo_dma_threshold: u8,
    tx_fifo_dma_enable: bool,
    rx_fifo_dma_enable: bool,
}

struct GlbGpioCfgType {
    gpio_pin: u8,
    gpio_fun: u8,
    gpio_mode: u8,
    pull_type: u8,
    drive: u8,
    smt_ctrl: u8,
    output_mode: u8,
}

#[repr(u32)]
enum GlbUartSigSwapGrp {
    Gpio0Gpio11 = 0,
    Gpio12Gpio23,
    Gpio24Gpio35,
    Gpio36Gpio45,
}

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
