#![no_std]
#![no_main]
#![feature(naked_functions, asm_const)]

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

         0: j  {light_on}
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
