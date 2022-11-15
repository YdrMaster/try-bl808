fn main() {
    use std::{env, fs, path::PathBuf};

    let ld = &PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("see.ld");
    fs::write(ld, LINKER).unwrap();
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=LOG");
    println!("cargo:rustc-link-arg=-T{}", ld.display());
}

const LINKER: &[u8] = b"
OUTPUT_ARCH(RISCV)
ENTRY(_start)
MEMORY {
    xip-flash : ORIGIN = 0x58000000, LENGTH = 4M
    ocram     : ORIGIN = 0x22020000, LENGTH = 64K
}
SECTIONS {
    .text 0x58000000 : {
        *(.text.entry)
        *(.text .text.*)
    } > xip-flash

    .rodata : {
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
    } > xip-flash

    .data : {
        *(.data .data.*)
        *(.sdata .sdata.*)
    } > ocram

    .bss (NOLOAD) : {
        *(.bss.uninit)
        . = ALIGN(4);
        sbss = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        . = ALIGN(4);
        ebss = .;
    } > ocram
}";
