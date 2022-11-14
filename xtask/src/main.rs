#[macro_use]
extern crate clap;

use clap::Parser;
use once_cell::sync::Lazy;
use os_xtask_utils::{BinUtil, Cargo, CommandExt};
use std::{
    fs,
    path::{Path, PathBuf},
};

static PROJECT: Lazy<&'static Path> =
    Lazy::new(|| Path::new(std::env!("CARGO_MANIFEST_DIR")).parent().unwrap());

#[derive(Parser)]
#[clap(name = "BL808")]
#[clap(version, about, long_about = None)]
struct Cli {
    #[clap(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Make this project
    Make(BuildArgs),
    /// Dump assembly code of RustSBI-QEMU
    Asm(AsmArgs),
}

fn main() {
    use Commands::*;
    match Cli::parse().command {
        Make(args) => {
            args.make("m0", true);
        }
        Asm(args) => args.dump(),
    }
}

#[derive(Args, Default)]
struct BuildArgs {
    /// Log level.
    #[clap(long)]
    log: Option<String>,
    /// Build in debug mode.
    #[clap(long)]
    debug: bool,
}

impl BuildArgs {
    fn make(&self, package: &str, binary: bool) -> PathBuf {
        let target = "riscv32imac-unknown-none-elf";
        Cargo::build()
            .package(package)
            .optional(&self.log, |cargo, log| {
                cargo.env("LOG", log);
            })
            .conditional(!self.debug, |cargo| {
                cargo.release();
            })
            .target(target)
            .invoke();
        let elf = PROJECT
            .join("target")
            .join(target)
            .join(if self.debug { "debug" } else { "release" })
            .join(package);
        if binary {
            let bin = elf.with_extension("bin");
            BinUtil::objcopy()
                .arg(elf)
                .arg("--strip-all")
                .args(["-O", "binary"])
                .arg(&bin)
                .invoke();
            bin
        } else {
            elf
        }
    }
}

#[derive(Args)]
struct AsmArgs {
    #[clap(flatten)]
    build: BuildArgs,
    /// Output file.
    #[clap(short, long)]
    output: Option<String>,
}

impl AsmArgs {
    fn dump(self) {
        let elf = self.build.make("m0", false);
        let out = PROJECT.join(self.output.unwrap_or(format!(
            "{}.asm",
            elf.file_stem().unwrap().to_string_lossy()
        )));
        println!("Asm file dumps to '{}'.", out.display());
        fs::write(out, BinUtil::objdump().arg(elf).arg("-d").output().stdout).unwrap();
    }
}
