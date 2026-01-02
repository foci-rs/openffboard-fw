use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Anchor protocol codegen
    anchor_codegen::ConfigBuilder::new()
        .entry("src/main.rs")
        .set_version(env!("CARGO_PKG_VERSION"))
        .set_build_versions("")
        .build();

    // Copy memory.x to OUT_DIR so the linker can find it
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());

    // Linker configuration
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
