use std::{env, fs, path::PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let memory = PathBuf::from("../../memory.x");
    fs::copy(&memory, out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed={}", memory.display());
    println!("cargo:rerun-if-changed=build.rs");
}
