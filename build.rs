//! Emit the linker memory layout into `OUT_DIR` and add the linker args the
//! cortex-m-rt / defmt link scripts need.
//!
//! By requesting that Cargo re-run this script whenever `memory.x` changes,
//! updating the memory map ensures a rebuild of the application with the new
//! settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // The memory layout is `include_bytes!`-ed and written into OUT_DIR rather
    // than being linked straight out of the crate root.  This mirrors the
    // upstream badge firmware, which keeps the script out of the crate root on
    // purpose: GNU ld's `INCLUDE` searches the current working directory (the
    // project root) before any `-L` path, so a `memory.x` sitting in the
    // project root would shadow the OUT_DIR copy and every build variant would
    // silently link with the same memory map.  Generating it here keeps the
    // OUT_DIR copy authoritative, so adding a second layout later (a
    // bootloader-less variant, say) is a one-line change instead of a debugging
    // session.
    let script: &[u8] = include_bytes!("memory.x");

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(script)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    // `--nmagic` turns off page alignment of sections, which saves flash and
    // is required for `flip-link` to relocate the stack correctly.
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
