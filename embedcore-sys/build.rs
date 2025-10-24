use std::env;
use std::path::PathBuf;

fn main() {
    // Tell cargo to look for shared libraries in the specified directory
    println!("cargo:rustc-link-search=native=.");
    
    // Tell cargo to tell rustc to link the C++ HAL library
    println!("cargo:rustc-link-lib=static=embedcore_hal");
    
    // Tell cargo to invalidate the built crate whenever the wrapper changes
    println!("cargo:rerun-if-changed=src/lib.rs");
    
    // Get the path to the C++ HAL source files
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let cpp_hal_dir = manifest_dir.parent().unwrap().join("cpp_hal");
    let include_dir = cpp_hal_dir.join("include");
    let src_dir = cpp_hal_dir.join("src");
    
    // Compile the C++ HAL library
    let mut build = cc::Build::new();
    build.cpp(true)
        .std("c++17")
        .include(&include_dir)
        .file(src_dir.join("gpio.cpp"))
        .file(src_dir.join("pwm.cpp"))
        .file(src_dir.join("uart.cpp"))
        .file(src_dir.join("timer.cpp"))
        .file(src_dir.join("motor.cpp"));
    
    // Add MSVC-specific flags
    if env::var("TARGET").unwrap().contains("msvc") {
        build.flag("/EHsc"); // Enable C++ exception handling
    }
    
    build.compile("embedcore_hal");
    
    // Tell cargo to invalidate the built crate whenever the C++ source changes
    println!("cargo:rerun-if-changed={}", cpp_hal_dir.display());
    for entry in std::fs::read_dir(&src_dir).unwrap() {
        let entry = entry.unwrap();
        println!("cargo:rerun-if-changed={}", entry.path().display());
    }
    for entry in std::fs::read_dir(&include_dir).unwrap() {
        let entry = entry.unwrap();
        println!("cargo:rerun-if-changed={}", entry.path().display());
    }
}
