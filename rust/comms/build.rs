fn main() {
    // Vendored ZeroMQ uses Windows security-descriptor APIs for its socket pair.
    // Its build script omits this import library. Inspect the target (not host),
    // so cross-compiling Linux services from Windows never links Windows APIs.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        println!("cargo:rustc-link-lib=advapi32");
    }
}
