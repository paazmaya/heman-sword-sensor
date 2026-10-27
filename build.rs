fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    if std::env::var("TARGET").as_deref() == Ok("thumbv7em-none-eabihf") {
        let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR"));
        std::fs::copy("memory.x", out.join("memory.x")).expect("copy memory.x");
        println!("cargo:rustc-link-search={}", out.display());
    }
}
