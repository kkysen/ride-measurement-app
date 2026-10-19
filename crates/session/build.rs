fn main() -> anyhow::Result<()> {
    let proto_root = "../../proto";
    println!("cargo::rerun-if-changed={proto_root}");
    let fds = protox::compile(["ride/v1/session.proto"], [proto_root])?;
    prost_build::Config::new().compile_fds(fds)?;
    Ok(())
}
