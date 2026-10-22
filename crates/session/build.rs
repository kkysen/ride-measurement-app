fn main() -> anyhow::Result<()> {
    let proto_root = "../../proto";
    println!("cargo::rerun-if-changed={proto_root}");
    let fds = protox::compile(["ride/v1/session.proto"], [proto_root])?;
    prost_build::Config::new().compile_fds(fds)?;
    // Only for reading the transit feeds recorded in `GtfsRealtimeFeed`; the app stores them undecoded.
    println!("cargo::rerun-if-changed=proto");
    let fds = protox::compile(["gtfs_realtime.proto"], ["proto"])?;
    prost_build::Config::new().compile_fds(fds)?;
    Ok(())
}
