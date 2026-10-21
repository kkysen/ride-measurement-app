import com.google.protobuf.gradle.proto
import javax.inject.Inject

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.protobuf)
}

android {
    namespace = "dev.khyber.ride"
    compileSdk = 37

    defaultConfig {
        applicationId = "dev.khyber.ride"
        // Android 14: needed for the `specialUse` fallback service type after a background restart.
        minSdk = 34
        targetSdk = 36
        versionCode = 1
        versionName = "0.1.0"
        // Azure app registration for OneDrive uploads, set in `local.properties` or the environment.
        val clientId = providers.gradleProperty("ride.onedriveClientId")
            .orElse(providers.environmentVariable("RIDE_ONEDRIVE_CLIENT_ID"))
            .getOrElse("")
        buildConfigField("String", "ONEDRIVE_CLIENT_ID", "\"$clientId\"")
        // The Rust library is built only for ARM64, the only ABI of the phones this targets.
        ndk { abiFilters += "arm64-v8a" }
    }

    // Every build, local or CI, debug or release, is signed with the same key,
    // so an update never fails with a signature mismatch that would force an uninstall.
    // There is deliberately no fallback to the per-machine debug key.
    signingConfigs {
        create("shared") {
            fun prop(name: String) = providers.gradleProperty("ride.signing.$name")
                .orElse(providers.environmentVariable("RIDE_SIGNING_${name.uppercase()}"))
                .orNull
                ?: throw GradleException(
                    "missing signing property ride.signing.$name (see README); refusing to sign with another key",
                )
            storeFile = file(prop("storeFile"))
            storePassword = prop("storePassword")
            keyAlias = prop("keyAlias")
            keyPassword = prop("keyPassword")
        }
    }

    buildTypes {
        debug {
            signingConfig = signingConfigs["shared"]
        }
        release {
            signingConfig = signingConfigs["shared"]
        }
    }

    // `android.util.Log` and friends return defaults in JVM unit tests instead of throwing.
    testOptions.unitTests.isReturnDefaultValues = true

    buildFeatures {
        compose = true
        buildConfig = true
    }

    sourceSets["main"].proto { srcDir("../../proto") }
}

// The Rust core (`crates/ffi`), built with `cargo` and called through `uniffi`-generated Kotlin bindings.
val repoRoot = rootDir.parentFile
val cargoTarget = File(repoRoot, "target")

abstract class CargoTask @Inject constructor(private val exec: ExecOperations) : DefaultTask() {
    @get:Internal
    abstract val repoRoot: DirectoryProperty

    // Rebuild whenever any Rust source or manifest changes; `cargo` itself skips work that's up to date.
    @get:InputFiles
    abstract val rustSources: ConfigurableFileCollection

    protected fun cargo(vararg args: String) {
        exec.exec {
            workingDir = repoRoot.get().asFile
            commandLine("cargo", *args)
        }
    }
}

// The Android library, packaged as `lib/arm64-v8a/libride_ffi.so`.
abstract class CargoBuildAndroid @Inject constructor(exec: ExecOperations, private val fs: FileSystemOperations) :
    CargoTask(exec) {
    @get:OutputDirectory
    abstract val outDir: DirectoryProperty

    @TaskAction
    fun build() {
        cargo("build", "--release", "-p", "ride-ffi", "--lib", "--target", "aarch64-linux-android")
        fs.sync {
            from(repoRoot.file("target/aarch64-linux-android/release/libride_ffi.so"))
            into(outDir.dir("arm64-v8a"))
        }
    }
}

// Kotlin bindings, generated from the host build of the library (library mode reads the metadata in it),
// which JVM unit tests also load.
abstract class UniffiBindings @Inject constructor(exec: ExecOperations, private val fs: FileSystemOperations) :
    CargoTask(exec) {
    @get:OutputDirectory
    abstract val outDir: DirectoryProperty

    @TaskAction
    fun generate() {
        cargo("build", "--release", "-p", "ride-ffi", "--lib")
        fs.delete { delete(outDir) }
        cargo(
            "run", "--release", "-q", "-p", "ride-ffi", "--bin", "uniffi-bindgen", "--",
            "generate", "--library", "target/release/libride_ffi.so",
            "--language", "kotlin", "--no-format", "--out-dir", outDir.get().asFile.path,
        )
    }
}

val rustInputs = files(
    File(repoRoot, "Cargo.toml"),
    File(repoRoot, "Cargo.lock"),
    File(repoRoot, ".cargo/config.toml"),
    fileTree(File(repoRoot, "crates")) { include("**/*.rs", "**/Cargo.toml") },
    fileTree(File(repoRoot, "proto")),
)
val cargoBuildAndroid = tasks.register<CargoBuildAndroid>("cargoBuildAndroid") {
    repoRoot.set(rootDir.parentFile)
    rustSources.from(rustInputs)
    outDir.set(layout.buildDirectory.dir("rust/jniLibs"))
}
val uniffiBindings = tasks.register<UniffiBindings>("uniffiBindings") {
    repoRoot.set(rootDir.parentFile)
    rustSources.from(rustInputs)
    outDir.set(layout.buildDirectory.dir("generated/uniffi"))
}

androidComponents {
    onVariants { variant ->
        variant.sources.jniLibs?.addGeneratedSourceDirectory(cargoBuildAndroid, CargoBuildAndroid::outDir)
        variant.sources.kotlin?.addGeneratedSourceDirectory(uniffiBindings, UniffiBindings::outDir)
    }
}

tasks.withType<Test>().configureEach {
    dependsOn(uniffiBindings)
    // Where JNA finds the host build of the library.
    systemProperty("jna.library.path", File(cargoTarget, "release").path)
    // `FixtureTest` writes sessions here for the Rust and Python cross-checks,
    // so a build cache hit has to restore them too, not just the test results.
    outputs.dir(layout.buildDirectory.dir("fixtures"))
}

protobuf {
    protoc {
        artifact = libs.protoc.get().toString()
    }
    generateProtoTasks {
        all().configureEach {
            builtins {
                create("java") {
                    option("lite")
                }
            }
        }
    }
}

dependencies {
    implementation(platform(libs.compose.bom))
    implementation(libs.compose.material3)
    implementation(libs.compose.ui)
    implementation(libs.activity.compose)
    implementation(libs.lifecycle.runtime.compose)
    implementation(libs.work.runtime)
    implementation(libs.protobuf.javalite)
    implementation("${libs.jna.get()}@aar")
    implementation(libs.osmdroid)

    testImplementation(libs.junit)
    // The JVM build of JNA, for loading the host build of the Rust library in unit tests.
    testImplementation(libs.jna)
}
