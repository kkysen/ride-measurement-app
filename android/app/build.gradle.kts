import com.google.protobuf.gradle.proto

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.protobuf)
}

android {
    namespace = "dev.kkysen.ride"
    compileSdk = 37

    defaultConfig {
        applicationId = "dev.kkysen.ride"
        minSdk = 31
        targetSdk = 36
        versionCode = 1
        versionName = "0.1.0"
        // Azure app registration for OneDrive uploads, set in `local.properties` or the environment.
        val clientId = providers.gradleProperty("ride.onedriveClientId")
            .orElse(providers.environmentVariable("RIDE_ONEDRIVE_CLIENT_ID"))
            .getOrElse("")
        buildConfigField("String", "ONEDRIVE_CLIENT_ID", "\"$clientId\"")
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

    buildFeatures {
        compose = true
        buildConfig = true
    }

    sourceSets["main"].proto { srcDir("../../proto") }
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
    implementation("${libs.zstd.jni.get()}@aar")

    testImplementation(libs.junit)
    // The JVM build of zstd-jni, for unit tests on the host.
    testImplementation(libs.zstd.jni)
}
