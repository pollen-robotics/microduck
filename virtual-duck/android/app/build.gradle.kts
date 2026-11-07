import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "io.github.samimreza.virtualduck"
    compileSdk = 36

    defaultConfig {
        applicationId = "io.github.samimreza.virtualduck"
        // 26 is where TYPE_APPLICATION_OVERLAY begins; the duck has no way onto other apps' screens
        // before it.
        minSdk = 26
        targetSdk = 36
        versionCode = 1
        versionName = "0.1.0"
        ndk {
            // Phones, old phones, and the x86_64 emulator. Filament's native code is most of the APK,
            // and 32-bit x86 is a device class that no longer exists.
            abiFilters += listOf("arm64-v8a", "armeabi-v7a", "x86_64")
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    androidResources {
        // Read straight out of the APK by Filament and SoundPool; compressing them only costs a copy.
        noCompress += listOf("glb", "ogg")
    }

    testOptions {
        unitTests.isReturnDefaultValues = true
    }
}

kotlin {
    compilerOptions {
        jvmTarget.set(JvmTarget.JVM_17)
    }
}

dependencies {
    // 1.75.1 is the last Filament that compiles against API 36: from 1.76.0 its AAR metadata
    // demands compileSdk 37, which needs AGP 9 — a toolchain move for nothing this app uses.
    implementation("com.google.android.filament:filament-android:1.75.1")
    implementation("com.google.android.filament:gltfio-android:1.75.1")

    testImplementation("junit:junit:4.13.2")
    // org.json is part of Android, but only as stubs on the JVM the unit tests run on.
    testImplementation("org.json:json:20260814")
}
