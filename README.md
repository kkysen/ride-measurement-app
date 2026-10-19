# Ride Logger

Records rides (trains, buses, planes) from an Android phone's sensors
for later offline analysis, without ever losing data.
See `PLAN.md` for the design.

Layout:

- `proto/`: the session format schema.
- `crates/session`: Rust session reader/writer, the reference for the format.
- `crates/cli`: the `ride` CLI (`summary`, `csv`).
- `crates/py` + `python/`: the `ride` Python package (numpy arrays, plots) on top of the Rust reader.
- `android/`: the logger app (Kotlin, Jetpack Compose).

## Your steps

Things only you can do, in order. Step 1 is already done on this machine.

### 1. Toolchain (done)

```sh
mise install                                   # Java and Gradle, from `mise.toml`
sudo apt install box64                         # runs Google's x86-64-only `aapt2` on ARM64 Linux
```

The Android SDK goes in `~/Android/Sdk`.
Gradle downloads the packages it needs once the license is accepted,
by `scripts/install-android-sdk.sh` or by the marker in `~/Android/Sdk/licenses/`.

### 2. Back up the signing key (do this now)

Every build, local or CI, is signed with one key, so the app can always update in place.
If the key is lost, the installed app can't be updated without an uninstall.

Back up both of these somewhere safe (e.g. a password manager):

- `~/.config/ride-logger/signing.jks`
- the `ride.signing.*` lines in `~/.gradle/gradle.properties`

Recordings themselves survive an uninstall (they live in shared storage), but the app's settings and OneDrive sign-in don't.

### 3. Install on the phone, without a cable

1. Phone: Settings > About phone, tap Build number 7 times to enable developer options.
2. Phone: Developer options > Wireless debugging: turn on, then "Pair device with pairing code".
3. Here: `adb pair <ip>:<pairing port>` and enter the code.
4. Here: `adb connect <ip>:<port>` (the port on the main Wireless debugging screen, not the pairing one).
5. `just install`.
6. In the app: grant all-files access, location, and notifications.
7. Recommended: Developer options > "Force full GNSS measurements" (for later GNSS logging).

The phone and this machine must be on the same network.
`adb` here is the x86-64 build running under `box64`.

### 4. Record and check a ride

1. Tap Start, ride, tap Stop. Recording continues with the screen off.
2. `just pull` copies sessions into `data/sessions/` (nothing is deleted on the phone).
3. `just summary data/sessions/<session>` shows rates and gaps; `just plot data/sessions/<session> accel` plots.

### 5. OneDrive uploads (optional, any time)

Without this the app just keeps sessions on the phone.

1. Go to https://portal.azure.com and sign in with the Microsoft account your OneDrive uses.
   If it says you have no directory, sign up for a free Azure account first (app registrations are free).
2. App registrations > New registration:
   - Name: `Ride Logger` (also the OneDrive folder name).
   - Supported account types: Personal Microsoft accounts only.
   - Redirect URI: leave empty.
3. Copy the Application (client) ID.
4. Authentication (or its Settings tab) > Advanced settings > Allow public client flows: Yes. Save.
5. API permissions > Add a permission > Microsoft Graph > Delegated:
   add `Files.ReadWrite.AppFolder` and `offline_access`.
6. Add `ride.onedriveClientId=<client ID>` to `~/.gradle/gradle.properties`, then `just install`.
7. In the app: Sign in to OneDrive, open the URL shown on any device, enter the code, approve.

Sessions then upload on WiFi to OneDrive `Apps/Ride Logger/sessions/`,
each file checked against OneDrive's `quickXorHash`; nothing is deleted from the phone.

### 6. GitHub Actions (when you want CI builds)

CI builds and tests on x86-64 runners and attaches the APK to a release for each `v*` tag.
It needs the same signing key, as repository secrets (Settings > Secrets and variables > Actions):

| Secret | Value |
|---|---|
| `RIDE_SIGNING_KEYSTORE_BASE64` | `base64 -w0 ~/.config/ride-logger/signing.jks` |
| `RIDE_SIGNING_STOREPASSWORD` | `ride.signing.storePassword` |
| `RIDE_SIGNING_KEYALIAS` | `ride` |
| `RIDE_SIGNING_KEYPASSWORD` | `ride.signing.keyPassword` |

Optionally the variable (not secret) `RIDE_ONEDRIVE_CLIENT_ID`.
CI fails if the APK isn't signed with the shared key's certificate.

## Development

```sh
just test      # Rust lints and tests, Android unit tests, Kotlin-to-Rust/Python format cross-checks
just apk       # debug APK
just install   # install over adb
```
