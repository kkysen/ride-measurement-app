# Checklist

Everything you need to do by hand, in order.
Details for each step are in `README.md` under "Your steps".

## Tonight: set up the phone

- [ ] Back up `~/.config/ride-logger/signing.jks` and the `ride.signing.*` lines in `~/.gradle/gradle.properties`
      (e.g. in a password manager). Losing them means the app can't be updated without an uninstall.
- [ ] Phone: Settings > About phone > Software information, tap Build number 7 times (developer options).
- [ ] Phone: Developer options > Wireless debugging: on. Phone and computer on the same WiFi.
- [ ] Phone: Wireless debugging > Pair device with pairing code. Here: `adb pair <ip>:<pairing port>`, enter the code.
- [ ] Here: `adb connect <ip>:<port>` (port from the main Wireless debugging screen).
- [ ] Here: `just install`.
- [ ] Open Ride Logger and grant everything it asks for:
  - [ ] All-files access (keeps recordings safe from uninstalls).
  - [ ] Location: precise, then "Allow location all the time" > Allow all the time.
  - [ ] Notifications, phone, nearby devices, microphone.
  - [ ] "Don't optimize battery for this app" > Allow.
- [ ] Phone: Settings > Battery > Background usage limits > Never sleeping apps: add Ride Logger.
      Check "Put unused apps to sleep" / "Deep sleeping apps" don't include it.
- [ ] Optional: Developer options > Force full GNSS measurements (the app already requests full tracking; this is a backup).
- [ ] Decide on audio: flip "Record audio" on in the app if you want to try it tomorrow (~100 MB/h; records nearby conversations).

## Tonight: 1-minute test at home

- [ ] Check the bottom of the main screen says "Rust core 0.1.0". If it shows an error instead, recording still works; send me the error.
- [ ] Tap Start, walk around for a minute (ideally near a window for GNSS), tap a couple of event buttons, tap Stop.
- [ ] While recording, check the live speed shows (it now runs in Rust). If it shows "Live speed failed" instead, recording still works; send me the error.
- [ ] While recording, tap Map: your path should appear (once GPS has a fix) and the map should follow you.
  If it shows an error instead, recording still works; send me the error.
- [ ] Here: `just pull`, then `just summary data/sessions/<session>`.
- [ ] Here: `just map data/sessions/<session>`, then open `data/sessions/<session>/map.html` in a browser.
      Expect: `accel_uncal`, `gyro_uncal`, `mag_uncal` at a few hundred Hz with 0 gaps, `pressure`, `light`, `proximity`,
      `gnss` and `location` records (if near a window), `radio` records, your events, and `audio` if on.
- [ ] If audio was on: `just wav data/sessions/<session>` and listen to `audio.wav`.
- [ ] Optional restart test: Start, switch to another app, `adb shell am kill dev.khyber.ride`;
      `just pull` + `just summary` should show a second session with `continues session ...`.
- [ ] Tell me anything odd in the summary (missing streams, gaps, low rates) so it can be fixed before the ride.

## Tomorrow: the ride

- [ ] Before entering the station: open the app, tap Start (GNSS gets a last fix before going underground).
- [ ] Keep the app in the background or the screen off; don't swipe it away from recents.
- [ ] Tap events as you go: Boarded, Doors open/closed, At station (type the station name first if you can), Alighted.
      Underground, these are the ground truth everything else is checked against.
- [ ] Use Add note for anything notable (express skipping stations, stopped in tunnel, crowded, phone in pocket vs hand).
- [ ] Note where the phone was (hand, pocket, bag): it changes the IMU data a lot.
- [ ] Glance at the live speed now and then and add a note if it's clearly wrong (e.g. "says 80, train stopped").
      It's a rough display-only estimate; underground it runs on the IMU alone and resets at each stop.
- [ ] After leaving the station: tap Stop.

## After the ride

- [ ] Here: `just pull`, `just summary data/sessions/<session>` for each session.
- [ ] Here: `just map data/sessions/<session>` for each session, and look at `map.html`.
- [ ] If audio was on: `just wav data/sessions/<session>`, listen for rail joints, door chimes, HVAC.
- [ ] Share the summaries (or the session directory) with me to start on analysis.

## Later, whenever

- [ ] OneDrive: Azure app registration, add `ride.onedriveClientId` to `~/.gradle/gradle.properties`, reinstall, sign in in the app.
- [ ] GitHub Actions: add the four `RIDE_SIGNING_*` repository secrets (README step 6), then ask me to push.
      They let CI sign APKs with your one key, so CI builds install over local ones; it means uploading the key to GitHub.
