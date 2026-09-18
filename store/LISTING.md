# Chrome Web Store listing copy

Everything below is ready to paste into the developer console.

---

## Item name (45 char max)

```
Flight Tracker for Google Earth
```

> Named this way on purpose. Chrome Web Store branding rules do not allow a third-party
> trademark to lead the item name, so "Google Earth Flight Tracker" would likely be rejected.

## Summary (132 char max)

```
Adds an aircraft, a glass cockpit panel and route recording to the Google Earth flight simulator. Export flights as KML or GPX.
```

## Category

`Just for Fun` (alternate: `Travel`)

## Language

English (United States)

## Description

```
Google Earth's web flight simulator gives you a wireframe HUD and nothing else: no aircraft,
no instruments, and no record of where you flew. This adds all three.

EXTERIOR VIEW
A light aircraft rendered over the simulator in a chase camera that lags behind the real
attitude, the way a proper flight sim's external view behaves. The ailerons, elevator and
rudder deflect with your actual control inputs and the propeller spins.

COCKPIT VIEW
A six-pack glass panel: airspeed indicator, artificial horizon, altimeter, turn coordinator,
heading indicator and vertical speed indicator, plus a windscreen frame. The airspeed and
vertical speed dials pick their own range automatically, because Google Earth's simulator is
quite happy to fly at jet speeds.

FLIGHT RECORDING
Every flight is logged with a live route map, distance, duration, maximum altitude and top
speed. Export any flight as KML, GPX or GeoJSON. Open the KML back in Google Earth and your
route is drawn in 3D at true altitude with takeoff and end markers, plus an animated track
you can play back. The layer stays visible inside the simulator, so you can fly an old route
again.

CONTROLS
V - cycle exterior / cockpit / off
Shift+R - start or stop recording
Shift+P - pause or resume

PRIVACY
No account, no sign-in, no network requests, no analytics. Everything is computed in the page
and stored in your own browser. The extension runs only on earth.google.com and requests no
Chrome permissions at all.

Open source, MIT licensed:
https://github.com/twentyseventhllc-lgtm/google-earth-flight-tracker
```

## Single purpose

```
Enhance the Google Earth web flight simulator with an aircraft model, flight instruments, and
recording of the flown route for export.
```

## Host permission justification (earth.google.com)

```
The extension only runs on earth.google.com because that is where the flight simulator lives.
It reads the simulator's current aircraft position and attitude in order to draw the aircraft
and instruments and to record the route. It does not read or transmit any page content, user
data or account information, and makes no network requests.
```

## Remote code

`No, I am not using remote code` - both scripts are contained in the package.

## Data usage disclosures

- Does not collect or use personally identifiable information
- Does not collect or use health information
- Does not collect or use financial or payment information
- Does not collect or use authentication information
- Does not collect or use personal communications
- Does not collect or use location **(note: flight-simulator coordinates only - they are
  positions inside a simulation, not the user's real-world location, and never leave the
  browser)**
- Does not collect or use web history
- Does not collect or use user activity
- Does not collect or use website content

Certifications: does not sell data, does not use or transfer data for unrelated purposes,
does not use or transfer data to determine creditworthiness or for lending.

## Images

| Asset | File | Size |
| --- | --- | --- |
| Icon | `icons/icon128.png` (in the package) | 128x128 |
| Screenshot 1 | `store/01-exterior.png` | 1280x800 |
| Screenshot 2 | `store/02-cockpit.png` | 1280x800 |
| Screenshot 3 | `store/03-banking.png` | 1280x800 |
| Screenshot 4 | `store/04-route-3d.png` | 1280x800 |
| Screenshot 5 | `store/05-route-in-sim.png` | 1280x800 |
| Small promo tile | `store/promo-440x280.png` | 440x280 |

## Package

`flight-tracker-for-google-earth-v1.1.0.zip` - manifest, `tracker.js`, `sim.js`, four icons.
No permissions requested, no remote code, ~80 KB.
