# Railroader Autopilot

A mod for [Railroader](https://store.steampowered.com/app/1689160/Railroader/) that automates car pickups and deliveries. Select a locomotive, press a button, and the mod delivers each waybilled car to its destination track or picks up cars from nearby sidings automatically.

## Features

- **Automatic delivery** — delivers cars tail-first to their waybill destinations, including cars and locos for sale or repair
- **Destination grouping** — consecutive cars going to the same track are delivered together
- **Runarounds** — when the loco is on the wrong side, it detaches, goes around via a passing siding, and recouples on the other end
- **Loop detection** — finds the nearest passing siding or wye that fits the train for runarounds
- **Train splitting** — when the train is too long for a siding, splits the train, delivers what it can, then returns to recouple the rest
- **Smart repositioning** — moves to the nearest loop when a runaround is needed but no loop is available from the current position
- **Pickup mode** — collect cars from nearby sidings and couple them to your train, filtered by destination; skips cars blocked by other rolling stock
- **Auto-refuel** — monitors fuel/water levels and automatically routes to the nearest facility when low (steam and diesel)
- **Parking space** — save a waypoint as a locomotive's "parking space" and return to it with one click
- **Park after delivery** — optionally send the loco to its parking space automatically when deliveries finish
- **Jump to waypoint** — button in the UI to jump the camera to the current waypoint

## Installation

1. Install [Unity Mod Manager](https://www.nexusmods.com/site/mods/21) (UMM)
2. Download the latest release from the [Releases](https://github.com/peterellisjones/railroader-autopilot/releases) page
3. In UMM, select Railroader, then drag the release zip into the "Drop zip file here" area (or use the Install Mod button)
4. The mod should appear in the UMM mod list as "Autopilot"

## Usage

1. Select a locomotive that has cars with waybills (delivery orders)
2. Make sure the Auto Engineer (AE) is enabled on the locomotive
3. Press **Ctrl+N** to open the Autopilot window (keybinding is configurable in UMM settings)
4. Click **Start**

The autopilot will:
- Plan the delivery order (tail cars first)
- Set waypoints to each destination
- Uncouple delivered cars and set their handbrakes
- Perform runarounds when needed (repositioning to a loop if necessary)
- Split long trains to deliver what fits, then return for the rest
- Continue until all waybilled cars are delivered

**Example:** You have 12 cars at Whittier Saw Mill — 4 for Connelly Creek L1, 4 for L2, and 4 for L3. After coupling the loco to the cars, click Start. The autopilot drops off the 4 cars for L1, sets handbrakes and uncouples, then delivers the next 4 to L2, sets handbrakes and uncouples. For L3, it brings the remaining cars to the nearest runaround, performs the runaround, delivers the last 4 into L3, sets handbrakes, uncouples, and backs away.

### Pickup Mode

1. Select the **Pickup** tab in the Autopilot window
2. Choose a destination from the dropdown (shows destinations with reachable waybilled cars)
3. Click **Start Pickup**

The autopilot will drive to each car, couple it, and continue until all reachable cars for that destination are collected.

### Auto-Refuel

The autopilot can automatically refuel and rewater locomotives when their levels get low.

- **Per-locomotive toggle** — enable/disable with the **Refuel when low** checkbox in the Autopilot window
- **Two thresholds** — configurable in UMM settings:
  - *Mid-run* (default 20%) — checks after each delivery/pickup, only interrupts when fuel is genuinely low
  - *Completion* (default 50%) — checks after all work is done before parking, more aggressive since the loco is idle anyway
- **Refuel Now button** — immediately route to the nearest facility regardless of thresholds
- **Steam locos** — handles water and coal separately, prioritizes water (runs out faster), prefers water towers near coal loaders to minimize travel
- **Diesel locos** — routes to the nearest diesel fuel facility
- **Opportunistic refueling** — after refueling one type, if the other type's facility is within 100m and below 90%, tops up automatically

Set either threshold to 0% to disable automatic refueling for that check point.

### Parking Space

Save a location so you can send a locomotive back there later:

1. Set a native waypoint on the map (click where you want the loco to park)
2. Click **Set Park** in the Autopilot window to save that location
3. Later, click **Park** to send the loco back

The parking space is saved per-locomotive and persists across save/load. Enable **Park after delivery** to automatically send the loco to its parking space when all deliveries are complete.

### Controls

| Button | Description |
|--------|-------------|
| **Start** | Begin automatic delivery |
| **Start Pickup** | Begin collecting cars for a destination |
| **Stop** | Pause the autopilot (can resume later) |
| **Retry** | Retry after an error |
| **Jump to WP** | Jump camera to the current waypoint |
| **Refuel Now** | Route to nearest fuel facility and refuel immediately |
| **Set Park** | Save the current waypoint as a parking space |
| **Park** | Send the loco to its saved parking space |

### Status Messages

The UI shows what the autopilot is doing and why:

- **Moving to Sylva Building Supply R1...** — delivering cars to a destination
- **Runaround: routing to PRR 34826...** — loco going around to recouple on the other side
- **Repositioning to loop...** — moving to a passing siding for a runaround
- **Splitting train — dropping 4 car(s)...** — dropping far-end cars to shorten the train
- **Returning to recouple dropped cars...** — picking up previously dropped cars
- **Moving to pick up PRR 34826...** — driving to a car for pickup
- **Collecting cars for: Sylva Building Supply** — pickup mode active
- **Moving to water tower...** — routing to a fuel facility
- **Refueling water (65%)...** — filling up at a facility

## How It Works

### Planning

Each time the autopilot needs to decide what to do, it runs through this priority list:

1. **Direct delivery** — can any tail-end cars be pushed directly into their destination siding?
2. **Runaround** — would flipping the train (loco goes to other end) enable deliveries?
3. **Split** — is the train too long? Drop far-end cars and deliver loco-end cars
4. **Reposition** — move to a nearby passing siding where a runaround is possible

### Approach Direction

The mod checks whether the tail car will enter the destination siding correctly (tail-first, not loco-first). It does this by finding the route from the tail's outward end to the destination and checking the direction of travel at each switch.

### Loop Detection

When a runaround is needed, the mod searches for passing sidings (loops) by walking the track graph outward from the train. A loop is two switches connected by two independent routes. The train must fit on at least one branch.

### Train Splitting

When the whole train is too long for a route, the mod groups cars by destination and finds a split point where the shorter consist can deliver the tail group. It checks that the delivery route doesn't pass through switches blocked by the dropped cars.

## Known Limitations

- **Not tested with modded tracks.** This mod has only been tested with the base game's track layout. Custom track mods may produce unexpected behavior, especially with loop detection and approach direction analysis. If you encounter issues on modded track, please report them with logs.

## Requirements

- [Railroader](https://store.steampowered.com/app/1689160/Railroader/)
- [Unity Mod Manager](https://www.nexusmods.com/site/mods/21) 0.31.0+

## Building from Source

1. Clone the repository
2. Copy `Paths.user.example` to `Paths.user` and set `RrInstallDir` to your Railroader installation path
3. Open `Autopilot.sln` in Visual Studio 2022
4. Build — debug builds output directly to the game's Mods folder