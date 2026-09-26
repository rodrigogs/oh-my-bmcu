# Hardware test log

Results of the [hardware test plan](hardware-test-plan.md) on the real printer, one section per
run. "pass" means the step was done as written and gave what the plan expects; "pending" means it
has not been run yet.

## Run 1: 2026-09-25, image `cec0e00`

- Image: `env:a1_solo_autoload_rgboff` from CI artifact `oh-my-bmcu-a1-cec0e00…-push`,
  sha256 `1c42d010ac10d6042bc5d3168a2b870e6af42c31f78d3440fced220e59d24d1d` (50,140 bytes), the
  same as a local build of `cec0e00`.
- Setup: Bambu Lab A1 "Tortuguita", printer firmware 01.08.01.00, AMS type "AMS", 0.4 mm
  stainless steel nozzle, Textured PEI plate, PLA white in slot 1.
- Flash: BMCU-Flasher v1.3 CLI, `--mode usb --verify-last`, printer unplugged, BMCU on USB-C
  (CH340 `1a86:7523`): 896/896 blocks programmed and read back, 12.2 s.

| Section | Step | Result |
|---|---|---|
| 0 | sha256 noted; filament out; printer unplugged; BMCU on USB-C | pass |
| 1 | First calibration after the flash | ended with the fast red flash on at least one channel: a step timed out or was too shallow, and that channel got the fallback range, as the plan's fallback step describes |
| 8 | 5 s recalibration hold (on USB-C, offline) | pass: blue blink, NVM wiped, reboot into calibration, no magenta flash |
| 1 | Calibration after the recalibration | pass: every step confirmed, green blink at the end |
| 2 | BMCU on USB-C only: SYS LED red, no heartbeat, for more than 4 minutes | pass |
| 1 | Reconnected with the printer unplugged, then powered on: the printer shows AMS A | pass (checked over the printer's LAN MQTT: `ams_exist_bits` 1, module `ams/0` 10.50.00.00 `AMS08`, only the known cosmetic HMS 0500-0400-0001-0044) |
| 1 | SYS LED red until the first heartbeat, then white; no red flash at boot | pending |
| 1 | Direction check (buffer LED blue at the low end, red when lifted) | pending |
| 2 | Slot 1 set to PLA white from OrcaStudio | pass: the printer reports slot 1 as PLA, white (`GFL99`, 190-240 °C), read back from the BMCU |
| 2 | The slot keeps its type and colour across a printer power cycle | pending |
| 2 | Seven filament changes, then a power cycle (journal page erase) | pending |
| 3 | Test print (the 5 h 27 min `两侧` project, 485 layers, PLA from slot 1) | running: the BMCU loaded slot 1 for every attempt (the printer reported it in the extruder within about 30 s), and the fifth attempt, started 21:26, is printing normally (layer 3 of 485 at 21:42, no print error) |
| 3 | Unloads and reloads between attempts | pass: five unload/reload cycles of slot 1 between 20:41 and 21:26, each confirmed by the printer (filament out of, then back in, the extruder) |
| 3 | Unload at the end of the print, about 95 mm, no red channel LED | pending |
| 4, 5, 7 | Tangle, unload limits, DM autoload | pending |
| 8 | Normal boots and a whole print with no reset (no magenta) | no magenta so far; the print is pending |

Notes:

- The first four print attempts were cancelled (printer error `0300-400C`, task cancelled) because
  of a clogged nozzle, not the BMCU: the user cleared it at 300 °C between attempts. The modified
  OrcaStudio restarted the first cancelled attempt by itself 5 s later. One AMS HMS,
  `0700-4500-0002-0001`, showed for about 6 s at 20:48 during that work and cleared by itself.
- The print was started with parts from an earlier print still on the plate, at the user's choice.

- After the flash erased the NVM, slot 1 read as "PETG, white", the firmware's default, until it
  was set again. This is expected: flashing erases the whole NVM sector.
- The printer got a new DHCP address after the power cycle; tools that remember the old one see it
  as offline.
