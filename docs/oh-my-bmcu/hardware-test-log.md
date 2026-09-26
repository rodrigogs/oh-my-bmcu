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
| 2 | The slot keeps its type and colour across a printer power cycle | pass: after the printer was switched off and on (2026-09-26 02:37) slot 1 still read PLA, white (`GFL99`, 190-240 °C), with AMS A and the filament present; this was the original bug (the slot fell back to "PETG, white") |
| 2 | Seven filament changes, then a power cycle (journal page erase) | pending |
| 3 | Test print (the 5 h 27 min `两侧` project, 485 layers, PLA from slot 1) | running: the BMCU loaded slot 1 at the first attempt (31 s from the printer's request to the filament in the extruder; later attempts started with it loaded), and the fourth attempt, started 21:21, is printing normally (layer 3 of 485 at 21:42, no print error) |
| 3 | Unloads and reloads between attempts | pass: four unload/reload cycles of slot 1 between 20:42 and 21:20, two of them through the external-spool position (tray 254), each confirmed by the printer (filament out of, then back in, the extruder) |
| 3 | A whole print from slot 1: `SpeedBoatRace_Bambu Pla Basic` (Benchy), 192 layers, 02:07-02:25 on 2026-09-26 | pass: every layer printed, `FINISH` with no print error and no new HMS |
| 3 | Unload at the end of that print | pass as far as the printer shows it: unload requested at 02:24 (`tray_tar` 255), filament out of the extruder (`tray_now` 255) within a minute, no error; the length (about 95 mm) and the channel LED (no red blink) were not observed |
| 4, 5, 7 | Tangle, unload limits, DM autoload | pending |
| 8 | Normal boots and a whole print with no reset (no magenta) | no reset seen: the printer kept AMS A through the Benchy (polled every minute); the user saw no magenta flash from the flash to the calibration, and the SYS LED was not watched during the print |

User report (2026-09-26): after the Benchy and the power cycle, the BMCU works well in normal
use. The steps still marked pending are the fault cases (tangle, unload limits, DM autoload
corner cases), the direction check, the journal page erase and the boot LEDs.

Notes:

- The first three print attempts were cancelled (printer error `0300-400C`, task cancelled, at
  20:33, 20:41 and 20:56) because of a clogged nozzle, not the BMCU: the user cleared it at 300 °C
  between attempts. The modified OrcaStudio restarted the first cancelled attempt by itself 5 s
  later. One HMS, `0700-4500-0002-0001`, showed for about 6 s at 20:48 during that work and
  cleared by itself.
- The print was started with parts from an earlier print still on the plate, at the user's choice.
- From 22:21 the user cancelled several more attempts early (again `0300-400C`), at layer 37 and
  then at layers 1 to 4, and the print was started again each time; one restart at 23:56 met the
  known `0500-409D` block, which OrcaStudio got past by starting again. Slot 1 stayed loaded
  throughout, with no BMCU error.
- At 01:04 an unload of slot 1 ended with printer error `0700-8003` ("Failed to pull out the
  filament from the extruder. This might be caused by clogged extruder or filament broken inside
  the extruder.") and HMS `0700-2000-0002-0004` ("AMS A Slot 1 filament may be broken in the tool
  head."): the same clog, with filament stuck in the tool head. After the user cleared it, slot 1
  loaded again and the Benchy printed and unloaded normally.
- The `0700-4500-0002-0001` HMS seen at 20:48 is the printer's filament cutter sensor ("The
  filament cutter sensor is malfunctioning"), not the BMCU.
- After the flash erased the NVM, slot 1 read as "PETG, white", the firmware's default, until it
  was set again. This is expected: flashing erases the whole NVM sector.
- The printer got a new DHCP address after the power cycle; tools that remember the old one see it
  as offline.
- Over the printer's LAN MQTT the BMCU's state can be read (slots, loaded slot, AMS HMS), but
  commands are refused on this firmware: an unsigned `ams_filament_setting` from a script was
  rejected with HMS `0500-0500-0001-0007` ("MQTT Command verification failed") and changed
  nothing. Slot changes, loads and unloads go through OrcaStudio (signed by the network plugin) or
  the printer screen.
