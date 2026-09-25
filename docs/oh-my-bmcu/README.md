# oh-my-bmcu

A fork of [jarczakpawel/BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK) maintained for one concrete setup: a single BMCU 370C on a Bambu Lab A1. Every change lands with an adversarial code review, a green CI and host unit tests for the logic that can be isolated from the hardware, and stays easy to send back upstream. Validation on the A1 itself is tracked in the [hardware test plan](hardware-test-plan.md), with results in the [hardware test log](hardware-test-log.md): until an image has passed it, treat it as untested on hardware.

## Target setup

| | |
|---|---|
| Printer | Bambu Lab A1 (model code N2S), printer firmware 01.08.01.00, AMS type set to **AMS** (not AMS Lite) |
| BMCU | one BMCU 370C with Hall sensors |
| Firmware variant | `standard(A1)` load force, SOLO slot (AMS A) with 0.095 m retract, AUTOLOAD, filament RGB off |
| PlatformIO env | `a1_solo_autoload_rgboff` (the default env) |
| Slicer | [OrcaStudio](https://github.com/jarczakpawel/OrcaStudio): on A1 firmware 01.08.x it ignores the blocking print error `0500-409D` and retries the print |

## Differences from upstream

Findings and their status are in the [audit backlog](audit-2026-09.md), the list of commits in the [changelog](CHANGELOG.md). In short:

**Bus with the printer**
- Filament info is saved again. Upstream PR [#134](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/pull/134) (Murilo Bastos) throttled LED updates to 10 ms, verified on the A1 on 2026-09-23. The root cause is also fixed: the active channel's LED strip is redrawn only when its colour actually changes, instead of 100 times a second with interrupts off.
- The RX parser resynchronises after a lost byte (a 200 us gap on the line or a UART overrun) instead of losing the next frame too, and is reset around our own replies.
- Flash writes wait for a quiet bus window instead of running while a reply is still on the wire. Filament changes are written 500 ms after the last change to a slot, then in the next quiet window, so wait about 2 s per edited slot before switching the printer off.
- A lost printer link is detected reliably (it used to depend on a 119 s timer phase), so the motors stop within about 1 s of the printer going silent.
- The set-filament handlers check frame lengths, and AMS discovery is answered again after a link loss.

**Motor and sensors**
- Tangle detection: the buffer must stay below 40% for 500 ms while the BMCU pushes at full force before the print pauses (it used to trip on a few milliseconds). The latch is meant to release when the printer resumes the channel after the buffer has been above 40% at least once (free the spool, feed filament, resume), or when the buffer is held at or above about 52% for 1 s while printing or paused. 52% is only a few mV above the buffer's rest position, inside its rest scatter, so a buffer that rests slightly high can release the latch by itself; a resume then pauses again 0.5 s after the buffer drops below 40%. Which commands the A1 sends on a resume has not been observed yet, so this is still to be confirmed on the printer. Full-force pushing is still capped at 20 s of push time, at any buffer level (the anti-stall's 0.5 s rests pause the count, so a stalled gear brakes after about 32 s). While latched, the BMCU does not push the channel, whatever the printer commands next: before_on_use and the idle control (once another channel or none is active) are braked, and unloads still only retract. Lifting a latched channel's buffer never starts the auto-unload: held at or above 85% for 1 s it releases the latch, and letting go afterwards does nothing, also if the buffer sagged (without reaching the middle) and was lifted again while held. Once the buffer is back in the middle of its travel (below 55%), a new lift unloads as usual; if it stops above that when let go, press it down to the middle once first.
- Unload pull back, redetect and the DM autoload Stage-2 have time, distance and stall limits. A pull that stalls (for example filament still held by the extruder) stops after about 1.6 s, and that channel's LED blinks red (1 s on, 1 s off) until the filament is pulled out or the slot is used again.
- Failed or impossible AS5600 readings are skipped instead of being used as angle 0, and the motor-direction test at first boot re-reads and confirms before saving.
- Every motion distance (unload length, autoload Stage-2) comes from an integer AS5600 count, so unloads stay 95 mm however much filament has gone through since boot (the float odometer used to lose precision after about 128 m).
- DM autoload only re-arms its 120 mm Stage-2 push after the filament really left both switches or was retracted, not after a switch flickers. A flickering switch no longer restarts the push either: the remaining length, the abort count and the time and stall limits carry over, so with the printer link up a gear that cannot turn gets at most about 6 s of push per insertion, whatever the buffer or buffer-lift unloads do (a switch that reads empty for a single pass ends the insertion; see the open points in the audit).
- While the printer link is down (SYS LED red), lifting the buffer no longer drives the motor: the auto-unload and the empty-channel pull only run with the link up.

**Watchdog and faults**
- An independent watchdog (1 s nominal, 0.67 to 1.6 s over the LSI tolerance) resets the BMCU if the firmware hangs. It starts after the first-boot calibration. After a watchdog reset the SYS LED flashes magenta three times at boot.
- A CPU trap (illegal instruction, bad memory access) now brakes all motors and releases the bus at once, then the watchdog resets the chip. Before, the motors kept their last PWM until the printer was power-cycled.

**Upstream #148 (A1 no longer finds the AMS after a power cycle with a channel loaded)**
- The cause is unproven. A channel the BMCU restored as in use at boot is now reset to idle as soon as its switches read empty.
- `env:a1_solo_autoload_rgboff_no_boot_restore` (`-DBMCU_BOOT_RESTORE_LOADED=0`) is an A/B image that boots every channel idle and applies the restored state only when the printer first references the channel. See the A/B procedure in the [hardware test plan](hardware-test-plan.md#6-upstream-148-ab-test).

**Calibration and flash**
- Calibration averages in float instead of double, which removes the soft-double routines (3,752 bytes) and saves 3,828 bytes of flash on the A1 image.
- A calibration step that times out, or a buffer moved less than the minimum span, gets a safe default range instead of an oversensitive one. Those channels flash red quickly at the end of calibration and for about 0.7 s at every boot until they are recalibrated.
- A flash journal page that holds a torn record or data from an older firmware layout is erased before writing, instead of making every save of that slot fail.

**Tooling**
- Pinned platform and SDK revisions, an explicit env for our variant, validation of the `env:fw` environment variables, a fixed `build_all_firmwares.sh`, host unit tests (`test/`), and CI.

## Build

```sh
pip install platformio==6.1.19
pio run                      # builds env:a1_solo_autoload_rgboff
ls .pio/build/a1_solo_autoload_rgboff/firmware.bin
```

Host unit tests for the logic that can run off the hardware (Unity, `test/`). `scripts/check_test_copies.py` checks that the firmware code copied verbatim into tests still matches `src/`, and that no test redefines a `src/` constant or macro:

```sh
python3 scripts/check_test_copies.py
bash scripts/test_build_all_firmwares.sh   # build_all_firmwares.sh OUT_DIR guards, stub pio
pio test -e native           # CI runs the same checks with GCC on Linux
```

Other variants:

- `env:fw` reads the variant from environment variables. They are validated before compiling (`scripts/check_fw_env.py`):
  `BMCU_DM_TWO_MICROSWITCH`, `BMCU_ONLINE_LED_FILAMENT_RGB`, `DBMCU_P1S`, `BMCU_SOFT_LOAD` (0/1), `BAMBU_BUS_AMS_NUM` (0-3) and `AMS_RETRACT_LEN` (metres, e.g. `0.095f`).
- `./build_all_firmwares.sh` builds the whole published matrix (780 images) into `build/firmwares/`. `OUT_DIR` overrides the output dir and is replaced as a whole: the script refuses one that is the checkout, `$HOME` or a directory above either, and, unless `FORCE=1`, one that exists and is neither empty nor an earlier output of the script (its `manifest.txt` starts with the script's header). `BUILD_ONLY_SOLO=1` builds only the 12 SOLO images.
  - `OUT_DIR=firmwares` regenerates the upstream mirror in place (`BUILD_ONLY_SOLO=1` refuses it however it is spelled or reached, and any directory that contains it, such as `.`). Only do that with upstream sources. The script replaces the whole directory and writes `which_to_choose.txt` guides, so the 22 hand-written `README.md` guides of the published tree are deleted; `git checkout -- firmwares` restores them.

## Flash

Use the CLI of [BMCU-Flasher](https://github.com/jarczakpawel/BMCU-Flasher) (v1.3). It needs `pip install pyserial`. Its GUI calls the same `flash_firmware()` function.

1. Remove all filament from the BMCU. Unplug the printer from the wall and disconnect the BMCU from the printer. Then connect the BMCU to the computer over USB-C. It shows up as a CH340, `1a86:7523`.
2. From the oh-my-bmcu checkout, run:

   ```sh
   python3 /path/to/BMCU-Flasher/bmcu_flasher.py "$PWD/.pio/build/a1_solo_autoload_rgboff/firmware.bin" --mode usb --verify-last
   ```

   The port is picked by VID/PID. Every block is read back after programming. Without `--verify-last` the CLI skips the last block; the GUI always checks it.
3. Flashing erases all 64 KB, including the NVM sector: calibration, motor direction, filament info and the loaded channel. The flasher then resets the MCU, and it boots straight into the first-boot calibration, even on USB power. The calibration samples the idle buffers and then waits up to 30 s per extreme for each buffer to be moved, one buffer at a time (see upstream's [calibration video](https://www.youtube.com/watch?v=Hn_DNzSmhuc)). While a buffer's LED blinks blue, move it to its low end, the end the extruder pulls it to when the filament is tight (the same end you hold to recalibrate), and let it return to rest; while it blinks red, lift it to the other end and let it return. Two yellow blinks confirm each step. The order matters: the first end becomes the low end, so a calibration done the other way round inverts the buffer. A calibration that is cut off is not saved and runs again on the next boot. A step that times out gets a safe default range, and that channel flashes red at the end and at every boot until it is recalibrated. So either do the calibration properly, or disconnect right away and do it at the first boot in the printer.
4. Reconnect the BMCU to the printer only while the printer is unplugged.

To recalibrate later, remove all filament and hold one buffer for about 5 s. This wipes the whole NVM, including every slot's filament type and colour. It works with the printer link up or down.

## Reproducibility

- With the pinned platform (`8cf3b51c`) and SDK (`34b1b783`), the V10.5 sources rebuild **bit for bit** to the published images. The CI job `reproduce-upstream` checks this for the 12 SOLO images on every push.
- The RISC-V toolchain is chosen per host OS by the platform (`toolchain-riscv-{mac,linux,windows}#gcc12`), so it is not pinned in `platformio.ini`. Known-good revisions: `toolchain-riscv-mac` `e6360e0f` (checked locally) and `toolchain-riscv-linux` `fde267f` (CI). CI puts the current `gcc12` head in its cache key and rebuilds V10.5 with it, so Linux toolchain drift fails the build. The macOS and Windows toolchains are only checked when someone rebuilds locally.
- Reference hashes:
  - V10.5 `standard(A1)/AUTOLOAD/FILAMENT_RGB_OFF/SOLO/solo_0.095f.bin`: `9237be17296e1d3c2a92162a859b1363dafd835df0dd953ffe0a52bdb91231d9`
  - This fork's A1 target at `73497e9` (V10.5 + #134): `052651efe729b611556b0c38badae09fca5e8351e1b7fcbf6fad883ef64596a1`
- This fork's own images are reproducible across hosts too: at `65bbd90` a local build on macOS (`toolchain-riscv-mac` `e6360e0f`) and the CI build on Linux gave the same `a1_solo_autoload_rgboff` (`fb40deab...`) and `a1_solo_autoload_rgboff_no_boot_restore` (`779705a8...`) images.

## Notes

- **`firmwares/` is an upstream mirror.** It holds the images published by upstream for V10.5. It is not built from this fork's sources. BMCU-Flasher's *Online* mode downloads from upstream, not from here. Images built from this fork come from a local `pio run` or from CI: every push that passes the tests uploads `oh-my-bmcu-a1-<commit>-push` with the `.bin` and `.elf` of `a1_solo_autoload_rgboff` and of the #148 A/B image, and the run summary lists their sha256.
- **Printer-facing version.** The BMCU reports the AMS version bytes `{0x00, 0x00, 0x32, 0x0A}` + `"AMS08"` (10.50) from `src/bambu_bus_ams.cpp`. The `version` file is not used by the build. We keep upstream's bytes on purpose: printers check the reported AMS version (see upstream [#73](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/issues/73) and [#119](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/issues/119), "the ams version is wrong, printer cannot continue"). Change them only after testing on the A1.
- **License.** Upstream does not declare a license, so this fork does not add one either. Ask upstream before redistributing the firmware.
- The [upstream README](../../README.md), below the fork banner, describes upstream's releases, flashing guide and safety notes. The safety notes apply here too.
