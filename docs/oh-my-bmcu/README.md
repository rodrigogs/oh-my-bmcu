# oh-my-bmcu

A fork of [jarczakpawel/BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK) maintained for one concrete setup: a single BMCU 370C on a Bambu Lab A1. Every change lands with an adversarial code review, a green CI and host unit tests for the logic that can be isolated from the hardware, and stays easy to send back upstream. Validation on the A1 itself is tracked in the [hardware test plan](hardware-test-plan.md), with results in the [hardware test log](hardware-test-log.md): until an image has passed it, treat it as untested on hardware. Run 1 exercised the image of `cec0e00`; no image with the fixes that followed it (from `08975e1` on) is byte-identical to it.

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
- The motion (0x03), motion-status (0x04) and online-detect (0x05) handlers also drop a frame too short for the fields they read (11, 12, and 8 or 26 bytes), instead of taking the rest from an older frame in the RX buffer. The 0x03 and 0x04 minima are one byte below the printer frames quoted in the source (this A1's were not captured); no capture of any 0x05 frame exists yet.
- AHUB (the 0x33 protocol of other printers; the A1 only speaks BambuBus): a set request is applied only if its data lies inside the frame, the active channel is stored only as a channel (0-3) or none, and frames without the short header are dropped.

**Motor and sensors**
- Tangle detection: the buffer must stay below 40% for 500 ms while the BMCU pushes at full force before the print pauses (it used to trip on a few milliseconds). The latch is meant to release when the printer resumes the channel after the buffer has been above 40% at least once (free the spool, feed filament, resume), or when the buffer is held at or above about 52% for 1 s while printing or paused. 52% is only a few mV above the buffer's rest position, inside its rest scatter, so a buffer that rests slightly high can release the latch by itself; a resume then pauses again 0.5 s after the buffer drops below 40%. Which commands the A1 sends on a resume has not been observed yet, so this is still to be confirmed on the printer. Full-force pushing is still capped at 20 s of push time, at any buffer level (the anti-stall's 0.5 s rests pause the count, so a stalled gear brakes after about 32 s). While latched, the BMCU does not push the channel, whatever the printer commands next: before_on_use and the idle control (once another channel or none is active) are braked, and unloads still only retract. Lifting a latched channel's buffer never starts the auto-unload: held at or above 85% for 1 s it releases the latch, and letting go afterwards does nothing, also if the buffer sagged (without reaching the middle) and was lifted again while held. Once the buffer is back in the middle of its travel (below 55%), a new lift unloads as usual; if it stops above that when let go, press it down to the middle once first.
- Unload pull back, redetect and the DM autoload Stage-2 have time, distance and stall limits. A pull that stalls (for example filament still held by the extruder) stops after about 1.6 s, and that channel's LED blinks red (1 s on, 1 s off) until the filament is pulled out or the slot is used again.
- The load (the printer's send_out) stops on a gear that does not turn: 800 PWM or more with less than 1 mm of travel in 1 s ends it, about 1.7 s after it starts, or 1 s after the gear stops if it was feeding. The 10 m cap stays, now in the same limit. The channel stays braked until the printer's next command for it. No LED changes: the printer's own load timeout reports it. So a snag that holds the gear for 1 s at full force now ends the whole load.
- The buffer-lift auto-unload (850 PWM, up to 15 s) stops after 1 s on a gear that does not turn, and the channel's LED then blinks red as after a stalled pull back. Its 15 s end sets no fault. The manual empty pull (700 PWM, no filament at the switches, buffer held up) has no stall limit.
- The idle control's push (a parked or idle channel with the buffer below 30%, about 510 to 800 PWM; the 420 PWM hold floor is the stall level) is braked after 1 s with the gear stalled (420 PWM or more, less than 1 mm) or after 10 s of pushing. At a rounded 28-29% the brake is silent (a gear that stops just short of 30%); below 28% the status LED turns steady red. The brake ends when the buffer is back at 30% or above, when the printer moves the channel, or when no filament is at the switches; if the buffer falls to a rounded 26% or less during a silent brake, a new push starts (a one-point dip to 27% keeps the brake, so ADC noise does not push a stopped gear again). A steady red (this fault, or a failed DM autoload) hides the unload-fault blink on the same channel until its next move.
- If the ADC readings stop coming for more than 100 ms (the buffer and filament-switch voltages would be frozen), every motor is braked, a running auto-unload ends, the tangle latch neither trips nor releases (also not through the load's release above 85%) and its 500 ms trip and 1 s release timers start again from the first fresh reading, the 5 s recalibration hold starts again from the first fresh reading, and every channel's status LED blinks pure blue (250 ms on, 250 ms off, all four together; a channel with an AS5600 fault stays steady red instead) with the buffer LEDs off. The brake does not change what the channel was doing, so a channel braked by the 20 s full-force limit, a load stopped by its stall limit and an idle push braked by its limit are still braked once the readings are back (a printer command for the channel during the stop still ends them as usual), and a load or unload does not replay its soft start. Everything resumes from the first new reading. The printer keeps getting the last filament state and pressure. While the readings are stale the ADCs and the DMA are restarted every 500 ms, and right after a DMA transfer error (the readings used to be 0 V from then until a reboot); the readings keep their last values until the first new one. Each ADC's two calibration waits (four in all, at boot and at every restart) give up after 10 ms each instead of hanging the boot at the dim red SYS LED; a timeout sets `g_adc_cal_timed_out`, readable only with a WCH-Link. If the ADC delivers nothing at boot, the blue blink shows from the first pass and no restart is tried.
- Failed or impossible AS5600 readings are skipped instead of being used as angle 0, and the motor-direction test at first boot re-reads and confirms before saving.
- Every motion distance (unload length, autoload Stage-2) comes from an integer AS5600 count, so unloads stay 95 mm however much filament has gone through since boot (the float odometer used to lose precision after about 128 m).
- DM autoload only re-arms its 120 mm Stage-2 push after the filament really left both switches or was retracted, not after a switch flickers. A flickering switch no longer restarts the push either: the remaining length, the abort count and the time and stall limits carry over, so with the printer link up a gear that cannot turn gets at most about 6 s of push per insertion, whatever the buffer or buffer-lift unloads do. An insertion ends only after both switches have read empty for 100 ms: a shorter glitch stops the drive while it lasts, then Stage-1 or Stage-2 goes on where it was, so a loose key wire no longer restarts the 5 s Stage-1 push or a new 120 mm Stage-2. A failed channel stays red through such a glitch, and its red (and the buffer-press gesture's lock-out) ends up to 100 ms after the filament is pulled out. Key readings more than 20 ms old (a stopped ADC stream) do not count: the 100 ms starts again from the first fresh reading, so a stream that stops right on a glitch no longer unloads a loaded channel and pushes it another 120 mm once the readings are back. The printer still sees the switches' raw state at once.
- While the printer link is down (SYS LED red), lifting the buffer no longer drives the motor: the auto-unload and the empty-channel pull only run with the link up.

**Watchdog and faults**
- An independent watchdog (1 s nominal, 0.67 to 1.6 s over the LSI tolerance) resets the BMCU if the firmware hangs. It starts after the first-boot calibration. After a watchdog reset the SYS LED flashes magenta three times at boot. The boot's channel detection feeds it before each of its 16 ADC samples, so ADC restarts there (a DMA transfer error on every read with a stuck calibration bit, about 42 ms each) cannot add up to a reset at every boot.
- A CPU trap (illegal instruction, bad memory access) now brakes all motors and releases the bus at once, then the watchdog resets the chip. Before, the motors kept their last PWM until the printer was power-cycled. After such a reset one blue flash follows the three magenta ones (490 ms instead of 420 ms), so the LED tells a trap from a hang.
- A fault record in the chip's backup registers survives resets: the reset flags that started the run, the resets in a row since the last power-on, what the main loop was doing (its phase), its longest pass, and for a trap mepc and mcause. The LED shows only hang or trap; the rest needs a WCH-Link (`mdw 0x40006c04 10` reads the live record, one register per 32-bit word with the low 16 bits valid; `g_blackbox_last` in RAM holds the run before the reset). A trap before the record is set up (startup code, SystemInit, the clock setup) is not recorded. The boot clears the record's magic first and writes it last, so a reset during that write leaves no record (the count of resets in a row starts again at 1), never an earlier run's trap. The board probably ties VBAT to VDD, so a power cycle likely clears the record (unconfirmed). The ADC calibration timeout and the flash-write give-up counts are not in the record (all 10 registers are used); they are in RAM (`g_adc_cal_timed_out`, `g_state_job`, `g_fil_job`) for a debugger.

**Upstream #148 (A1 no longer finds the AMS after a power cycle with a channel loaded)**
- The cause is unproven. A channel the BMCU restored as in use at boot is now reset to idle as soon as its switches read empty.
- `env:a1_solo_autoload_rgboff_no_boot_restore` (`-DBMCU_BOOT_RESTORE_LOADED=0`) is an A/B image that boots every channel idle and applies the restored state only when the printer first references the channel. See the A/B procedure in the [hardware test plan](hardware-test-plan.md#6-upstream-148-ab-test).

**Calibration and flash**
- Calibration averages in float instead of double, which removes the soft-double routines (3,752 bytes) and saves 3,828 bytes of flash on the A1 image.
- A calibration step that times out, or a buffer moved less than the minimum span, gets a safe default range instead of an oversensitive one. Those channels flash red quickly at the end of calibration and for about 0.7 s at every boot until they are recalibrated.
- A flash journal page that holds a torn record or data from an older firmware layout is erased before writing, instead of making every save of that slot fail.
- A flash write that keeps failing is retried after 500 ms, then after 5 s, then left alone until the next change to that slot (or the next load or unload), instead of every 500 ms for good: a failing page used to get 7,200 (filament slot) or 14,400 (loaded channel) erases an hour, so a page rated for about 10,000 erases wore out in 83 or 42 minutes. The change stays in RAM meanwhile; a power cycle before the next change brings back the saved value.
- A loaded-channel record torn by a power loss is skipped: the next save goes into the next erased slot instead of erasing the page that holds the newest record, so a reset during that save no longer brings back an older loaded channel. A loaded-channel save that fails again after its page erase no longer counts as saved, so loading that channel again writes it (before, the write was skipped and the next power cycle brought back an older channel).

**Tooling**
- Pinned platform and SDK revisions, an explicit env for our variant, validation of the `env:fw` environment variables, a fixed `build_all_firmwares.sh`, host unit tests (`test/`), and CI.
- `src/` builds with `-Wall -Wextra -Wundef -Werror`; `pio check` (cppcheck) runs in CI and fails on any high or medium finding; the host tests also run under ASan and UBSan (`env:native_asan`); CI compiles `env:moj` and three `env:fw` variants (soft load, P1S, filament RGB) besides the two A1 images, gates the free RAM at 4 KiB, pins every GitHub action to a commit, and stops any job after 20 minutes.
- The host tests include golden frames of the BambuBus replies (the real `bambu_bus_ams.cpp` compiled on the host), a fuzz of the RX parser, and the offline decision, fault record, flash journal records and every new motion limit. The test models of firmware code are locked to a hash of the `src/` code they model, and the guard scripts have their own tests.

## Build

```sh
pip install platformio==6.1.19
pio run                      # builds env:a1_solo_autoload_rgboff
ls .pio/build/a1_solo_autoload_rgboff/firmware.bin
```

`src/` is compiled with `-Wall -Wextra -Wundef -Werror`, so a new warning fails the build. Static analysis with cppcheck, over `src/` only (CI fails on a high or medium finding; the low ones are listed in its summary):

```sh
pio check -e a1_solo_autoload_rgboff --fail-on-defect=high --fail-on-defect=medium
```

Host unit tests for the logic that can run off the hardware (Unity, `test/`). `scripts/check_test_copies.py` checks that the firmware code copied verbatim into tests still matches `src/`, that no test redefines a `src/` constant or macro, and that every `test_*` case is passed to `RUN_TEST()`:

```sh
python3 scripts/check_test_copies.py
python3 -m unittest discover -s scripts/tests   # the guard scripts' own fixtures, standard library only
bash scripts/test_build_all_firmwares.sh   # build_all_firmwares.sh OUT_DIR guards, stub pio
pio test -e native           # CI runs the same checks with GCC on Linux
pio test -e native_asan      # the same suites with ASan and UBSan
```

- A test model marked `adapted from` with `// ---- anchor: <symbol>[ from /<regex>/][ to /<regex>/] ----` lines under it (for example `// ---- anchor: motor_motion_switch ----`) is locked to a sha256 of the `src/` region it names, in `test/adapted.lock`. When a commit changes such a region, the check fails and names the harnesses that model it: review them against the new code, then re-lock with `python3 scripts/check_test_copies.py --relock`. A marker without an anchor is accepted and counted as unlocked; add an anchor to every new one. `Motion_control_run` and `motor_motion_switch` are locked whole, so most edits there need a review and a re-lock; the calls that feed the send, idle push, pull back and redetect guards are locked too. A lock only says the models were looked at, not that they are right.
- The `test/` root holds shared headers only (`bus_rx_frames.h`): PlatformIO puts `test/` on every suite's include path, and a `.c` or `.cpp` there would be built into every suite.

Other variants:

- `env:fw` reads the variant from environment variables. They are validated before compiling (`scripts/check_fw_env.py`):
  `BMCU_DM_TWO_MICROSWITCH`, `BMCU_ONLINE_LED_FILAMENT_RGB`, `DBMCU_P1S`, `BMCU_SOFT_LOAD` (0/1), `BAMBU_BUS_AMS_NUM` (0-3) and `AMS_RETRACT_LEN` (metres, e.g. `0.095f`). CI compiles `env:moj` and three `env:fw` variants, each with one of `BMCU_SOFT_LOAD`, `DBMCU_P1S` and `BMCU_ONLINE_LED_FILAMENT_RGB` at 1, as the last step of its A1 job: an `env:fw` build with other variables makes PlatformIO delete the whole `.pio/build/` tree, so run it after anything that reads the A1 images there.
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
3. Flashing erases all 64 KB, including the NVM sector: calibration, motor direction, filament info and the loaded channel. The flasher then resets the MCU (not a watchdog reset: no magenta or blue flash on the SYS LED), and it boots straight into the first-boot calibration, even on USB power. The calibration samples the idle buffers and then waits up to 30 s per extreme for each buffer to be moved, one buffer at a time (see upstream's [calibration video](https://www.youtube.com/watch?v=Hn_DNzSmhuc)). While a buffer's LED blinks blue, move it to its low end, the end the extruder pulls it to when the filament is tight (the same end you hold to recalibrate), and let it return to rest; while it blinks red, lift it to the other end and let it return. Two yellow blinks confirm each step. The order matters: the first end becomes the low end, so a calibration done the other way round inverts the buffer. A calibration that is cut off is not saved and runs again on the next boot. A step that times out gets a safe default range, and that channel flashes red at the end and at every boot until it is recalibrated. So either do the calibration properly, or disconnect right away and do it at the first boot in the printer.
4. Reconnect the BMCU to the printer only while the printer is unplugged.

To recalibrate later, remove all filament and hold one buffer for about 5 s. This wipes the whole NVM, including every slot's filament type and colour. It works with the printer link up or down (not while the ADC readings are stale, see below).

LED patterns that mean a fault, besides the calibration's own:

- SYS LED at boot, three magenta flashes (70 ms on, 70 ms off): the watchdog reset the BMCU (a hang, or a CPU trap). One blue flash right after them: the run before that reset ended in a CPU trap. A power-on, the flasher's reset and the recalibration's reboot show neither.
- Every channel's status LED blinking pure blue together (250 ms on, 250 ms off) with the buffer LEDs off, except a channel with an AS5600 fault, which stays steady red: the ADC readings are stale and every motor is braked. The blue blinks of the calibration and of the recalibration hold are on the buffer LEDs, so neither looks like this.
- A channel's status LED steady red: a tangle, a failed DM autoload, an AS5600 fault, the 20 s full-force limit, or an idle push stopped by its 1 s stall or 10 s limit. Blinking red, 1 s on and 1 s off: an unload or buffer-lift auto-unload stopped by its limit.

## Reproducibility

- With the pinned platform (`8cf3b51c`) and SDK (`34b1b783`), the V10.5 sources rebuild **bit for bit** to the published images. The CI job `reproduce-upstream` checks this for the 12 SOLO images on every push.
- The RISC-V toolchain is chosen per host OS by the platform (`toolchain-riscv-{mac,linux,windows}#gcc12`), so it is not pinned in `platformio.ini`. Known-good revisions: `toolchain-riscv-mac` `e6360e0f` (checked locally) and `toolchain-riscv-linux` `fde267f` (CI). CI puts the current `gcc12` head in its cache key and rebuilds V10.5 with it, so Linux toolchain drift fails the build. The macOS and Windows toolchains are only checked when someone rebuilds locally.
- Reference hashes:
  - V10.5 `standard(A1)/AUTOLOAD/FILAMENT_RGB_OFF/SOLO/solo_0.095f.bin`: `9237be17296e1d3c2a92162a859b1363dafd835df0dd953ffe0a52bdb91231d9`
  - This fork's A1 target at `73497e9` (V10.5 + #134): `052651efe729b611556b0c38badae09fca5e8351e1b7fcbf6fad883ef64596a1`
- This fork's own images are reproducible across hosts too: at `65bbd90` a local build on macOS (`toolchain-riscv-mac` `e6360e0f`) and the CI build on Linux gave the same `a1_solo_autoload_rgboff` (`fb40deab...`) and `a1_solo_autoload_rgboff_no_boot_restore` (`779705a8...`) images, at `05f35d4` the same `a1_solo_autoload_rgboff` (`cb5f86ddc984d56121f71e7a2c929e760d5db358e0d84f821b1cfb34526b22be`), and at `6d678c6` the same `a1_solo_autoload_rgboff` (`642dfb208ab8c7b7b2e5486d5a54353e74a617c0963fd90a5cd1967b7dea2857`) and `a1_solo_autoload_rgboff_no_boot_restore` (`fa3174ae30d43f45471b83c21c31805ed4f87723d441c2bffe2be95883c2b840`) images.

## Notes

- **`firmwares/` is an upstream mirror.** It holds the images published by upstream for V10.5. It is not built from this fork's sources. BMCU-Flasher's *Online* mode downloads from upstream, not from here. Images built from this fork come from a local `pio run` or from CI: every push that passes the tests and the static analysis uploads `oh-my-bmcu-a1-<commit>-push` with the `.bin` and `.elf` of `a1_solo_autoload_rgboff` and of the #148 A/B image, and the run summary lists their sha256.
- **Printer-facing version.** The BMCU reports the AMS version bytes `{0x00, 0x00, 0x32, 0x0A}` + `"AMS08"` (10.50) from `src/bambu_bus_ams.cpp`. The `version` file is not used by the build. We keep upstream's bytes on purpose: printers check the reported AMS version (see upstream [#73](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/issues/73) and [#119](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/issues/119), "the ams version is wrong, printer cannot continue"). Change them only after testing on the A1.
- **License.** Upstream does not declare a license, so this fork does not add one either. Ask upstream before redistributing the firmware.
- The [upstream README](../../README.md), below the fork banner, describes upstream's releases, flashing guide and safety notes. The safety notes apply here too.
