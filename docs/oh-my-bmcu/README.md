# oh-my-bmcu

A fork of [jarczakpawel/BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK) maintained for one concrete setup: a single BMCU 370C on a Bambu Lab A1. Every change lands with host unit tests, an adversarial code review and a green CI, and stays easy to send back upstream. Validation on the A1 itself is tracked in the [hardware test plan](hardware-test-plan.md): until an image has passed it, treat it as untested on hardware.

## Target setup

| | |
|---|---|
| Printer | Bambu Lab A1 (model code N2S), printer firmware 01.08.01.00, AMS type set to **AMS** (not AMS Lite) |
| BMCU | one BMCU 370C with Hall sensors |
| Firmware variant | `standard(A1)` load force, SOLO slot (AMS A) with 0.095 m retract, AUTOLOAD, filament RGB off |
| PlatformIO env | `a1_solo_autoload_rgboff` (the default env) |
| Slicer | [OrcaStudio](https://github.com/jarczakpawel/OrcaStudio): on A1 firmware 01.08.x it ignores the blocking print error `0500-409D` and retries the print |

## Differences from upstream

Findings and their status are in the [audit backlog](audit-2026-09.md). In short:

**Bus with the printer**
- Filament info is saved again. Upstream PR [#134](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/pull/134) (Murilo Bastos) throttled LED updates to 10 ms, verified on the A1 on 2026-09-23. The root cause is also fixed: the active channel's LED strip is redrawn only when its colour actually changes, instead of 100 times a second with interrupts off.
- The RX parser resynchronises after a lost byte (a 200 us gap on the line or a UART overrun) instead of losing the next frame too, and is reset around our own replies.
- Flash writes wait for a quiet bus window instead of running while a reply is still on the wire. Filament changes are written 500 ms after the last change to a slot, so wait a second before switching the printer off after editing a slot.
- A lost printer link is detected reliably (it used to depend on a 119 s timer phase), so the motors stop within about 1 s of the printer going silent.
- The set-filament handlers check frame lengths, and AMS discovery is answered again after a link loss.

**Motor and sensors**
- Tangle detection: the buffer must stay below 40% for 500 ms while the BMCU pushes at full force before the print pauses (it used to trip on a few milliseconds). To resume after a tangle, free the spool, feed filament until the buffer is above 40%, then resume on the printer; while paused, holding the buffer at or above about 52% for 1 s also clears the red LED. Continuous full-force pushing is still capped at 20 s.
- Unload pull back, redetect and the DM autoload Stage-2 have time, distance and stall limits. A pull that stalls (for example filament still held by the extruder) stops after about 1.6 s, and that channel's LED blinks red (1 s on, 1 s off) until the filament is pulled out or the slot is used again.
- Failed or impossible AS5600 readings are skipped instead of being used as angle 0, and the motor-direction test at first boot re-reads and confirms before saving.

**Calibration and flash**
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

Host unit tests for the hardware-free modules (Unity, `test/`):

```sh
pio test -e native           # CI runs the same tests with GCC on Linux
```

Other variants:

- `env:fw` reads the variant from environment variables. They are validated before compiling (`scripts/check_fw_env.py`):
  `BMCU_DM_TWO_MICROSWITCH`, `BMCU_ONLINE_LED_FILAMENT_RGB`, `DBMCU_P1S`, `BMCU_SOFT_LOAD` (0/1), `BAMBU_BUS_AMS_NUM` (0-3) and `AMS_RETRACT_LEN` (metres, e.g. `0.095f`).
- `./build_all_firmwares.sh` builds the whole published matrix (780 images) into `build/firmwares/`. `OUT_DIR` overrides the output dir. `BUILD_ONLY_SOLO=1` builds only the 12 SOLO images.
  - `OUT_DIR=firmwares` regenerates the upstream mirror in place. Only do that with upstream sources, and note that the script writes `which_to_choose.txt` guides: the `README.md` guides in the published tree are not in the repo.

## Flash

Use the CLI of [BMCU-Flasher](https://github.com/jarczakpawel/BMCU-Flasher) (v1.3). It needs `pip install pyserial`. Its GUI calls the same `flash_firmware()` function.

1. Remove all filament from the BMCU. Unplug the printer from the wall and disconnect the BMCU from the printer. Then connect the BMCU to the computer over USB-C. It shows up as a CH340, `1a86:7523`.
2. From the oh-my-bmcu checkout, run:

   ```sh
   python3 /path/to/BMCU-Flasher/bmcu_flasher.py "$PWD/.pio/build/a1_solo_autoload_rgboff/firmware.bin" --mode usb --verify-last
   ```

   The port is picked by VID/PID. Every block is read back after programming. Without `--verify-last` the CLI skips the last block; the GUI always checks it.
3. Flashing erases all 64 KB, including the NVM sector: calibration, motor direction, filament info and the loaded channel. The flasher then resets the MCU, and it boots straight into the first-boot calibration, even on USB power. The calibration samples the idle buffers and then waits up to 30 s per extreme for each buffer to be moved (see upstream's [calibration video](https://www.youtube.com/watch?v=Hn_DNzSmhuc)). A calibration that is cut off is not saved and runs again on the next boot. A step that times out gets a safe default range, and that channel flashes red at the end and at every boot until it is recalibrated. So either do the calibration properly, or disconnect right away and do it at the first boot in the printer.
4. Reconnect the BMCU to the printer only while the printer is unplugged.

To recalibrate later, remove all filament and hold one buffer for about 5 s. This wipes the whole NVM, including every slot's filament type and colour.

## Reproducibility

- With the pinned platform (`8cf3b51c`) and SDK (`34b1b783`), the V10.5 sources rebuild **bit for bit** to the published images. The CI job `reproduce-upstream` checks this for the 12 SOLO images on every push.
- The RISC-V toolchain is chosen per host OS by the platform (`toolchain-riscv-{mac,linux,windows}#gcc12`), so it is not pinned in `platformio.ini`. Known-good revisions: `toolchain-riscv-mac` `e6360e0f` (checked locally) and `toolchain-riscv-linux` `fde267f` (CI). CI puts the current `gcc12` head in its cache key and rebuilds V10.5 with it, so Linux toolchain drift fails the build. The macOS and Windows toolchains are only checked when someone rebuilds locally.
- Reference hashes:
  - V10.5 `standard(A1)/AUTOLOAD/FILAMENT_RGB_OFF/SOLO/solo_0.095f.bin`: `9237be17296e1d3c2a92162a859b1363dafd835df0dd953ffe0a52bdb91231d9`
  - This fork's A1 target at `73497e9` (V10.5 + #134): `052651efe729b611556b0c38badae09fca5e8351e1b7fcbf6fad883ef64596a1`

## Notes

- **`firmwares/` is an upstream mirror.** It holds the images published by upstream for V10.5. It is not built from this fork's sources. BMCU-Flasher's *Online* mode downloads from upstream, not from here. Images built from this fork come from CI artifacts.
- **Printer-facing version.** The BMCU reports the AMS version bytes `{0x00, 0x00, 0x32, 0x0A}` + `"AMS08"` (10.50) from `src/bambu_bus_ams.cpp`. The `version` file is not used by the build. We keep upstream's bytes on purpose: printers check the reported AMS version (see upstream [#73](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/issues/73) and [#119](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/issues/119), "the ams version is wrong, printer cannot continue"). Change them only after testing on the A1.
- The [upstream README](../../README.md), below the fork banner, describes upstream's releases, flashing guide and safety notes. The safety notes apply here too.
