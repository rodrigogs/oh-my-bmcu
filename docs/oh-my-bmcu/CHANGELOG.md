# oh-my-bmcu changelog

Changes of this fork on top of upstream [V10.5](https://github.com/jarczakpawel/BMCU-C-PJARCZAK)
(`fbff4e3`, docs up to `9573821`). The printer-facing version bytes are still upstream's 10.50.
Details, reasoning and review notes are in each commit message and in the
[audit backlog](audit-2026-09.md).

## Unreleased

Every change below had an adversarial review and a green CI, and has host unit tests for the logic
that can be isolated from the hardware (the trap handlers, the LED and motor wiring and the bus
timing can only be checked on the A1). None of it has run on the A1 yet, except upstream PR #134:
follow the [hardware test plan](hardware-test-plan.md) before relying on an image.

### Bus with the printer

- Throttle LED updates to 10 ms, upstream PR #134 by Murilo Bastos (`73497e9`). Verified on the A1
  on 2026-09-23: filament type and colour changes are saved again.
- Redraw a WS2812 strip only when its shown colours change, the root cause of #134 (`a08c7de`).
- Latch bus-offline detection and decide it by the protocol the printer speaks, so the motors stop
  within about 1 s of a lost link (`a6a84f6`).
- Resynchronise the RX parser on 200 us line gaps and UART overruns (`5e0f404`), and reset it around
  our own replies (`d8fef9e`).
- Run flash writes only in quiet bus windows, one per main-loop pass, with filament changes
  debounced by 500 ms (`e0dbe12`).
- Check set-filament frame lengths, always apply valid data, and answer AMS discovery again after a
  link loss (`ccfca7b`).
- Drop the redundant 100 us WS2812 reset busy-wait (`d8fef9e`).

### Motor and sensors

- Skip failed or impossible AS5600 reads instead of using them as angle 0; confirm the motor
  direction test before saving it (`ad5163c`).
- Bound the unload pull back, redetect and DM Stage-2 by time, distance and stall; blink the channel
  LED red after a stalled unload (`01391a1`).
- Time-qualify the tangle latch (500 ms below 40 % while pushing at full force) and release it on
  resume or buffer recovery (`7101ff4`); keep the 20 s full-force push limit at any buffer level
  (`90b44bc`).
- Take every motion distance from the integer AS5600 count instead of the float odometer, and make
  the pull back length deterministic (`a003d33`).
- Re-arm the DM Stage-2 push only after the filament really left both switches (`a604637`); clear
  its failure latch when the printer loads the channel (`b1fe02c`).
- Brake a jam-latched channel in before_on_use (`4a0a137`) and in the idle control once it is not
  the active channel (`89d9096`), so a resume or a channel change cannot push into the tangle.
- Keep the DM Stage-2 progress (remaining length, aborts, time and stall limits) across switch dips,
  and count Stage-1 pushes and buffer-lift unloads against it: with the link up, a blocked gear
  gets at most about 6 s of 900 PWM per insertion (`000d153`).
- Stop the auto-unload and the empty-channel pull while the printer link is down (`91387ff`).
- Move the 20 s full-force push limit into a host-tested helper, same behaviour (`7f9cbf9`).

### Calibration and flash

- Give timed-out or shallow calibration steps a safe range, flag them in NVM and flash them at every
  boot (`2ba70e9`).
- Average calibration readings in float instead of double, 3,828 bytes less flash (`1851c71`).
- Erase a flash journal page before writing over a slot that is not erased (`c3b5772`).

### Watchdog and faults

- Independent watchdog (1 s nominal) started after the first-boot calibration, fail-safe trap
  handlers that brake the motors and release the bus, and a magenta boot flash after a watchdog
  reset (`8c085a1`).
- Give the debug-build USART3 handler C linkage so it replaces the SDK's endless default
  (`4eb265f`).

### Upstream #148

- Undo a restored in-use state when the channel reads empty, and add the A/B image
  `env:a1_solo_autoload_rgboff_no_boot_restore` (`8934136`).

### Tooling

- `.gitignore`, pinned platform/SDK revisions, an explicit default env for the A1 target, validation
  of the `env:fw` variables, a safe `build_all_firmwares.sh` that builds every published mode.
- Host unit tests with Unity (`pio test -e native`) and a check that verbatim firmware copies in
  tests still match `src/`; every copy is marked verbatim (checked) or adapted (listed), and any
  other marker fails (`70f4310`).
- CI: the host tests with GCC and a rebuild of the V10.5 sources that must match the published
  binaries bit for bit. Only after both pass, both A1 images (the default and the #148 A/B image)
  are built with a size budget, their sha256 and an ELF check (strong trap and USART handlers, no
  soft-double routines), and uploaded as one artifact (`06c712e`).
- `build_all_firmwares.sh` normalises `OUT_DIR`, so no spelling of `firmwares/` slips past the
  `BUILD_ONLY_SOLO=1` guard (`65bbd90`).
- Host tests for the motion fixes above, including a simulation of the DM autoload with the
  firmware's own state machine and auto-unload code (`46b5895`, `8e6d597`, `ab5e978`, `31d5511`,
  `2ee7551`, `e12c340`).
