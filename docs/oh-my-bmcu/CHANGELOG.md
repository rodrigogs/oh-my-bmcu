# oh-my-bmcu changelog

Changes of this fork on top of upstream [V11](https://github.com/jarczakpawel/BMCU-C-PJARCZAK)
(`5b8e235`, merged on 2026-10-14); the entries below the merge were made on top of V10.5 (`fbff4e3`,
docs up to `9573821`). The printer-facing version bytes are V11's 11.00 since the merge.
Details, reasoning and review notes are in each commit message and in the
[audit backlog](audit-2026-09.md).

## Unreleased

Every change below had an adversarial review and a green CI, and has host unit tests for the logic
that can be isolated from the hardware (the trap handlers, the LED and motor wiring and the bus
timing can only be checked on the A1).

Hardware test Run 1 ([log](hardware-test-log.md), 2026-09-25) ran the image of `cec0e00`
(`a1_solo_autoload_rgboff`, sha256
`1c42d010ac10d6042bc5d3168a2b870e6af42c31f78d3440fced220e59d24d1d`), which holds every change under
[Up to Run 1](#up-to-run-1-image-cec0e00). It exercised normal use: the calibration and its
fallback, the recalibration hold, slot type and colour kept across a printer power cycle, loads,
unloads and a whole print, with no reset seen. Still pending on it: the fault cases (tangle, unload
limits, DM autoload corner cases), the direction check, the journal page erase and the boot LEDs.
Everything under [Since Run 1](#since-run-1-untested-on-hardware) is untested on hardware, and no
image with any of it (from `08975e1` on) is byte-identical to the tested one: follow the [hardware
test plan](hardware-test-plan.md) before relying on an image.

Hardware test Run 2 ([log](hardware-test-log.md), 2026-09-28) smoke-tested the image of `68c77c6`
(sha256 `642dfb208ab8c7b7b2e5486d5a54353e74a617c0963fd90a5cd1967b7dea2857`, the same as
`cb1d94d`'s), which holds every change under Since Run 1: calibration, a load, a print, its unload
and a printer power cycle passed; the fault cases are still pending. The [V11
merge](#merge-upstream-v11-5b8e235-untested-on-hardware) came after it: no merged image has run on
the A1, and the `cb1d94d` image is the rollback.

### Merge upstream V11 (`5b8e235`) (untested on hardware)

Upstream V11 (released 2026-10-04; `5b8e235` rebuilt its `firmwares/` on top of the `v11` tag
`7cba9a7`, same `src/`) merged into the fork at `cb1d94d` (merge base `9573821`) on 2026-10-14.
Where V11's rework covers a fork fix, the merge takes V11's code; the fork fixes V11 lacks are
re-applied on top of it, each deviation from V11 with code evidence. The merged tree then had a
seven-lens review (lost fixes, motion safety, bus protocol, NVM, V11 parity, auto-merge hazards,
build and tests), whose 17 confirmed findings were applied before the merge was committed. Local
gate, and CI at the merge commit `2e53ecb` (run 38677577919, all four jobs green): 21 host suites
with 467 cases pass under `env:native` and `env:native_asan`, `pio check` reports no high or medium
finding, and `check_test_copies.py` reports 0 errors (33 verbatim copies, 72 adapted, 67 of them
locked to 103 `src/` regions).

#### Visible behaviour changes

- The printer shows the AMS version 11.00.00.00 (V11's `{0x00,0x00,0x00,0x0B}` + "AMS08") instead
  of 10.50.00.00; the serial and discovery bytes are unchanged with AMS_A.
- The first-boot calibration is V11's and fail-closed: a step that times out or ends less than
  190 mV below rest, a range under V11's minimum, or a buffer that does not settle at the end aborts
  the run with two dim red blinks on every buffer LED, saves nothing and leaves every buffer without
  drive until a calibration succeeds (power-cycle to calibrate again). It needs every switch empty
  (its closing release phase; its idle baseline also refuses a switch at 1.40 V or more). The
  fork's fallback range and its fast red flash at boot are gone: Run 1's first calibration, which
  hit the fallback, would abort on V11.
- The 5 s recalibration hold clears only the calibration and motion records: filament slots and
  the loaded channel stay. It ignores a buffer without a valid calibration.
- A connected buffer without a valid calibration shows a steady dim red buffer LED; the mid-buffer
  orange shows only when every channel is empty; the SYS LED is white on every online pass.
- The direction test (V11's) skips a channel with filament at its switches, which then keeps no
  direction and does not drive until a boot with its switches empty.
- A failed DM autoload retracts first (up to 1.5 s, or 1.5 s plus 1.5 s after the third buffer
  abort, at most 120 mm), then stays red with the motor off; the pull back ends on empty switches
  after 5 fresh ADC publications, not one reading.
- A printer pull back on a channel without a valid calibration ends at once with the unload-fault
  blink, instead of after its 9.9 s budget.
- At boot a restored loaded channel with empty switches or no valid calibration is unloaded (V11),
  in both A1 images.
- Flash writes start without the 500 ms debounce or a quiet-gap wait, also while the printer link
  is down; a failed step stops every NVM write until the next boot, with no retry.
- LED strips are not drawn while the bus is busy or the flash clock is slowed.
- The A1 image grows from 52,716 to 59,156 bytes (the #148 A/B image to 59,176 bytes).

#### Bus with the printer

- From V11: USART1 RX through a 2 KB DMA ring that `bus_rx_poll()` parses in the main loop (no
  per-byte RX interrupt), a 3-buffer receive queue (two completed frames wait while a reply is
  pending and are answered on later passes), TX by DMA with the receiver off and a 100 ms abort,
  the heartbeat fast path, the frame minima (0x03 12, 0x04 13, 0x05 8 or 26, 0x08 45 bytes, 0x218
  34 data bytes), set_motion's before_on_use rule and `ams_state_claim_loaded`, the motion
  replies' meters from the odometer, the version bytes, and the AMS_AUTO addressing
  (`ams_addressing.h`, not used by the A1 env).
- Re-applied on V11: the 200 us stall threshold (`BUS_RX_RESYNC_GAP_TICKS`) in `bus_rx_poll()`
  instead of V11's 11.3 ms, measured between polls; the parser reset at the TX boundary as a ring
  mark (`rx_mark_tx`, `rx_reached`), also on the heartbeat fast path (`rx_mark_inside`, a review
  finding: the fast path skipped past the mark and ate the head of the printer's next frame); the
  latched link state (`bus_link.h`) in `bambubus_run` and `ahubus_run`, where V11 still had the 119 s
  flip; the online-detect re-arm after a link loss, in fixed addressing only; the set-filament save
  moved into the two handlers, so only data applied to this unit is saved; the AHUB short-header
  check; the reply-layout static_asserts; the host hook for the UID base; `= {}` initialisers for
  `-Werror`.
- Dropped: the interrupt-driven RX parser with its gap and overrun resync (DMA RX removes the
  cause); `src/bambubus_frame_len.h` and `src/bambubus_set_filament.h` (V11's minima, the stricter
  45 / 34 for set-filament, since they never read outside the frame and the A1 frame was never
  captured); the AHUB `ahub_set_data_fits` / `ahub_now_filament_num` helpers (V11's literals give the
  same bounds, and V11 drops a request with a bad channel); `host_link_offline()` (`src/host_link.h`
  keeps only the host-type defines; V11's inline decision gives the same result on the A1); a
  heartbeat cut by our TX no longer counts as seen.

#### Flash

- From V11: writes as background jobs that `Flash_background_run()` carries out one page erase or
  one half-word program per main-loop pass, started only while `bus_background_ready()`, with a
  sticky `g_flash_fault` and no retry; the flash clock halved around every flash operation; the
  header magic programmed last; filament page rollover through a scratch page (page 14) that the boot
  replays; the loaded-channel log on pages 6-13 with tag `0xA6`, a channel filter, and a slot search
  that never erases the page holding the newest record; the CAL v2 record (64-byte payload with the
  DM key thresholds, valid mask in bits 8-11, sequence in bits 16-31) on page 0 and page 15, and a
  calibration clear that writes an empty record. `Flash_NVM_full_clear()` has no caller left.
- Re-applied on V11: the strict free-slot test (`0xE339E339` only) at the two places that pick a
  slot, since V11's lenient one would program over a `0xFFFFFFFF` word and set the sticky fault;
  `src/nvm_records.h` and `src/nvm_journal.h` adapted to V11's rules (`nvm_sta_find_slot`, the cursor
  after the last used slot, the CAL rsv helpers); the page-layout static_asserts.
- Dropped: the quiet-window scheduling with its 500 ms debounce, retry back-off and give-up counters
  (`src/nvm_save_sched.h`, `g_state_job`, `g_fil_job`); the erase in the write path; the torn
  loaded-channel slot skip and the failed-retry fix, which V11's slot search and job model supersede;
  the strict erase verification (V11's lenient one only accepts more).
- Layout: a CAL v1 record or a tag `0xA5` loaded-channel record of a pre-merge image is ignored, so a
  non-erasing programmer leads to a calibration and no restored channel (BMCU-Flasher erases the NVM
  anyway).

#### Calibration

- From V11: the fail-closed run with a per-channel valid mask, readings outside 0.05-3.25 V refused,
  an idle baseline of 60 samples (40 mV spread at most, every switch below 1.40 V), a release phase
  (every buffer within 80 mV of rest and every switch empty for 1.2 s), the 25 mV rest window and
  700 ms settle, the DM key thresholds in the record, no NVM wipe and no diagnostic colour steps,
  float averaging.
- Re-applied on V11: the 190 mV first-step floor, at capture and when a stored record is loaded
  (`src/mc_pull_cal_range.h`, trimmed to it).
- Dropped: the fallback range and its boot flash (`2ba70e9`), `src/mc_pull_cal_mean.h` (V11 averages
  in float), the fork's NVM wipe at calibration.

#### Motor and sensors

- From V11: the int64 AS5600 odometer (`encoder_count()` is its low 32 bits) and
  `Motion_control_filament_meters()`; the AS5600 valid mask; `MOTOR_get_dir`, with a watchdog feed
  per 10 ms step added; the autoload fail retracts; the Stage-1 gate (closed while the printer or an
  auto-unload moves the channel); the pull back's 5-publication key debounce; the calibration-valid
  gates (PWM 0, the dim red LED, the direction test); the all-empty condition of the mid-buffer
  orange; `TIM_Prescaler = 0` for V11's slower APB1 (the PWM stays 72 kHz).
- Re-applied on V11: the fork's metre API over V11's counts; every motion limit, the jam latch, the
  auto-unload guards (now also off on a channel without a valid calibration or with the DM fail
  latch), the idle push limit, the stale-ADC stop, the DM re-arm and Stage-2 carry-over, the
  link-down rule and the watchdog feeds; the fixed pull back length (V11 still had the
  before_pull_back accumulator); a finished printer load clears the DM fail latch (V11 never does).
- New fork deviations from the review: the recalibration hold skips a buffer without a valid
  calibration (its raw voltage fired the hold after every boot, a clear-and-reboot loop); a printer
  pull back on such a channel ends at once.
- Dropped: the fork's direction probe (`as5600_dir_probe_*`), and V11's `dm_idle_empty`, its 100 ms
  key drop and its printer-load block (the fork's DM re-arm covers them).

#### ADC

- From V11: `ADC_DMA_generation()` and the ADC divider /4 on V11's 72 MHz APB2, the same 18 MHz ADC
  clock as before (audit #29 unchanged).
- Kept: the restart after a DMA transfer error and every 500 ms while stale, the 10 ms calibration
  waits and the watchdog feeds.

#### Main loop, watchdog and faults

- The main loop is V11's (`bus_rx_poll`, the two protocols, `send_package`, the host decision and
  SYS LED, the saves on dirty flags while the bus is idle, motion, `Flash_background_run`, LEDs) with
  the fork's watchdog feed and fault-record phases; a new phase `BLACKBOX_PHASE_FLASH` (12) covers
  the flash step, and the RX parser also runs inside the NVM, FLASH and RGB phases.
- The boot keeps the fork's restore of the loaded channel and its A/B image, plus V11's check after
  `Motion_control_init`.
- `watchdog_cfg.h` re-checks the 100 ms feed gap against V11's flash paths; two feeds were added to
  the recalibration and one per step to `MOTOR_get_dir`.
- `RGB_update` and `WS2812_class::updata` skip while the bus is busy or the flash clock is slowed,
  before the 10 ms throttle, so a pass that cannot draw does not use up the gap.

#### Tooling and CI

- CI's `reproduce-upstream` rebuilds 24 V11 images from `5b8e235` (`BUILD_ONLY_MIN=1`) and compares
  them with its `firmwares/manifest.txt` (`--expect-count 24`); the `firmwares/` mirror is now V11's
  (1,020 images).
- `build_all_firmwares.sh` follows V11's matrix: `BUILD_ONLY_MIN=1` (24 images: AMS_AUTO and AMS_A at
  0.095 m) replaces `BUILD_ONLY_SOLO=1`, which now stops with an error.
- `check_fw_env.py` accepts `BAMBU_BUS_AMS_NUM` 0-4.
- CI gates: `IMAGE_BUDGET_BYTES` 57,344 to 59,392 B and `RAM_HEADROOM_FLOOR_BYTES` 4,096 to 2,048 B
  (measured 2,524 B: V11's 2 KB RX DMA ring); the ELF check also requires `DMA1_Channel4_IRQHandler`
  and `DMA1_Channel5_IRQHandler`; the RGB compile-smoke line builds AMS_AUTO with DM=1, the largest
  published image (60,752 B, under the 61,440 B cap).
- `env:native` builds with `-ffp-contract=off`, so the Mac computes what the x86_64 runner does (a
  review finding: a new calibration test passed only with fused multiply-add).
- `.gitignore` adds V11's `.idea/`.

#### Host tests

- Deleted: `test_bambubus_frame_len` and `test_bambubus_set_filament` (their cases moved into
  `test_bambubus_frames`), `test_mc_pull_cal_mean`, `test_nvm_save_sched`.
- Rewritten: `test_bus_rx_parser` and `test_bus_rx_fuzz` (V11's parser, the ring model, the TX mark,
  the stall reset), `test_host_link` (V11's inline decision), `test_mc_pull_cal_range` (the floor,
  the record check, the recalibration skip), `test_nvm_journal` and `test_nvm_records` (V11's jobs
  with a power cut at every step, the strict slot test, the CAL v2 codec), `test_boot_restore` (V11's
  state API and boot check).
- Extended: `test_bambubus_frames` (version 11.00, V11's minima, the queued packet),
  `test_ahub_frame` (V11's literals), `test_dm_stage2` (fail retracts, the Stage-1 gate),
  `test_motion_limits` (the odometer, the 5-publication pull back end, the uncalibrated pull back),
  `test_auto_unload` (the V11 gates), `test_as5600_sample` (V11's driver; the probe cases deleted),
  `test_ws2812_frame` (the bus and flash-clock gate, the dim red), `test_blackbox` (phase 12).
- 21 suites (25 before), 467 cases.

#### Images

- `a1_solo_autoload_rgboff` 59,156 bytes, sha256
  `d76731bd8fcebbe7756a9c1c66ee539e7ae29bfd955d0a3b035ca835a3a8d61f`;
  `a1_solo_autoload_rgboff_no_boot_restore` 59,176 bytes, sha256
  `b571f812fb0c545b88f2e34de4607e832c9fb11909352b5b67a54199c5502a40`. Built locally on macOS
  (`toolchain-riscv-mac` `e6360e0f`); CI's Linux build of `2e53ecb` (run 38677577919) gave the same
  hashes.
- Not in the merge (follow-ups): V11's heartbeat fast path can still fire inside a frame's payload
  and lose more than one frame (a fix with a host test and an upstream issue); a `Flash_fault()`
  accessor for the sticky flash fault; audit #29.

### Since Run 1 (untested on hardware)

#### Visible behaviour changes

- A load (send_out) stops on a gear that does not turn: 800 PWM or more with less than 1 mm of
  travel in 1 s. The channel stays braked until the printer's next command for it, no LED changes,
  and the printer's own load timeout reports it.
- The buffer-lift auto-unload stops after 1 s on a gear that does not turn, and the channel LED
  blinks red (1 s on, 1 s off) as after a stalled pull back.
- The idle control's push stops after 1 s on a stalled gear (420 PWM or more) or after 10 s of
  pushing: silently at a rounded 28-29 % (held until a rounded 26 % or less), with a steady red
  status LED below 28 %.
- Stale ADC readings (no new half-buffer for more than 100 ms) brake every channel without changing
  its motion, so a channel stopped by a limit stays stopped once the readings are back, and every
  status LED blinks pure blue with the buffer LEDs off. The stream is restarted after a DMA transfer
  error (which used to leave the readings at 0 V until a reboot) and every 500 ms while stale (a
  stream that stopped otherwise used to stay frozen until a reboot).
- The DM key must read 'none' for 100 ms before an insertion ends, counting only readings at most
  20 ms old; a failed DM channel's red now lasts up to 100 ms after the filament is pulled out.
- At boot, a blue flash after the three magenta ones means the reset followed a CPU trap.
- A flash write that keeps failing gets at most three attempts per change instead of one every
  500 ms for good.
- A 0x03, 0x04 or 0x05 frame too short for its handler is dropped with no reply.
- The image changes: no build from `08975e1` on is byte-identical to the Run 1 image.

#### Bus with the printer

- Drop 0x03, 0x04 and 0x05 frames too short for the fields their handlers read (11, 12, and 8 or 26
  bytes), in the host-tested `src/bambubus_frame_len.h` (`a8c7b6b`); the 0x08 minimum uses the same
  CRC16 length (`01d7c5a`).
- Pin the sizes and field offsets of the 0x03 and 0x04 motion replies with static_asserts and name
  the bytes the before_on_use replay copies, same bytes sent (`745ebc0`).
- Zero the online-detect reply template's CRC16, which never matched its bytes; every reply rebuilds
  it, so the bytes sent do not change (`98b1078`).
- AHUB, unreachable on the A1: store only a channel or none as the active channel (`08975e1`),
  apply a set request only if its data lies inside the frame (`7a73a4c`), and drop frames without
  the short header (`2a1a337`).
- Move main()'s host type and offline decision into the host-tested `src/host_link.h`, same
  behaviour; the image differs because main() is laid out differently (`6c273c8`).

#### Motor and sensors

- Stop the send on a stalled gear, with the 10 m cap in the same limit and no time budget
  (`2dd02d2`).
- Stop the buffer-lift auto-unload on a stalled gear and blink the unload-fault LED; no time or
  distance limit beyond its own 15 s end (`2858362`).
- Stop the idle control's push on a stalled gear (at the 420 PWM hold floor) or after 10 s, braking
  without a fault at a rounded 28-29 % and red below (`e0445a4`); the silent brake holds down to a
  rounded 27 % (`8f0df7e`).
- Give the four ADC calibration waits (reset and calibration, for each ADC) a 10 ms deadline each,
  so a stuck bit no longer hangs the boot before the watchdog runs (`b109e72`), and keep the
  timeout flag `g_adc_cal_timed_out` in the image for a debugger (`9766ac3`); restart the ADC stream
  after a DMA transfer error, keeping the last readings until the first new half (`5aca6b2`), and
  every 500 ms while it stays stale (`876fef3`); stop every channel while the readings are more than
  100 ms old, with the blue blink; the tangle latch's timed trip and release, and the recalibration
  hold, restart from the first fresh reading (`ec4e022`). The stale stop brakes through one helper
  (`6110382`) and leaves each channel's motion as it is, so the silent 20 s latch, a send stopped by
  its limit and an idle push braked by its limit stay set, the send's and pull's soft starts are not
  replayed, and the jam trip timer starts on the first fresh pass (`1187db5`); send_out no longer
  releases a jam latch on a stale reading (`c2a2b2d`).
- Debounce the DM key 'none' by 100 ms before it ends an insertion; a shorter one pauses the drive
  and Stage-1 or Stage-2 goes on (`f0d247e`). Key readings more than 20 ms old end the run of 'none'
  and the retract check instead of counting, so a stream that stops on a one-pass 'none' no longer
  unloads a loaded channel and pushes it 120 mm once it is back (`6d678c6`).

#### Flash

- Back off failing NVM writes (retry after 500 ms, then 5 s) and give up after three failed
  attempts until the next change; count the give-ups in RAM (`a798e00`).
- Skip a torn loaded-channel slot instead of erasing the page that holds the newest record
  (`929325a`).
- Move the journal record pack, validate and boot-scan logic into the host-tested
  `src/nvm_records.h`, same behaviour (`3f0dbbe`), and take the loaded-channel log geometry from
  `nvm_journal.h` everywhere (`c19c8ff`).
- Do not take the loaded channel as saved after the fallback erase when the retry fails too, so a
  later load of the same channel is written (`9e0897e`).

#### Watchdog and faults

- Keep a fault record in the backup registers across resets (reset flags, resets in a row, main-loop
  phase, longest pass, mepc and mcause of a trap) and flash the SYS LED blue once after the
  watchdog flash when the last run trapped (`735a69f`); write the trap words only once the backup
  domain is clocked and writable (`ffcd91d`, comments in `4e5dbb6`); clear the record's magic first
  and write it last at boot, so a reset during that write leaves no record instead of an earlier
  trap (`5eba3c0`).
- Feed the watchdog before each of the boot's 16 channel-detect samples and after the last, so ADC
  restarts there (up to about 42 ms each) cannot add up to a reset at every boot (`5a01970`).

#### Tooling

- Firmware build with `-Wall -Wextra -Wundef` (`cdf46bb`), its 62 warnings cleared (`5c7896b`),
  `BMCU_SOFT_LOAD` and `BMCU_P1S` defaulted to 0 (`f9ac585`), then `-Werror` on `src/` (`06d0de0`,
  comment in `e0a3435`).
- `pio check` (cppcheck) configured for the firmware envs (`b294768`), its high and medium false
  positives suppressed inline with the reason (`eaf927d`), and a gating CI job that the A1 images
  now wait for (`030d52d`).
- `env:native_asan` runs the host suites with ASan and UBSan (`f533fb8`), also in CI (`1dee6a4`).
- CI compiles `env:moj` and three `env:fw` variants (`8539749`), reports the RAM headroom and fails
  below 4 KiB (`69281c7`; `00a9c32` reads the symbols in one `nm` pass per image), pins every action
  to a commit (`621ef8e`), shares one composite setup action, `.github/actions/setup-pio`, across its
  four jobs (`9ec9adb`) and stops any job after 20 minutes (`0c2e199`).
- `check_test_copies.py` locks the `adapted from` test models to a hash of the `src/` region they
  model, with `--relock` (`82d659e`; anchors and `test/adapted.lock` in `52dc274`, `05f35d4`, and
  the call sites of the idle push, pull back and redetect guards in `f0f9302`), and
  flags a `test_*` case never passed to `RUN_TEST()` (`d08332c`); the guard scripts get their own
  fixtures in `scripts/tests/`, run in CI (`1155400`).
- Host tests: a fuzz of the RX parser with state invariants after every byte (`6d6e4a2`, `e15a78e`,
  `b54da5f`); golden frames of the BambuBus replies with the real `bambu_bus_ams.cpp` compiled on
  the host (`347161e`, `296435e`); the frame minima (`49cdf40`); the AHUB helpers (`a8aadce`,
  `f22b202`, `b7b81b4`); the host type and offline decision (`2899a1c`); the fault record and boot
  code (`8c3d8dc`); the NVM back-off, records and torn-slot skip (`d9448c2`, `4a3a2ee`, `b7f10fc`,
  `aee435d`); the ADC stream age and stale stop (`0609525`); the DM 'none' debounce (`9e32bb8`); the
  send, auto-unload and idle push limits (`4115e52`, `b9942f9`, `02c58c1`), the send guard on an A1
  hold at 800 PWM or more with jitter (`06768f3`) and the auto-unload guard's start right after a
  stall (`9446070`); pass limits on the open-ended loops of `test_dm_stage2` (`3ba12db`).

### Up to Run 1 (image `cec0e00`)

#### Bus with the printer

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

#### Motor and sensors

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
- Do not start the auto-unload from a buffer lift while a channel is tangle-latched, nor from the
  lift that releases the latch (`c513a32`), also when the hand sags and lifts the buffer again
  before letting go: the hold-off ends only with the buffer back below 55 % (`36efea7`); drop an
  unreachable 0xF06F report from the silent 20 s latch, no change in behaviour (`26d8841`).
- Keep the DM Stage-2 progress (remaining length, aborts, time and stall limits) across switch dips,
  and count Stage-1 pushes and buffer-lift unloads against it: with the link up, a blocked gear
  gets at most about 6 s of 900 PWM per insertion (`000d153`).
- Stop the auto-unload and the empty-channel pull while the printer link is down (`91387ff`).
- Move the 20 s full-force push limit into a host-tested helper, same behaviour (`7f9cbf9`).

#### Calibration and flash

- Give timed-out or shallow calibration steps a safe range, flag them in NVM and flash them at every
  boot (`2ba70e9`).
- Average calibration readings in float instead of double, 3,828 bytes less flash (`1851c71`).
- Erase a flash journal page before writing over a slot that is not erased (`c3b5772`).

#### Watchdog and faults

- Independent watchdog (1 s nominal) started after the first-boot calibration, fail-safe trap
  handlers that brake the motors and release the bus, and a magenta boot flash after a watchdog
  reset (`8c085a1`).
- Give the debug-build USART3 handler C linkage so it replaces the SDK's endless default
  (`4eb265f`).

#### Upstream #148

- Undo a restored in-use state when the channel reads empty, and add the A/B image
  `env:a1_solo_autoload_rgboff_no_boot_restore` (`8934136`).

#### Tooling

- `.gitignore`, pinned platform/SDK revisions, an explicit default env for the A1 target, validation
  of the `env:fw` variables, a safe `build_all_firmwares.sh` that builds every published mode.
- Host unit tests with Unity (`pio test -e native`) and a check that verbatim firmware copies in
  tests still match `src/`; every copy is marked verbatim (checked) or adapted (listed), and any
  other marker fails (`70f4310`).
- The calibration's safety tests use the firmware's own jam trip level, percent mapping and
  recalibration rule, and checked copies of its rounding and load stops (`8b2c1f1`, `a467620`);
  every other firmware copy in the host tests is now a real include, a checked copy or marked
  adapted (`95de679`), and the copy check also fails on a test that redefines a `src/` constant or
  macro, and on a verbatim copy that src/ has only inside a longer line (`8969b66`).
- CI: the host tests with GCC and a rebuild of the V10.5 sources that must match the published
  binaries bit for bit. Only after both pass, both A1 images (the default and the #148 A/B image)
  are built with a size budget, their sha256 and an ELF check (strong trap and USART handlers, no
  soft-double routines), and uploaded as one artifact (`06c712e`).
- `build_all_firmwares.sh` normalises `OUT_DIR` (`65bbd90`) and compares it by file identity, so no
  spelling, letter case or symlinked path of `firmwares/` slips past the `BUILD_ONLY_SOLO=1` guard.
  It refuses an `OUT_DIR` that is the checkout, `$HOME` or a directory above either, and one that
  exists and is neither empty nor an earlier output of the script (a `manifest.txt` with its
  header) unless `FORCE=1`, and ignores an exported `CDPATH` (`88b1467`). CI tests these guards
  with a stub `pio` (`c7cfcb3`).
- Host tests for the motion fixes above, including a simulation of the DM autoload with the
  firmware's own state machine and auto-unload code (`46b5895`, `8e6d597`, `ab5e978`, `31d5511`,
  `2ee7551`, `e12c340`).
