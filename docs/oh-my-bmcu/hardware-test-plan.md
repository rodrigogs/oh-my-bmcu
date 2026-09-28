# Hardware test plan

What to check on the real A1 after flashing a build of this fork. Every change on `main` was built,
unit-tested on the host, reviewed and passed CI, but has not run on the hardware until an image
passes this plan. Run the **basic** checks (no extra tools) for every new image. The **optional**
ones need a logic analyser or a WCH-Link on the BMCU and are for chasing a regression.

Steps marked **(USB)** need only the BMCU on USB-C power, with no printer; every other step needs
the printer. On USB power alone no motor turns (there is no 24 V), so a USB step can only check
LEDs.

Which image: [Run 1](hardware-test-log.md) exercised the image of `cec0e00`
(`a1_solo_autoload_rgboff`, sha256 `1c42d010ac10d6042bc5d3168a2b870e6af42c31f78d3440fced220e59d24d1d`).
The images built with the fixes that followed it (from `08975e1` on) are byte-identical to no
hardware-tested image: main() is laid out differently since `host_link.h`, the linker inlines less
of the boot, and the fixes since then changed the code. Before such an image replaces the tested one,
smoke-test it: the boot, the first heartbeat and AMS A (section 1, the reconnect step), a load of
slot 1 (section 2, first step), a print and its unload (section 3, first step), and slot 1's type
and colour across a printer power cycle (section 2, second step). If one of them fails, flash the
tested image back and report it. Then run the checks of its new features: the steps marked "New" in
sections 2, 5, 7 and 8, and sections 9 and 10.

Setup: Bambu Lab A1 on printer firmware 01.08.01.00, AMS type "AMS", one BMCU 370C, image
`env:a1_solo_autoload_rgboff` (CI artifact `oh-my-bmcu-a1-<commit>-push` or a local `pio run`),
flashed as described in [README.md](README.md#flash)
(`bmcu_flasher.py ... --mode usb --verify-last`).

## 0. Before flashing

- [ ] **(USB)** Note the sha256 of the image and the commit it came from, and check in the
      [log](hardware-test-log.md) whether an image with that sha256 already passed.
- [ ] **(USB)** Remove all filament from the BMCU. Unplug the printer, disconnect the BMCU, connect
      it by USB-C.

## 1. First boot and calibration

- [ ] **(USB)** After flashing, the SYS LED shows no magenta and no blue flash (the flasher's reset
      is not a watchdog reset), then the calibration starts.
- [ ] **(USB)** After flashing, calibrate properly, one buffer at a time (upstream video:
      https://www.youtube.com/watch?v=Hn_DNzSmhuc). All LEDs blink yellow slowly for about 1.5 s
      while the idle buffers are sampled (leave them at rest), then three quick yellow blinks, then:
      - while the buffer's LED blinks blue, move it to its low end (the end the extruder pulls it to
        when the filament is tight, the same end you hold to recalibrate) and let it return to rest;
      - while it blinks red, lift it to the other end and let it return.
      Two yellow blinks confirm each step. The order matters: the first end becomes the low end. The
      end blink is green, and later boots show no red flash.
- [ ] **(USB)** Optional, to check the fallback: let one buffer time out. It flashes red quickly at
      the end, then for about 0.7 s at every boot; the printer still finds the AMS (with the
      printer). Recalibrate it properly and the boot flash stops.
- [ ] Reconnect to the printer only while it is unplugged, then power on. SYS LED: red until the
      printer's first heartbeat, then white. The printer shows AMS A.
- [ ] Direction check, printer on and the slot still empty: push the buffer to its low end for a
      moment (well under 5 s, or it starts a recalibration) and let it go: the buffer's LED is blue
      while it is down. On this AUTOLOAD image that push also starts upstream's buffer-tap autoload:
      the channel's status LED turns yellow and the gear feeds forward for about 5 s, then briefly
      red with a short retract. That is expected and says nothing about the direction. Wait about
      6 s until the motor has stopped, then lift the buffer: its LED turns red, and the motor may
      turn backwards while it is up (upstream's manual unload). If the buffer's colours are the
      other way round, the calibration was done in the wrong order: recalibrate.

## 2. Bus, filament info and flash

- [ ] Load PLA into slot 1 (AUTOLOAD). The LEDs go through the load colours and end steady; the
      loaded slot's LED does not flicker while nothing changes.
- [ ] Change slot 1 type/colour three times within a few seconds (printer screen or OrcaStudio).
      Wait about 2 s per edited slot, power-cycle the printer: slot 1 keeps the last values (it used
      to fall back to "PETG, white").
- [ ] Change slot 1's filament at least 7 times, about 1 s apart, then power-cycle: the last values
      stay. This goes through the page erase every 6 records and confirms that erased flash reads
      `0xE339E339` on this chip, which every save relies on.
- [ ] Leave the printer idle for more than 5 minutes (covers a SysTick wrap): the SYS LED stays
      white and the slot still answers.
- [ ] **(USB)** With the BMCU on USB-C only (not connected to the printer), the SYS LED stays red,
      with no printer heartbeat, for more than 4 minutes.
- [ ] New: power-cycle the printer three times: AMS A is found every time. The online-detect probe
      (0x05, subtype 00) and confirm (subtype 01) now need at least 8 and 26 bytes; no capture of
      the A1's exists, so a confirm shorter than 26 bytes would be dropped and the AMS not found.
      If a sniffer is at hand, log the confirm's length (see the logic analyser section).
- [ ] New: a load, a print and an unload of slot 1 show no HMS error and no step stalls: the
      printer's 0x03 and 0x04 frames (12 and 13 bytes as captured) pass the new minima of 11 and 12.
- [ ] New: load and unload slot 1 a few times, then power-cycle the printer: the printer shows the
      right slot as loaded (or none) and slot 1's type and colour come back. The torn-slot skip and
      the write back-off have not run on the board; normal writes must behave as before.
- [ ] New: switch the printer off during an unload of slot 1 (while the BMCU retracts), then back
      on: the printer shows slot 1 either still loaded or unloaded, never another slot, and the
      next load and unload of slot 1 work. The host model does not cover a half-erased page whose
      words read as erased but are weak.
- [ ] Optional, only with a switch or breakout that opens just the A/B data pair, wired while the
      printer is unplugged: opening it with the printer on turns the SYS LED red within about 1 s
      and stops the motors; while it is red, lifting the buffer moves no motor, and an auto-unload
      that was running stops and does not restart. Closing it turns the LED white and the slot
      works. Do not do this by pulling the bus cable from a powered printer (the upstream safety
      notes forbid it), and a BMCU on USB-C power alone proves nothing here: without the printer's
      24 V no motor turns anyway. Without such a breakout, the host tests (test_auto_unload) are
      all there is for this.

## 3. Printing and unloading

- [ ] A test print from slot 1 (for example the 15 mm cube) starts, prints and unloads at the end.
      The unload pulls about 95 mm and the channel LED does not blink red afterwards.
- [ ] A long print (1 h or more) finishes with no "AMS offline/communication" errors and no motion
      timeouts.
- [ ] Cut power mid-print and restore it: the BMCU still reports slot 1 loaded (resume), and no
      tangle error appears unless the spool is really stuck.

## 4. Tangle detection (motor safety)

- [ ] Hold the spool for less than 0.5 s while printing: the print does not pause.
- [ ] Hold the spool until the print pauses: red LED and the printer's tangle error, about 0.5 s
      later than with upstream.
- [ ] Resume with the spool still held and nothing else done: it pauses again at once, motor
      braked, no push (also when the printer resumes through its load step, before_on_use).
- [ ] Free the spool, feed filament until the buffer is above 40 %, then resume: printing should
      continue. While paused, holding the buffer at or above about 52 % for 1 s should also turn the
      red LED off. Both depend on what the A1 sends on a resume, which has not been observed yet:
      note which of the two worked, and what the printer showed if neither did.
- [ ] While paused after a tangle, touch nothing for 10 s: note whether the red LED goes out by
      itself (a buffer resting a few mV above neutral releases the latch). If it does, the resume
      must pause again about 0.5 s after the buffer drops below 40 %.
- [ ] If the A1 unloads after the tangle: the latch holds through the pull back, idle and the
      reload, unless the buffer is lifted above 85 %. The motor stays silent meanwhile, also once
      the printer has deselected the slot and the buffer sits below 30 % (the red LED stays on).
- [ ] After a tangle, printer idle, slot still selected: lift the buffer to the top and hold it
      firmly until the red LED goes out (about 1 s), keep holding 2 s, let go. The filament must
      not be unloaded. The motor may pull back while you hold it (the idle control, no purple LED);
      only a purple-LED retract after letting go would be the auto-unload. Repeat, but after the LED
      goes out let the buffer drop about a quarter of its travel (not to the middle), lift it to the
      top again at once and let go: again no purple LED. Note where the buffer comes to rest. A
      quick lift and release from rest afterwards unloads it as usual; if the buffer rested above
      the middle, the first lift (pressed back down to the middle) only ends the hold-off and the
      second one unloads.
- [ ] Same with another slot (or none) selected: a quick lift of the latched slot's buffer does
      nothing and the LED stays red; held at the top for 1 s the LED goes out, and letting go does
      not unload.
- [ ] Optional, hard to set up by hand: brake the spool just enough that the buffer stays above
      40 % while the motor strains at full force for more than 20 s of push (the anti-stall's 0.5 s
      rests pause the count, so with a gear that stalls or crawls, likely on the 0.2 mm nozzle, this
      takes about 32 s). The channel then brakes and its
      LED turns red; the print only pauses if the buffer then stays below 40 % for 0.5 s.

## 5. Unload limits

- [ ] Pull the filament out of the BMCU inlet during an unload: the gear stops after about 95 mm,
      the slot shows empty, and another slot loads without a power cycle.
- [ ] Hold the filament during a pull back: the motor stops about 1.6 s in, the channel LED blinks
      red (1 s on, 1 s off) and the other slots are still served. Note what the A1 reports. The
      blinking stops when the filament is pulled out or the slot is used again.
- [ ] DM autoload: a normal insertion still feeds 120 mm.
- [ ] New: a normal buffer-lift unload (key at 'both': lift the buffer to the top and let it go):
      the purple retract runs until the filament leaves the switches (plus 1.5 s) or the buffer
      drops, as before; it does not stop early, and the channel LED does not blink red afterwards.
      Note how long the unload of a whole tube takes: the auto-unload ends by itself at 15 s.
- [ ] New: hold the filament firmly on the printer side of the BMCU (or leave it stuck in the tool
      head) so the retract pulls against it and the gear cannot turn; a clamped spool does not stop
      a retract, the filament only goes slack. Do the lift gesture, and as soon as the buffer is
      back in the middle, hold it there by hand (45-55 %, in any case above 35 %) while the
      retract pulls: the 850 PWM retract stops after about 1 s (not 15 s), and the channel LED
      blinks red (1 s on, 1 s off) until the filament is pulled out of the switches or the printer
      loads, prints or unloads that slot. Free the filament and lift again: it unloads normally.
      Without the hand the retract drags the buffer below 35 % and ends at once with no blink:
      that is its normal end, not a failure (the stall case is also host-tested, in
      `test_auto_unload`). A gear that slips on the filament is not caught: the retract then runs
      to its 15 s end.
- [ ] New: with no filament at the switches, hold the buffer up: the motor turns backwards while it
      is held (the manual empty pull, 700 PWM, which has no stall limit) and stops when it is let
      go.

## 6. Upstream #148 A/B test

Only if the A1 ever stops finding the AMS after a power cycle with a slot loaded.

- [ ] With the default image (`env:a1_solo_autoload_rgboff`): load a slot into the extruder, then
      power-cycle the printer 5 times. Note each time whether the printer finds AMS A.
- [ ] Flash `env:a1_solo_autoload_rgboff_no_boot_restore` (from the same CI artifact, or
      `pio run -e a1_solo_autoload_rgboff_no_boot_restore`, then
      `.pio/build/a1_solo_autoload_rgboff_no_boot_restore/firmware.bin`) and repeat the same 5 power
      cycles. Until
      the printer uses the loaded slot, its LED shows the idle colour. A print cut by a power loss
      must still resume, and an unload right after boot must still retract.
- [ ] Repeat both with several slots holding filament and none loaded in the extruder.
- [ ] If only the default image loses the AMS, the boot restore is the cause: report it with both
      counts.

## 7. DM autoload re-arm

- [ ] Insert filament: Stage-2 feeds about 120 mm past the inner switch, once.
- [ ] Nudge the filament so a switch flickers without removing it: no second 120 mm push.
- [ ] Pull the filament out past both switches and insert it again: Stage-2 runs again.
- [ ] If Stage-2 ever gives up (channel LED red after an insertion): loading that slot from the
      printer clears the red LED once the load finishes, and so does pulling the filament out.
- [ ] Flick the lever now and then during the Stage-2 push: the push still ends after about
      120 mm in all, not 120 mm from the last flick.
- [ ] Optional, with the outlet blocked: three aborts, then red, also while flicking the lever.
      With the filament jammed in the gear and the tip at the inner lever, flicking the lever and
      moving the buffer (past 75 %, or lift gestures) drive the motor about 6 s in all, then red.
- [ ] Lift gesture during the Stage-2 push (buffer to 80 % or more and back to the middle within
      1 s): the filament comes out. The LED is red only while the buffer is held up, then purple,
      and does not stay red; inserted again, the filament gets a full autoload.
- [ ] New: push filament into the outer switch by hand several times, some of them with a slightly
      wobbly push at the lever: Stage-1 (yellow), then Stage-2 (120 mm) complete every time, and the
      filament is never left waiting at the outer switch. A key reading 'none' (both switches open)
      for less than 100 ms now only pauses the drive.
- [ ] New: with a channel loaded (white) or failed (red), pull the filament fully out and keep it
      out: within about 0.1 s the channel counts as empty (the red clears, the LED goes off), and
      the printer shows the slot empty at once. Insert it again: the autoload runs again.
- [ ] New, optional: on a failed (red) channel, flick the outer lever quickly, if you can, so both
      switches open only for a moment: it stays red and does not push. Only a removal of 0.1 s or
      more clears it. The case is host-tested
      (`test_a_key_glitching_to_none_keeps_a_failed_run_failed`).
- [ ] New: hold the filament so it cannot reach the inner switch: Stage-1 stops pushing about 5 s
      after it began and turns red, also if the filament is wiggled at the outer lever meanwhile
      (each wiggle shorter than 0.1 s; a longer one ends the insertion, and the next one starts a
      new Stage-1 with its own 5 s).

## 8. Watchdog

- [ ] Normal boots and a whole print: no reset, and the SYS LED never flashes magenta.
- [ ] First boot after flashing (empty NVM): the motor-direction test and a full calibration that
      takes minutes finish with no reset.
- [ ] **(USB)** The 5 s recalibration hold: blue blink, NVM wipe, reboot into calibration, and no
      magenta or blue flash of the SYS LED or reset during it (this confirms a software reset stops
      the watchdog on this chip). If it resets instead (magenta flashes, calibration restarting
      about every second), switch the printer off and unplug it, which power-cycles a BMCU fed by
      the printer (on USB-C, unplug the USB-C instead); a power-on reset does stop the watchdog.
      Connect or disconnect the BMCU only with the printer unplugged. Power on again, calibrate (it
      runs with no watchdog) and report it.
- [ ] Optional, with a test build that hangs in the main loop while a motor runs: the motor stops,
      the BMCU reboots after about 1 s (0.67 to 1.6 s) with three magenta flashes, and the printer
      finds the AMS again.
- [ ] New, **(USB)**: unplug and plug in the USB-C five times: the SYS LED turns red about as fast
      as with the previous image and never flashes magenta or blue (a power-on is not a watchdog
      reset, so no flash whatever the backup registers hold; with a WCH-Link, its power-cycle step
      checks how a cold backup domain reads). The fault record's boot code adds only a few register
      accesses.
- [ ] New: after a normal power-on in the printer, the A1 finds AMS A as before.
- [ ] A hang or a trap on purpose, and the fault record, need a WCH-Link: see its section below.

## 9. Load and idle push limits

- [ ] Several loads of slot 1 from the printer, some of them right after the printer has cut the
      filament: each completes as before (about 31 s from the request to the filament in the
      extruder in Run 1), with no early stop. A snag that holds the gear for 1 s or more at 800 PWM
      or above now ends the whole load, braked until the printer's next command, and shows only as
      the printer's load timeout: note any load that stops early.
- [ ] Hold the filament at the spool side of the BMCU gear (or clamp the spool) and start a load:
      the motor stops about 1.7 s after the push starts instead of straining at full force; no LED
      changes. Note what the printer reports after its load timeout. Free the spool and load again
      from the printer: it loads normally.
- [ ] Let a load feed for a few seconds, then hold the spool: the motor stops about 1 s after the
      gear stops.
- [ ] After a normal unload from the printer screen (95 mm pull back), and for the old slot after a
      channel switch: no parked slot shows a steady red status LED, and the buffer settles at its
      rest position.
- [ ] Parked, loaded slot, spool held firmly so it cannot turn: press that slot's buffer to its low
      end and hold it (the 5 s recalibration hold needs every slot empty, so it cannot start here).
      The motor pushes about 1 s, then brakes, and the slot's status LED turns
      steady red. Let the buffer go back to the middle: the red clears. Press it again: it pushes
      about 1 s again, then red. Do not do this with the spool free: the push would feed the parked
      filament towards the printer for up to 10 s.
- [ ] With a slot braked red by that limit, keep the spool held and the buffer pressed while
      starting a load of that slot from the printer (the red lasts only while the buffer stays
      below 30 %). The red clears at once when the load starts; let go of the spool and the buffer
      as soon as it does, within about 1 s, because the send's stall check stops a held gear about
      1.7 s after the load starts. The load then runs normally.
- [ ] DM buffer gesture on a loaded slot (press the buffer below 10 %, hold it there about 1.5 s,
      then let go; the gesture allows at most 2 s from the press to the buffer back in the middle),
      with the spool held as in the steps above: the idle push starts as the buffer passes below
      30 % and brakes red about 1 s later, the red clears when the buffer is let go, and nothing
      else changes. A press let go less than about 1 s after the push starts ends it before its
      stall check: no red. With the spool free no red shows, but the idle push feeds filament for
      the whole press.
- [ ] Note what was never measured: the idle push's 10 s budget, its 1 s stall window at 420 PWM
      and above, and the silent brake at a rounded 28-29 % (a gear that stops just short of 30 %,
      braked without the red, and held until the reading falls to a rounded 26 %) all rest on the
      gear's speed and breakaway at about 510-565 PWM, which no one has measured on this unit. A
      parked slot that shows red with nothing wrong is a regression to report: note where its
      buffer rests. The limit decides silent or red on one rounded reading, so ADC noise near
      27.5 % can show red on a gear stopped at the band's edge (an open point in the audit).

## 10. ADC readings

- [ ] **(USB)** Boot with a calibration stored: dim red SYS LED, no calibration prompt, the same
      boot time to the eye as the previous image, and no status LED blinking blue afterwards. The
      buffer LEDs still follow the buffers (blue when pressed to the low end, red when lifted), so
      the ADC stream runs after the new calibration deadlines (a timeout shows on no LED, it only
      leaves an offset; with a WCH-Link, `g_adc_cal_timed_out` reads 1 after one, see its section).
- [ ] Normal use (a load, a print, an unload): no status LED ever blinks pure blue (all four
      together, 250 ms on, 250 ms off, with the buffer LEDs off), including right after flash
      writes (a slot's filament set, a load or an unload saved) and at the boot after the
      first-boot calibration and after the 5 s recalibration hold. That blink means the readings
      went stale for more than 100 ms and every motor is braked.

## Optional: WCH-Link

On the CH32V203's SWD pins (PA13 = SWDIO, PA14 = SWCLK), with WCH's OpenOCD and the BMCU on USB-C.
Connect only SWDIO, SWCLK and GND while the BMCU runs on USB-C: leave the link's 3.3 V unconnected
in every step, so two supplies never feed VDD and a power cycle really removes power.
Type each check as one line in the OpenOCD console (telnet port 4444): the watchdog keeps counting
while the core is halted, so a halt of more than about 0.67 s resets the chip, and a check split
over several lines reads as a hang. Addresses of RAM variables come from the ELF of the same
build: `nm .pio/build/a1_solo_autoload_rgboff/firmware.elf | grep <name>`.

- [ ] **(USB)** Read the live fault record: `halt; mdw 0x40006c04 10; resume`. One 32-bit word per
      register, BKP_DATAR1-10; only the low 16 bits count (`mdh 0x40006c04 10` reads half-words and
      misses DATAR6-10). Word 1 is 0xBB01; word 2's low byte holds the flags of the reset that
      started this run (bit 0 PIN, 1 POR, 2 SFT, 3 IWDG, 4 WWDG, 5 LPWR; 0x03 after a power-on) and
      its high byte the resets since the last power-on (0 after a power-on); word 3 is its check;
      word 4 the phase (1 boot, 2 calibration, 3 NVM read, 4 motion init, 5 bus init, then 6 to 11
      in the loop: AHUB, BambuBus, send, NVM, motion, RGB); words 5-6 the longest main-loop pass
      in 18 MHz SysTick ticks (low, high); words 7-10 are 0 (no trap in this run).
- [ ] **(USB)** Read the run before the last reset: `halt; mdb <g_blackbox_last> 16; resume`. Byte 0
      valid, 1 reset flags, 2 resets in a row, 3 phase, 4-7 longest pass, 8 trapped, 9 mcause,
      12-15 mepc (little-endian; `blackbox_t` in `src/blackbox.h`).
- [ ] **(USB)** Hang: `halt`, and leave the core halted. About 1 s later (0.67 to 1.6 s) the chip
      resets (if OpenOCD halts it again after the reset, `resume`): three magenta flashes (70 ms on,
      70 ms off), then red, no blue, about 420 ms. The live record's word 2 has IWDG set (low byte
      0x08 or 0x09) and its high byte one higher; `g_blackbox_last` shows trapped 0 and the phase
      of the step that was halted. If the chip does not reset within 2 s, the debug halt stops the
      watchdog on this chip: note it, `resume`, and do the trap step below anyway (a trap starts
      the watchdog and resets without a halt).
- [ ] **(USB)** Trap, as one line: `halt; reg pc 0x60000000; resume` (an unmapped address, so an
      instruction access fault). Within 0.67 to 1.6 s the chip resets: three magenta flashes, one
      blue flash, then red, about 490 ms in all (both sequences stay under 500 ms; a slow-motion
      video shows it). `g_blackbox_last` shows trapped 1, mcause 0x01 (or 0x02 if that region reads
      as zeros; note which), mepc 0x60000000 and the phase that was running; the live words 7-10 are
      0 again and word 2's high byte went up by one.
- [ ] **(USB)** Trap before the record is set up: `reset halt`, `bp <SystemInit> 2 hw`, `resume`;
      when it stops there, first `mdw 0x4002101c 1` (RCC_APB1PCENR: bits 27 and 28 clear) and
      `mdw 0x40007000 1` (PWR_CTLR: bit 8 clear); if any of them is set, `reset halt` was not a
      system reset, the record stays writable and a blue flash is then expected. Then
      `rbp all; reg pc 0x60000000; resume` (the watchdog is not started yet at this point, so there
      is no hurry; if the chip resets before the trap, the reset did not stop a running watchdog:
      note it). The trap handler starts the watchdog and the chip resets: magenta three times, no
      blue, and `g_blackbox_last` shows the run before `reset halt`, not a trap: the handler skips
      the record while the backup domain is off.
- [ ] **(USB)** Power cycle, with the WCH-Link's 3.3 V not connected (or unplug the link too, so it
      does not keep VDD up): unplug the USB-C for a few seconds, plug it in, reconnect and read
      `g_blackbox_last`. Valid 0 means a power cycle clears the backup registers (VBAT tied to VDD
      on the 370C); a valid record of the run before means the backup domain kept power. Note which
      in the log.
- [ ] **(USB)** Stale ADC readings: stop DMA1 channel 1 with `halt; mmw 0x40020008 0 1; resume` (its
      EN bit). Within about 100 ms every status LED blinks pure blue (250 ms on, 250 ms off, all
      four together) and every buffer LED goes off. About 0.5 s after the stop the firmware
      restarts the ADCs and the DMA, and the LEDs are normal again by themselves: the blink lasts
      about 0.4 s, one or two blue flashes (a slow-motion video shows it), and
      `halt; mdw 0x40020008 1; resume` reads bit 0 set again. If the blink goes on, no restart
      revived the stream: note it, then unplug and plug in the USB-C (with the link's 3.3 V not
      connected). With the printer, the same stops every motor, also one in the idle control or the
      manual empty pull. A slot braked red by the idle push limit (section 9, spool and buffer
      still held) shows red again as soon as the blink ends and its motor does not push again (an
      image before `1187db5` pushed for about 1 s more against the held gear, then went red). A
      loaded DM slot gets no new 120 mm push: this is a regression check only, the fix itself
      (`6d678c6`) needs a one-pass 'none' at the moment the stream stops and is host-tested in
      `test_dm_stage2`.
- [ ] After normal use, and after the stale step above, no ADC calibration wait gave up and the
      flash-write give-ups are 0: `halt; mdb <g_adc_cal_timed_out> 1; mdw <g_state_job> 1;
      mdw <g_fil_job> 12; resume`. `g_adc_cal_timed_out` is 1 once a calibration wait, at boot or
      on a restart, gave up after its 10 ms (note it). Each job is 3 words (`struct nvm_job` in
      `src/nvm_save_sched.h`) and its give-up count is the high 16 bits of its first word: the state
      job's word, and words 0, 3, 6 and 9 of the four filament jobs.

## Optional: logic analyser

PA10 = RX, PA12 = DE, channel LED data pins PA11/PA8/PB1/PB0.

- [ ] Largest gap between bytes inside printer frames (expected about 0) and smallest gap between
      frames. The RX resync threshold is 200 us, so no printer frame may contain a longer gap.
- [ ] DE goes low right after the last stop bit of every reply, and flash activity only starts
      after at least 1 ms of silence, or after about 200 us once a due write has waited 1 s without
      such a gap (toggle a spare GPIO around the flash calls to see it).
- [ ] No WS2812 bursts on the loaded channel's data pin while nothing changes (the old image sent a
      ~59 us burst every 10 ms).
- [ ] Decode the bus as 8E1: no parity or framing errors on real traffic.
- [ ] The bus frames around a tangle pause and the resume (stop_on_use, on_use, before_on_use,
      pull back, send_out): which release path the A1 actually uses.
- [ ] The lengths of the printer's 0x03, 0x04 and 0x05 frames (subtypes 00 and 01): at least 11,
      12, 8 and 26 bytes, the new minima. Note the confirm's length, which no capture has yet.
      After the confirm, a later 0x05 discovery probe (subtype 00) gets no reply: the registration
      is latched.
- [ ] The A1's 0x103 version request and our reply, the 0x05 probe and confirm sequence, and one
      idle and one on_use 0x03 exchange, compared byte for byte with `kReplyVersion`,
      `kOnlineDetect*`, `kReplyIdle` and `kReplyOnUse` in `test/test_bambubus_frames`: that turns
      the golden frames from what the code sends into what the A1 accepts.

## Reporting

Record, for every run, the image sha256, this checklist and anything unexpected: LED colours, the
printer's HMS codes, OrcaStudio log lines, in the [hardware test log](hardware-test-log.md). A
regression goes into the audit backlog with the commit that introduced it.
