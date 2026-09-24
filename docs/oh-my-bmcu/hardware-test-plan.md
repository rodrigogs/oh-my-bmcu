# Hardware test plan

What to check on the real A1 after flashing a build of this fork. Every change on `main` was built,
unit-tested on the host, reviewed and passed CI, but has not run on the hardware until an image
passes this plan. Run the **basic** checks (printer only) for every new image. The **optional**
ones need a logic analyser on the BMCU and are for chasing a regression.

Setup: Bambu Lab A1 on printer firmware 01.08.01.00, AMS type "AMS", one BMCU 370C, image
`env:a1_solo_autoload_rgboff` (CI artifact or local `pio run`), flashed as described in
[README.md](README.md#flash) (`bmcu_flasher.py ... --mode usb --verify-last`).

## 0. Before flashing

- [ ] Note the sha256 of the image and the commit it came from.
- [ ] Remove all filament from the BMCU. Unplug the printer, disconnect the BMCU, connect it by
      USB-C.

## 1. First boot and calibration

- [ ] After flashing, calibrate properly: move each buffer to both ends when its LED prompts
      (upstream video: https://www.youtube.com/watch?v=Hn_DNzSmhuc). The end blink is green, and later
      boots show no red flash.
- [ ] Optional, to check the fallback: let one buffer time out. It flashes red quickly at the end, then
      for about 0.7 s at every boot; the printer still finds the AMS. Recalibrate it properly and the
      boot flash stops.
- [ ] Reconnect to the printer only while it is unplugged, then power on. SYS LED: red until the
      printer's first heartbeat, then white. The printer shows AMS A.

## 2. Bus, filament info and flash

- [ ] Load PLA into slot 1 (AUTOLOAD). The LEDs go through the load colours and end steady; the
      loaded slot's LED does not flicker while nothing changes.
- [ ] Change slot 1 type/colour three times within a few seconds (printer screen or OrcaStudio).
      Wait at least 2 s, power-cycle the printer: slot 1 keeps the last values (it used to fall back
      to "PETG, white").
- [ ] Change slot 1's filament at least 7 times, about 1 s apart, then power-cycle: the last values
      stay. This goes through the page erase every 6 records and confirms that erased flash reads
      `0xE339E339` on this chip, which every save relies on.
- [ ] Leave the printer idle for more than 5 minutes (covers a SysTick wrap): the SYS LED stays
      white and the slot still answers.
- [ ] Only if safe: interrupt the bus while the BMCU stays powered (for example BMCU on USB-C power
      and the bus data unplugged). The SYS LED turns red within about 1 s, the motors stop, and it
      stays red for more than 4 minutes. When the bus returns the LED turns white and the slot works.

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
      braked, no push.
- [ ] Free the spool, feed filament until the buffer is above 40 %, then resume: printing continues.
      While paused, holding the buffer at or above about 52 % for 1 s also turns the red LED off.
- [ ] If the A1 unloads after the tangle: the latch holds through the pull back, idle and the
      reload, unless the buffer is lifted above 85 %.

## 5. Unload limits

- [ ] Pull the filament out of the BMCU inlet during an unload: the gear stops after about 95 mm,
      the slot shows empty, and another slot loads without a power cycle.
- [ ] Hold the filament during a pull back: the motor stops about 1.6 s in, the channel LED blinks
      red (1 s on, 1 s off) and the other slots are still served. Note what the A1 reports. The
      blinking stops when the filament is pulled out or the slot is used again.
- [ ] DM autoload: a normal insertion still feeds 120 mm.

## 6. Upstream #148 A/B test

Only if the A1 ever stops finding the AMS after a power cycle with a slot loaded.

- [ ] With the default image (`env:a1_solo_autoload_rgboff`): load a slot into the extruder, then
      power-cycle the printer 5 times. Note each time whether the printer finds AMS A.
- [ ] Flash `env:a1_solo_autoload_rgboff_no_boot_restore` and repeat the same 5 power cycles. Until
      the printer uses the loaded slot, its LED shows the idle colour. A print cut by a power loss
      must still resume, and an unload right after boot must still retract.
- [ ] Repeat both with several slots holding filament and none loaded in the extruder.
- [ ] If only the default image loses the AMS, the boot restore is the cause: report it with both
      counts.

## 7. DM autoload re-arm

- [ ] Insert filament: Stage-2 feeds about 120 mm past the inner switch, once.
- [ ] Nudge the filament so a switch flickers without removing it: no second 120 mm push.
- [ ] Pull the filament out past both switches and insert it again: Stage-2 runs again.

## 8. Watchdog

- [ ] Normal boots and a whole print: no reset, and the SYS LED never flashes magenta.
- [ ] First boot after flashing (empty NVM): the motor-direction test and a full calibration that
      takes minutes finish with no reset.
- [ ] The 5 s recalibration hold: blue blink, NVM wipe, reboot into calibration, and no magenta
      flash or reset during it (this confirms a software reset stops the watchdog on this chip).
- [ ] Optional, with a test build that hangs in the main loop while a motor runs: the motor stops,
      the BMCU reboots after about 1 s (0.67 to 1.6 s) with three magenta flashes, and the printer
      finds the AMS again.

## Optional: logic analyser

PA10 = RX, PA12 = DE, channel LED data pins PA11/PA8/PB1/PB0.

- [ ] Largest gap between bytes inside printer frames (expected about 0) and smallest gap between
      frames. The RX resync threshold is 200 us, so no printer frame may contain a longer gap.
- [ ] DE goes low right after the last stop bit of every reply, and flash activity only starts
      after at least 1 ms of silence (toggle a spare GPIO around the flash calls to see it).
- [ ] No WS2812 bursts on the loaded channel's data pin while nothing changes (the old image sent a
      ~59 us burst every 10 ms).
- [ ] Decode the bus as 8E1: no parity or framing errors on real traffic.
- [ ] The bus frames around a tangle pause and the resume (stop_on_use, on_use, before_on_use,
      pull back, send_out): which release path the A1 actually uses.

## Reporting

Record, for every run, the image sha256, this checklist and anything unexpected: LED colours, the
printer's HMS codes, OrcaStudio log lines. A regression goes into the audit backlog with the commit
that introduced it.
