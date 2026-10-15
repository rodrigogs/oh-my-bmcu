# Hardware test plan

What to check on the real A1 after flashing a build of this fork. Every image is built, unit-tested
on the host, reviewed and must pass CI, but it has not run on the hardware until it passes this
plan. Run the **basic** checks (no extra tools) for every new image. The **optional** ones need a
logic analyser or a WCH-Link on the BMCU and are for chasing a regression.

Steps marked **(USB)** need only the BMCU on USB-C power, with no printer; every other step needs
the printer. On USB power alone no motor turns (there is no 24 V), so a USB step can only check
LEDs.

Which image: [Run 1](hardware-test-log.md) exercised the image of `cec0e00`. Run 2 smoke-tested the
image of `68c77c6` (`a1_solo_autoload_rgboff`, sha256
`642dfb208ab8c7b7b2e5486d5a54353e74a617c0963fd90a5cd1967b7dea2857`, 52,716 bytes), which then
replaced it in normal use; a build of `cb1d94d`, the last commit before the upstream V11 merge, is
byte-identical to it. The merge of upstream V11 (`5b8e235`) takes V11's bus receive and transmit (a
DMA ring parsed from the main loop), its NVM layer (background flash jobs, a new layout), its
fail-closed buffer calibration and its motor-direction test, and reports version 11.00.00.00. Its
image has not run on the printer yet. Before it replaces the Run 2 image in normal use, it must pass
the [Run 3 smoke test](#run-3-smoke-test-the-v11-merge) below. If one of its steps fails, flash the
rollback image (section 0) and report it. Then run the steps marked "V11", which check what the
merge changed, and the fault cases still pending from Runs 1 and 2 (sections 4, 5, 7, 9 and 10).

Setup: Bambu Lab A1 on printer firmware 01.08.01.00, AMS type "AMS", one BMCU 370C, image
`env:a1_solo_autoload_rgboff` (AMS A, 9.5 cm retract, DM autoload, filament RGB off; CI artifact
`oh-my-bmcu-a1-<commit>-push` or a local `pio run`), flashed as described in
[README.md](README.md#flash) (`bmcu_flasher.py ... --mode usb --verify-last`). Flashing erases the
whole NVM: the first boot calibrates the buffers, and every slot reads "PETG, white" until it is set
again.

## Run 3 smoke test (the V11 merge)

The minimum before the merged image replaces the Run 2 image in normal use, as Run 2 did for its own
image. Each item is a step of the sections below, where its pass criteria are given in full.

- [ ] Section 0: the image's sha256 is `d76731bd…` and the CI run summary of the pushed commit shows
      the same hash; the rollback image is at hand; no filament at any switch; printer unplugged;
      BMCU on USB-C.
- [ ] Section 1: no magenta or blue flash after the flash; the first calibration ends with the
      green blink (a red abort: power-cycle and calibrate again, and note it).
- [ ] Section 1: reconnected with the printer unplugged and no filament at any switch, then powered
      on: SYS LED red, then white at the first heartbeat; the printer shows AMS A.
- [ ] Section 1, V11: over the printer's LAN MQTT, module `ams/0` reports `11.00.00.00` `AMS08`,
      and no HMS beyond the known 0500-0400-0001-0044 and 0500-409D.
- [ ] Section 2, V11: slot 1's type and colour set from OrcaStudio: the printer shows the new values.
- [ ] Section 2: a load of slot 1 (DM autoload, then the printer's load).
- [ ] Section 3: a test print from slot 1: the status LED is steady cyan while it prints, the print
      finishes, and the unload at the end pulls about 95 mm with no red blink afterwards.
- [ ] Section 2, V11: after that print, change slot 1's type or colour and power-cycle the
      printer: slot 1 keeps the new values.
- [ ] Section 8: no magenta flash of the SYS LED at the boots and during the print.

## 0. Before flashing

- [ ] **(USB)** Note the sha256 of the image and the commit it came from, and check in the
      [log](hardware-test-log.md) whether an image with that sha256 already passed. The merged
      image, `env:a1_solo_autoload_rgboff`, built locally (macOS, toolchain-riscv-mac `e6360e0f`,
      platform ch32v `8cf3b51c`, SDK `34b1b783`): sha256
      `d76731bd8fcebbe7756a9c1c66ee539e7ae29bfd955d0a3b035ca835a3a8d61f` (59,156 bytes). The CI run
      summary of the pushed commit (its build job lists each env's sha256 and size) must show the
      same hash; if it does not, do not flash, report it. The A/B image of section 6,
      `env:a1_solo_autoload_rgboff_no_boot_restore`: sha256
      `b571f812fb0c545b88f2e34de4607e832c9fb11909352b5b67a54199c5502a40` (59,176 bytes).
- [ ] **(USB)** Have the rollback image ready: the last hardware-validated image, built at
      `cb1d94d` (identical to `68c77c6`'s, Run 2), sha256
      `642dfb208ab8c7b7b2e5486d5a54353e74a617c0963fd90a5cd1967b7dea2857` (52,716 bytes). Get it
      from the CI artifact `oh-my-bmcu-a1-cb1d94d…-push`, or check out `cb1d94d` in a separate
      worktree and run `pio run -e a1_solo_autoload_rgboff` there; check its sha256 before use. It
      is flashed the same way. Its NVM layout is the one before the merge; the flasher erases the
      NVM, so after a rollback the first boot calibrates again and the slots start at "PETG, white".
- [ ] **(USB)** V11: no filament at any channel's switches, from before the flash until the first
      boot in the printer has finished (section 1). V11's calibration aborts when a switch sees
      filament: its idle sampling rejects a switch reading of 1.40 V or more, and its last phase
      waits for every switch to read empty. And V11's motor-direction test, which can only work at
      the first boot with the printer's 24 V, skips a channel with filament at its switches: that
      channel's motor then drives nothing at all until a later boot with its switches empty.
- [ ] **(USB)** Unplug the printer, disconnect the BMCU, connect it by USB-C.

## 1. First boot and calibration

- [ ] **(USB)** After flashing, the SYS LED shows no magenta and no blue flash (the flasher's reset
      is not a watchdog reset), then the calibration starts. The SYS LED stays red through it: the
      bus starts only afterwards.
- [ ] **(USB)** V11: calibrate, one buffer at a time (the procedure of the upstream README,
      "Calibration / Re-calibration"; upstream video: https://www.youtube.com/watch?v=Hn_DNzSmhuc).
      Every connected buffer is calibrated:
      - All buffer LEDs blink yellow slowly for about 1 s while the idle buffers and switches are
        sampled: touch nothing.
      - Then each buffer in turn, channel 1 first. While its LED blinks blue (0.2 s on, 0.2 s off),
        push it fully to its low end (the end the extruder pulls it to when the filament is tight,
        the same end you hold to recalibrate; upstream: "all the way in", about 1 s) and let it
        return to rest. Once it has rested for 0.7 s, two quick yellow blinks confirm the step.
      - While it blinks red, pull it fully to the other end and let it return: two yellow blinks
        again. Its LED then stays dim yellow while the other buffers are calibrated.
      - The order matters: the end the first step moves to becomes the low end.
      - After the last buffer, leave every buffer at rest. Once all are at rest (within 80 mV of
        their centre) and every switch reads empty for 1.2 s, the calibration is saved and all
        buffer LEDs blink green twice (0.22 s on, 0.22 s off). Later boots show no blink and go
        straight on.
- [ ] **(USB)** V11, fail-closed: any failure aborts the whole run. All buffer LEDs blink red twice
      (0.22 s on, 0.22 s off), nothing is saved, and every connected buffer's LED then stays dim red:
      no channel is calibrated, and none drives (no autoload, auto-unload or manual pull; the
      printer's moves do not turn the motor, and a printer unload of such a channel ends at once
      with the status LED blinking red until the filament is out). The causes:
      - a buffer that moved, or a switch that saw filament, during the yellow sampling (the readings
        spread over more than 40 mV, or a switch at 1.40 V or more): the red blink then comes right
        after the yellow phase;
      - a step not finished within 30 s;
      - a first step whose end is less than 190 mV from the centre (a shallow press; the fork's
        floor, V11 alone accepts 100 mV): it aborts before its two yellow blinks. Press the buffer
        fully to its end;
      - a range with a side under 50 mV, or under 120 mV in all, or a reading outside 0.05-3.25 V;
      - the final rest not reached within 30 s, or the save failing.

      A red abort means: power-cycle (on USB-C, unplug the USB-C and plug it in again; in the
      printer, switch the printer off and on) and calibrate again. The 5 s recalibration hold does
      not start on a channel without a valid calibration (a fork rule). There is no fallback range
      any more: Run 1's first calibration, which ended on the fallback, would be a red abort on
      this image. Note every abort and what came before it.
- [ ] **(USB)** Optional, to see the abort: while one buffer blinks blue, leave it untouched for
      30 s: the red blink, then every buffer LED steady dim red. Unplug and plug in the USB-C: the
      calibration starts again from the yellow phase. Calibrate properly.
- [ ] Reconnect to the printer only while it is unplugged, with no filament at any switch, then
      power on. On the first boot after a flash or a recalibration, each gear may turn for a moment
      (the motor-direction test, at most about 2 s). SYS LED: red until the printer's first
      heartbeat, then white. The printer shows AMS A.
- [ ] V11: over the printer's LAN MQTT, `ams_exist_bits` is 1, module `ams/0` reports
      `11.00.00.00` `AMS08` (it was 10.50.00.00 up to Run 2), and no HMS shows beyond the known
      cosmetic 0500-0400-0001-0044 and the 0500-409D block that OrcaStudio gets past.
- [ ] With no filament in any channel, each buffer LED at rest shows dim amber (the buffer at its
      calibrated centre, 49-51 %). V11 shows it only while every channel is empty. It is not a
      fault.
- [ ] Direction check, printer on and the slot still empty: push the buffer to its low end for a
      moment (well under 5 s, or it starts a recalibration) and let it go: the buffer's LED is blue
      while it is down. On this AUTOLOAD image that push also starts upstream's buffer-tap autoload:
      the channel's status LED turns yellow and the gear feeds forward for about 5 s, then the LED
      is red for about 0.1 s and goes off. That is expected and says nothing about the direction.
      Wait about 6 s until the motor has stopped, then lift the buffer: its LED turns red, and the
      motor may turn backwards while it is up (upstream's manual unload). If the buffer's colours
      are the other way round, the calibration was done in the wrong order: recalibrate.

## 2. Bus, filament info and flash

- [ ] Load PLA into slot 1 (AUTOLOAD). The LEDs go through the load colours and end steady; the
      loaded slot's LED does not flicker while nothing changes.
- [ ] V11: set slot 1's type and colour from OrcaStudio (or the printer screen): within a few
      seconds the printer shows the new values, read back from the BMCU. Power-cycle the printer:
      slot 1 keeps them. V11 drops a set-filament frame (0x08) shorter than 45 bytes, and a long
      one (0x218) with fewer than 34 data bytes; the A1's lengths were never captured. If the slot
      does not take the new values at all, note it: the follow-up is to restore the fork's lenient
      decode (25 / 18 bytes).
- [ ] Change slot 1 type/colour three times within a few seconds (printer screen or OrcaStudio).
      Wait about 5 s after the last change, power-cycle the printer: slot 1 keeps the last values.
      Since V11 each change only queues a background flash job, and the main loop programs it one
      step per pass while the bus is idle, with no debounce.
- [ ] V11: change slot 1's filament at least 7 times, about 1 s apart, then power-cycle: the last
      values stay. A slot's page holds 6 records, so the seventh goes through V11's rollover: the
      record into the scratch page (page 14), the filament page erased and the record written to
      its slot 0, then the scratch page erased. It also confirms that erased flash reads
      `0xE339E339` on this chip: the journal takes only such words as free.
- [ ] V11: the 5 s recalibration keeps the filament slots (it used to wipe them to "PETG, white",
      audit #14). Set slot 1 to a type and colour other than PETG white. Unload it, take the
      filament out of every switch, switch the printer off and unplug it, connect the BMCU by
      USB-C, and do the recalibration hold (section 8), then calibrate (section 1). Reconnect to the
      printer (unplugged, switches still empty), power on, insert the filament: slot 1 shows the
      type and colour set before. Optional, in the printer instead of on USB-C: note whether the
      A1 finds AMS A again by itself after the BMCU's reboot, or only after a printer power cycle.
- [ ] V11: after a whole print (section 3), change slot 1's type or colour, wait about 5 s and
      power-cycle the printer: the new values stay. V11 latches a flash fault on any failed or
      timed-out erase or program and then drops every NVM write until the next reboot, with no
      sign on any LED (a debugger reads it, see the WCH-Link section); this edit is the only
      visible check that no fault latched during the print.
- [ ] Leave the printer idle for more than 5 minutes (covers a SysTick wrap): the SYS LED stays
      white and the slot still answers.
- [ ] **(USB)** With the BMCU on USB-C only (not connected to the printer), the SYS LED stays red,
      with no printer heartbeat, for more than 4 minutes.
- [ ] Power-cycle the printer three times: AMS A is found every time. The online-detect probe
      (0x05, subtype 00) and confirm (subtype 01) need at least 8 and 26 bytes (V11's minima, as
      the fork's); no capture of the A1's exists, so a confirm shorter than 26 bytes would be
      dropped and the AMS not found. If a sniffer is at hand, log the confirm's length (see the
      logic analyser section).
- [ ] V11: a load, a print and an unload of slot 1 show no HMS error and no step stalls. The
      printer's 0x03 and 0x04 frames quoted in the source (12 and 13 bytes; this A1's were never
      captured) meet V11's minima of 12 and 13 exactly (the fork had 11 and 12), so one byte less
      would be dropped.
- [ ] V11: load and unload slot 1 a few times, then power-cycle the printer: the printer shows the
      right slot as loaded (or none) and slot 1's type and colour come back. V11's loaded-channel
      log (a new record tag, eight pages) and its background writes have not run on the board.
- [ ] Switch the printer off during an unload of slot 1 (while the BMCU retracts), then back on:
      the printer shows slot 1 either still loaded or unloaded, never another slot, and the next
      load and unload of slot 1 work. The host model does not cover a half-erased page whose words
      read as erased but are weak.
- [ ] Optional, only with a switch or breakout that opens just the A/B data pair, wired while the
      printer is unplugged: opening it with the printer on turns the SYS LED red within about 1 s
      and stops the motors; while it is red, lifting the buffer moves no motor, and an auto-unload
      that was running stops and does not restart. Closing it turns the LED white and the slot
      works. Do not do this by pulling the bus cable from a powered printer (the upstream safety
      notes forbid it), and a BMCU on USB-C power alone proves nothing here: without the printer's
      24 V no motor turns anyway. Without such a breakout, the host tests (test_auto_unload,
      test_bus_link) are all there is for this.
- [ ] V11, optional, with the same breakout: a heartbeat loss with the BMCU powered re-arms the
      online detect (fixed AMS A mode). Keep the pair open for a few seconds: the SYS LED turns red
      about 1 s after the last heartbeat. Close it: the LED turns white again once the printer's
      heartbeats are back, and the printer still shows AMS A and the slot works, also if it ran
      its discovery (0x05) again meanwhile, which the BMCU now answers instead of ignoring until a
      reboot. Note whether the printer showed an AMS error.

## 3. Printing and unloading

- [ ] A test print from slot 1 (for example the 15 mm cube) starts, prints and unloads at the end.
      While it prints, the channel's status LED is steady cyan (on_use). The unload pulls about
      95 mm and the channel LED does not blink red afterwards. V11 counts the gear on a 64-bit
      odometer, and the fork measures the pull back in gear counts from it, so the length holds
      at any odometer value.
- [ ] A long print (1 h or more) finishes with no "AMS offline/communication" errors and no motion
      timeouts.
- [ ] Cut power mid-print and restore it: the BMCU still reports slot 1 loaded (resume), and no
      tangle error appears unless the spool is really stuck. Since V11 a loaded slot is restored at
      boot only while its switches see filament and its calibration is valid; otherwise it is set
      unloaded.

## 4. Tangle detection (motor safety)

- [ ] Hold the spool for less than 0.5 s while printing: the print does not pause.
- [ ] Hold the spool until the print pauses: red LED and the printer's tangle error, about 0.5 s
      later than with upstream.
- [ ] Resume with the spool still held and nothing else done: it pauses again at once, motor
      braked, no push (also when the printer reloads the channel, send_out then before_on_use).
- [ ] Free the spool, feed filament until the buffer is above 40 %, then resume: printing should
      continue. While paused, holding the buffer at or above about 52 % for 1 s should also turn the
      red LED off. Both depend on what the A1 sends on a resume, which has not been observed yet:
      note which of the two worked, and what the printer showed if neither did. Since V11 the
      BMCU ignores a before_on_use that does not follow a send_out, so a resume from the pause is
      released by the printer's on_use.
- [ ] While paused after a tangle, touch nothing for 10 s: note whether the red LED goes out by
      itself (a buffer resting a few mV above neutral releases the latch). If it does, the resume
      must pause again about 0.5 s after the buffer drops below 40 %.
- [ ] If the A1 unloads after the tangle: the latch holds through the pull back (100 mm after a
      tangle), idle and the reload, unless the buffer is lifted above 85 %. The motor stays silent
      meanwhile, also once the printer has deselected the slot and the buffer sits below 30 % (the
      red LED stays on).
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
      40 % while the motor strains at 800 PWM or more for more than 20 s of push (the anti-stall's
      0.5 s rests pause the count, so with a gear that stalls or crawls, likely on the 0.2 mm
      nozzle, this takes about 32 s). The channel then brakes and its LED turns red; the print only
      pauses if the buffer then stays below 40 % for 0.5 s.

## 5. Unload limits

- [ ] Pull the filament out of the BMCU inlet during an unload: the gear stops after about 95 mm,
      the slot shows empty, and another slot loads without a power cycle.
- [ ] Hold the filament during a pull back: the motor stops about 1.6 s in, the channel LED blinks
      red (1 s on, 1 s off) and the other slots are still served. Note what the A1 reports. The
      blinking stops when the filament is pulled out or the slot is used again.
- [ ] DM autoload: a normal insertion still feeds 120 mm.
- [ ] A normal buffer-lift unload (key at 'both': lift the buffer to the top and let it go): the
      purple retract runs until the filament leaves the switches (plus 1.5 s) or the buffer drops,
      as before; it does not stop early, and the channel LED does not blink red afterwards. Note how
      long the unload of a whole tube takes: the auto-unload ends by itself at 15 s.
- [ ] Hold the filament firmly on the printer side of the BMCU (or leave it stuck in the tool head)
      so the retract pulls against it and the gear cannot turn; a clamped spool does not stop a
      retract, the filament only goes slack. Do the lift gesture, and as soon as the buffer is back
      in the middle, hold it there by hand (45-55 %, in any case above 35 %) while the retract
      pulls: the 850 PWM retract stops after about 1 s (not 15 s), and the channel LED blinks red
      (1 s on, 1 s off) until the filament is pulled out of the switches or the printer loads,
      prints or unloads that slot. Free the filament and lift again: it unloads normally. Without
      the hand the retract drags the buffer below 35 % and ends at once with no blink: that is its
      normal end, not a failure (the stall case is also host-tested, in `test_auto_unload`). A gear
      that slips on the filament is not caught: the retract then runs to its 15 s end.
- [ ] With no filament at the switches, hold the buffer up: the motor turns backwards while it is
      held (the manual empty pull, 700 PWM, which has no stall limit) and stops when it is let go.
- [ ] V11: neither the buffer-lift unload nor the manual empty pull runs on a channel whose
      calibration is not valid (dim red buffer LED) or whose DM autoload has failed (red status
      LED): lifting its buffer drives nothing.

## 6. Upstream #148 A/B test

Only if the A1 ever stops finding the AMS after a power cycle with a slot loaded. Each flash erases
the NVM: after it, calibrate again with no filament at any switch (sections 0 and 1), then set and
load the slot again before the power cycles.

- [ ] With the default image (`env:a1_solo_autoload_rgboff`, sha256 `d76731bd…`): load a slot into
      the extruder, then power-cycle the printer 5 times. Note each time whether the printer finds
      AMS A.
- [ ] Flash `env:a1_solo_autoload_rgboff_no_boot_restore` (sha256 `b571f812…`, from the same CI
      artifact, or `pio run -e a1_solo_autoload_rgboff_no_boot_restore`, then
      `.pio/build/a1_solo_autoload_rgboff_no_boot_restore/firmware.bin`) and repeat the same 5 power
      cycles. Until the printer uses the loaded slot, its LED shows the idle colour. A print cut by
      a power loss must still resume, and an unload right after boot must still retract.
- [ ] Repeat both with several slots holding filament and none loaded in the extruder.
- [ ] In both images, a slot saved as loaded whose switches read empty at boot, or whose calibration
      is not valid, is set unloaded at boot (V11): the printer shows no slot loaded.
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
- [ ] Optional, with the outlet blocked: three buffer aborts, then red. V11: after the third abort
      the motor first retracts at 900 PWM, for up to 1.5 s, then up to 1.5 s more while the key
      reads 'external only', at most 120 mm, and stops when both switches read empty. A free gear
      thus pulls the filament out: the red clears about 0.1 s later and the slot is empty again. A
      jammed one stays red with the motor off. Also while flicking the lever. With the filament
      jammed in the gear and the tip at the inner lever, flicking the lever and moving the buffer
      (past 75 %, or lift gestures) push the motor about 6 s in all, then red; the fail retracts
      can add up to 3 s backwards (at most 5.998 s + 3 s of drive, `test_dm_stage2`), then the
      motor stays off. A time or stall limit of the Stage-2 run fails it at once, with no retract.
- [ ] Lift gesture during the Stage-2 push (buffer to 80 % or more and back to the middle within
      1 s): the filament comes out. The LED is red only while the buffer is held up, then purple,
      and does not stay red; inserted again, the filament gets a full autoload.
- [ ] Push filament into the outer switch by hand several times, some of them with a slightly
      wobbly push at the lever: Stage-1 (yellow), then Stage-2 (120 mm) complete every time, and the
      filament is never left waiting at the outer switch. A key reading 'none' (both switches open)
      for less than 100 ms only pauses the drive.
- [ ] With a channel loaded (white) or failed (red), pull the filament fully out and keep it out:
      within about 0.1 s the channel counts as empty (the red clears, the LED goes off), and the
      printer shows the slot empty at once. Insert it again: the autoload runs again.
- [ ] Optional: on a failed (red) channel, flick the outer lever quickly, if you can, so both
      switches open only for a moment: it stays red and does not push. Only a removal of 0.1 s or
      more clears it. The case is host-tested
      (`test_a_key_glitching_to_none_keeps_a_failed_run_failed`).
- [ ] Hold the filament so it cannot reach the inner switch: Stage-1 stops pushing about 5 s after
      it began and turns red, also if the filament is wiggled at the outer lever meanwhile (each
      wiggle shorter than 0.1 s; a longer one ends the insertion, and the next one starts a new
      Stage-1 with its own 5 s). V11: it then retracts at 900 PWM for up to 1.5 s; let the filament
      go with it. If that takes the filament out of the outer switch, the red clears about 0.1 s
      later and the next insertion autoloads; if not, the channel stays red with the motor off until
      the filament is pulled out.
- [ ] V11: Stage-1 starts only once the switches have read empty for 0.1 s with the channel idle.
      Leave filament at the outer switch only (not at the inner one) and power-cycle the printer:
      nothing pushes it at boot. Pull it out and insert it again: the autoload runs.
- [ ] V11: after an unload (from the printer, or a buffer-lift unload), the autoload does not push
      the filament back while its tip still sits at the outer switch; it runs again only once the
      filament has been out of both switches for 0.1 s and is inserted again.

## 8. Watchdog

- [ ] Normal boots and a whole print: no reset, and the SYS LED never flashes magenta.
- [ ] First boot after flashing (empty NVM): a full calibration that takes minutes (before the
      watchdog starts), then in the printer the motor-direction test (up to 2 s) finish with no
      reset.
- [ ] **(USB)** The 5 s recalibration hold, with no filament at any switch: hold one calibrated
      buffer at its low end for 5 s. All buffer LEDs blink blue for 3 s (0.15 s on, 0.15 s off),
      then the BMCU reboots into calibration (section 1), with no magenta or blue flash of the SYS
      LED and no reset during it (this confirms a software reset stops the watchdog on this chip).
      V11 clears only the calibration and motion records: the filament slots stay (section 2), and
      the next boot in the printer runs the direction test again, so keep the switches empty until
      it has run. If clearing them fails, there is no blue blink: every buffer LED turns dim red, no
      channel counts as calibrated, and the BMCU goes on without rebooting; power-cycle it and
      report it. If it resets instead
      (magenta flashes, calibration restarting about every second), switch the printer off and
      unplug it, which power-cycles a BMCU fed by the printer (on USB-C, unplug the USB-C instead);
      a power-on reset does stop the watchdog. Connect or disconnect the BMCU only with the printer
      unplugged. Power on again, calibrate (it runs with no watchdog) and report it.
- [ ] Optional, with a test build that hangs in the main loop while a motor runs: the motor stops,
      the BMCU reboots after about 1 s (0.67 to 1.6 s) with three magenta flashes, and the printer
      finds the AMS again.
- [ ] **(USB)** Unplug and plug in the USB-C five times: the SYS LED turns red about as fast as
      with the previous image and never flashes magenta or blue (a power-on is not a watchdog
      reset, so no flash whatever the backup registers hold; with a WCH-Link, its power-cycle step
      checks how a cold backup domain reads). The fault record's boot code adds only a few register
      accesses.
- [ ] After a normal power-on in the printer, the A1 finds AMS A as before.
- [ ] A hang or a trap on purpose, and the fault record, need a WCH-Link: see its section below.
      V11 adds the FLASH phase (12, the background flash step), after RGB (11).

## 9. Load and idle push limits

- [ ] Several loads of slot 1 from the printer, some of them right after the printer has cut the
      filament: each completes as before (about 31 s from the request to the filament in the
      extruder in Run 1), with no early stop. A snag that holds the gear for 1 s or more at 800 PWM
      or above ends the whole load, braked until the printer's next command, and shows only as the
      printer's load timeout: note any load that stops early.
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
      The motor pushes about 1 s, then brakes, and the slot's status LED turns steady red. Let the
      buffer go back to the middle: the red clears. Press it again: it pushes about 1 s again, then
      red. Do not do this with the spool free: the push would feed the parked filament towards the
      printer for up to 10 s.
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
      the ADC stream runs after the calibration deadlines (a timeout shows on no LED, it only
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
      word 4 the phase (1 boot, 2 calibration, 3 NVM read, 4 motion init, 5 bus init, then 6 to 12
      in the loop: 6 AHUB, 7 BambuBus, 8 send, 9 NVM, 10 motion, 11 RGB, 12 FLASH; FLASH runs
      between motion and RGB, and the bus receive parser also runs inside NVM, FLASH and RGB);
      words 5-6 the longest main-loop pass in 18 MHz ticks (low, high); words 7-10 are 0 (no trap
      in this run).
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
- [ ] After normal use, and after the stale step above, no ADC calibration wait gave up and no
      flash step failed: `halt; mdb <g_adc_cal_timed_out> 1; mdb <g_flash_fault> 1; resume` (the
      ELF names the second `_ZL13g_flash_fault`). `g_adc_cal_timed_out` is 1 once a calibration
      wait, at boot or on a restart, gave up after its 10 ms (note it). `g_flash_fault` is 1 once
      a background erase or program since this boot timed out or failed its read-back: V11 then
      drops every NVM write until the next reboot (note it, and which edit or move came before).
      The fork's per-job give-up counts (`g_state_job`, `g_fil_job`) are gone with V11's flash
      jobs.

## Optional: logic analyser

PA10 = RX, PA12 = DE, channel LED data pins PA11/PA8/PB1/PB0.

- [ ] Largest gap between bytes inside printer frames (expected about 0) and smallest gap between
      frames. The main-loop parser drops a frame in progress once it has seen no new byte for
      200 us between its polls, in practice 200 us plus up to about two loop passes after the last
      byte, so no printer frame may contain a longer gap, and a frame that starts sooner than that
      after a broken one can be lost with it.
- [ ] DE goes low right after the last stop bit of every reply, and a background flash step starts
      only while the bus is idle: no reply queued or being sent, no frame being received, nothing
      left unparsed (toggle a spare GPIO around `Flash_background_run` to see it).
- [ ] No WS2812 bursts on the loaded channel's data pin while nothing changes (the image before
      Run 1 sent a ~59 us burst every 10 ms), and none that starts while a reply is being sent or a
      frame received (V11 puts a strip off while the bus is busy; a frame can still begin during a
      burst).
- [ ] Decode the bus as 8E1: no parity or framing errors on real traffic.
- [ ] The bus frames around a tangle pause and the resume (stop_on_use, on_use, before_on_use,
      pull back, send_out): which release path the A1 actually uses.
- [ ] The lengths of the printer's 0x03, 0x04 and 0x05 frames (subtypes 00 and 01): at least 12,
      13, 8 and 26 bytes, V11's minima. Note the confirm's length, which no capture has yet, and the
      lengths of a set-filament frame (0x08, at least 45 bytes) and of a long 0x218 (at least 34
      data bytes). After the confirm, a later 0x05 discovery probe (subtype 00) gets no reply while
      the heartbeats flow: the registration is latched.
- [ ] Printer retries (the same request sent again) mean the BMCU dropped a frame. V11's heartbeat
      shortcut in the parser can fire on bytes inside another frame's payload and drop more than
      one frame (a follow-up of the merge): note any burst of retries and the bytes before it.
- [ ] The A1's 0x103 version request and our reply (version `00 00 00 0B`, "AMS08"), the 0x05
      probe and confirm sequence, and one idle and one on_use 0x03 exchange, compared byte for byte
      with `kReplyVersion`, `kOnlineDetect*`, `kReplyIdle` and `kReplyOnUse` in
      `test/test_bambubus_frames`: that turns the golden frames from what the code sends into what
      the A1 accepts.

## Reporting

Record, for every run, the image sha256, this checklist and anything unexpected: LED colours, the
printer's HMS codes, OrcaStudio log lines, in the [hardware test log](hardware-test-log.md). A
regression goes into the audit backlog with the commit that introduced it. A smoke test that fails
goes back to the rollback image of section 0 first.
