# Box HW Test

Hardware validation and piece calibration firmware for the Gaspetto GBox programming board (STM32F411CE BlackPill).

## Purpose

The GBox programming board reads 20 piece slots through five ADS1115 ADCs and drives three APA106 status LEDs (dedicated FastLED APA106 controller on PB8). This firmware runs two runtime test modes over the same build:

1. **Health mode** — power rails, five ADS1115 probes and the NRF24 radio are exercised on demand (`s`), then each probe change is reported as it happens; the APA106 animations run from the LED menu instead
2. **Calibration mode** — a capture of how the positive hall sensor reads each actual piece, used to derive the per-slot threshold table

The production and test firmware must agree on thresholds, so both read the same header `soft/emu-pc/targets/gbox/include/piece_thresholds.h` (symlinked into this project's `include/`). Both tables live in that header: production `GBox.cpp` still decodes with the legacy global defines, while this firmware decodes with the per-slot rows. Porting production to the per-slot scan and dropping the legacy defines is a planned separate step.

## Runtime Modes

- **Health** (default at boot) — idles until a key asks for a test (`s` runs the cycle); the board LED heartbeat runs while quiet, and each cycle reports probe/radio changes against the previous one
- **Calibration** — keeps the 5V LED rail off, raises the 3V3 sensor rail for each reading rather than holding it on, and allows piece labeling

## USB Commands

| Key | Action |
|-----|--------|
| `h` | Health mode |
| `c` | Calibration mode |
| `s` | Run the active mode test once |
| `?` | Print the active mode and rail states |
| `m` | Continuous readout of a slot with a live decoded piece (`<slot>m`, the number is required) |
| `x` | Frozen monitor: waits without sampling, SPACE opens one clean 3 s measurement (`<slot>x`, the number is required) |
| `j` | Start or resume jig mode: piece census then slot gains, no keypress during the walk (calibration only) |
| `k` | Forget one slot's jig captures so the plan asks for it again: `<slot>k` (calibration only) |
| `l` | LED test menu: 1 walk, 2-4 channels on LED 0-2, 5-9 animations, 0 all, `q` quits (5V rail is switched for the menu) |
| `p` | Scan the 20 slots, build the production payload and send it to the car; the LEDs show the verdict (green ACK, amber empty, red refused, RF error no answer) |
| `e` | Listen to `_box_`: prints the car's telemetry and log fragments until a key stops it |
| `t` | Radio menu: `1` probes the chip over SPI, `2` prints the addresses and config, `3` sends a one command test program, `q` quits |
| `dfu` | Type the three letters in a row to jump to the STM32 DFU bootloader |
| `reset` | Type the five letters in a row to reboot the MCU |
| `b` | Deep sleep (STOP): the chip sleeps until the wake button (PB9) is pressed; the USB port drops and comes back after the wake |

Serial is USB CDC at 115200 baud. `delay` calls are time-sliced so typed keys always get through.

Typed keys are echoed back so you always see what the board received: `[SP]` marks SPACE, and a new line starts when the board consumes the Enter. The monitor stop echoes its stop key and annotates the burst tail it drains as `[dropped N]`.

## The Pieces

Four played pieces plus the empty board: FORWARD (green, 4 mm N),
TURN_RIGHT (yellow, 1 mm N), TURN_LEFT (blue, 4 mm S), LOOP_CALL (red,
1 mm S). Each carries a magnet glued inside a hole so the piece can be
placed upside down; the slot hall sensor reads a distinct window per pole
and per height, which is what the threshold table maps to raw ranges.
All printed values are raw ADS1115 counts (GAIN_ONE, single-ended, 3V3
rail) — the same scale the production decode compares against, so pasted
thresholds need no conversion.

Capture runs through the jig mode below; the guided game walk and the manual
per-key labeling it replaced are gone, and `j` reprints the analysis and the
rows at any time.

## Slot Monitor (`m`)

Continuous readout of one slot while you work it, ~5 readings per second:

```text
5m          # monitor slot 5 (the slot number is required)
   [ADS module 2/5: I2C1 0x49, slot 5 = channel 1/4]
   slot 5: raw=14025 min=14019 max=14027  TURN_RIGHT
   slot 5: raw=14026 min=14019 max=14027  TURN_RIGHT
   slot 5: raw=14024 min=14019 max=14027  TURN_RIGHT
   slot 5: raw=14025 min=14019 max=14027  TURN_RIGHT
   [ADS module 2/5: I2C1 0x49, slot 5 = channel 1/4]
   slot 5: raw=14025 min=14019 max=14027  TURN_RIGHT
   ...
<any key>   # stop, prints a summary (n, mean, min, max, span)
```

The readouts are grouped by four — one header per group reminds which ADS module and channel the slot sits on. Each line ends with the piece this slot's **compiled row** decodes right now (the same table the scan and the future production decode use, so a zeroed or stale row honestly shows `INVALID`/the wrong label — it reflects the firmware, not the RAM clusters). The running `min`/`max` are the point of the tool:

- **Clamp check**: a max pinned near the top of the scale (~26400 on the 3V3 rail) means the 49E hall output is saturated — the piece is too strong or too close, not "reading high". Fix the physics (more air gap, glue spacer under the magnet), not the code
- **Variance check**: span across a settled reading set shows the class spread; keep it well below the distance to the next class's range when pasting thresholds
- Readouts sit inside the calibration mode: any typed key stops the monitor before it can consume a command
- **Stop drains the burst**: the key that stops the monitor also drops the rest of the serial burst typed while it ran, so a command typed mid-monitor must be retyped once the monitor has unwound
- **Self-recovery**: a failed read re-inits the module (I2C1 bus restart + begin, I2C3 bit-bang re-init) and retries the read once — the `[RECOVERED]`/`[WARN]` line tells which bus healed

## Slot Frozen Monitor (`x`)

One clean, self-contained measurement per keystroke: the monitor waits
**without sampling**, and a SPACE opens a fixed 3 s window whose stats print
as a single line. Made for orientation sweeps, where rotating the piece
between two measures would poison a running min/max.

```text
5x          # frozen monitor on slot 5 (the slot number is required)
            # waits silently: nothing is read or accumulated
<SPACE>     # measurement window: continuous reads for 3 s, then one line
   slot 5: n=9 mean=11342 min=11336 max=11358 span=22  TURN_LEFT
<SPACE>     # next window, as many as needed
<any key>   # quit (drains the typed burst like the live monitor)
```

The windows never accumulate: each line is one measurement (n, mean, min,
max, span, plus the label the slot's compiled row decodes), so a whole sweep
can be copied from the log at once. Placing or rotating the piece between
windows cannot pollute any stat. A key typed mid-window ends the window
early and still prints the partial line; a SPACE typed mid-window is
swallowed (one key, one measure).

## Step-by-Step Calibration

Full campaign recipe, in order:

1. Flash the firmware, open the serial monitor, press `c` — calibration mode enables capture, and every reading raises the 3V3 sensor rail for its own duration
2. Run the jig walk (next section): `j`, then follow the prompts until the firmware prints the analysis, the verification sheet and the generated rows
3. Paste the generated rows over the `BEGIN/END GENERATED PER-SLOT ROWS` block of `soft/emu-pc/targets/gbox/include/piece_thresholds.h`, rebuild and flash
4. Verify: `s` runs the scan pass — every calibrated slot must decode its placed piece; watch real re-seating variance with `<slot>m` before trusting a boundary

Each scan now ends with a `--- PASS COMPARISON ---` block: for every slot it
prints the previous pass's mean, this pass's mean and their delta, so a piece
that was placed, removed or nudged between two passes is obvious instead of
hiding inside the session-wide `min`/`max`. Deltas past 150 counts are flagged
`<<`. The first scan of a session only stores the baseline; a failed read keeps
the slot's last valid reading as the reference. The running `min`/`max` on the
slot lines still span the whole session, so a piece that moves between passes
inflates their `span` — use the comparison block, not that span, to judge
stability.

A slot that needs rework later: `19k` forgets its captures, and the next `j`
asks for it again — ten seconds instead of a campaign.

## Jig Mode (Piece Census and Slot Gains)

The measured bench path for a full table regeneration. Enter it in calibration mode with `j`. It replaces the game's slot-major walk with a type-major one: one piece stays in the operator's hand for a whole pass, and **no keypress is needed during the walk** — the reading says when a piece is in and when it has settled.

### Two Quantities, Not One

A piece is characterised by its **deviation** from the slot's idle level, and a slot by the **gain** that scales every deviation. Everything else follows from those two.

Working in deviations is what makes the numbers usable: the rail wanders by about 0.2 % over minutes, and the scan fought that wander for a whole evening. A deviation is taken against an idle level measured seconds earlier in the same slot, so a shift of the supply moves both terms and cancels. That is also why the warm-up rule of the scan does not apply here: nothing is compared across minutes except ratios.

The exception is a census spread over several minutes, where an early capture taken against a much older idle would be biased — the reference slot's idle is therefore re-measured at the start of each type, and the gain pass re-measures it in every slot, the reference one included, so both ends of every ratio come from the same pass.

### Phases

1. **Idle** of the reference slot (slot 1): the datum every deviation is taken against
2. **Census**: five pieces per type, four orientations per piece while they matter, all in slot 1
3. **Gains**: the same reference piece in all 20 slots, each preceded by its own idle — the reference slot included, so both ends of every ratio come from the same pass. Which piece it was is read back from the captures, not from the census, so a census redone later does not orphan a gain pass that is still valid
4. **Analysis**: the type medians and spreads, the slot gains and margins, and the pasteable rows

The plan is recomputed from what is missing at every step, so a capture can simply be repeated by replacing the piece: the newest one wins. Orientation is only asked four times while it moves the reading by more than 50 counts; after three pieces the firmware drops to one placement per piece and says so.

A piece placed badly is **restarted** with `n`: its poses are dropped and the walk asks for it again from orientation 1, so a mistake costs four placements instead of the whole census. It acts on the piece just measured, not on the one on screen — by the time a mistake is noticed the prompt has already moved on, and that next piece is not the one anybody wants to redo. **`b`** steps back a single capture for the same reason, when only the last orientation was captured badly. A broken or lost piece is **written off** with `N`: all four orientations are dropped from the plan, and a type is treated as complete from **three pieces up**, which is still enough for a median and a spread.

### Detector

A reading 200 counts away from the slot's idle or more is a piece; six consecutive readings inside a 25-count span are a settled plateau and the capture happens. A wild first reading widens the span and resets the plateau on its own, so no explicit discard is needed.

A new capture in the same slot also needs the reading to have **moved by 30 counts since the previous one**. That is what stops a placement that has not moved from being recorded twice, and it replaces a first design that armed on a return to idle: a reading costs the rail's settle time, so a piece lifted and turned in a third of a second was invisible to it and the walk stalled until SPACE was pressed.

30 counts is only just above the jitter two identical placements show (0 to 15). A first value of 150 was wrong in a way worth recording: it treated "the piece did not move" and "the piece turned but the reading moved by less than 150" as the same thing, and a turn that shifts the reading by 48 counts is a real turn. When a piece genuinely reads the same in every orientation, the guard gives way after eight polls, records the value and says only that — the firmware has no way to know the magnet is centred, so it no longer claims it. The census prints each piece's **rotation spread**, which is the measurement that answers the question.

An idle level is refused when it reads more than 900 counts from the expected one, which is how a piece left in the slot is kept from becoming that slot's reference. SPACE lifts both guards but never the stability requirement: a forced capture is still a settled plateau, never a single reading.

### Marking the Pieces

The census measures four orientations per piece and the analysis keeps the one that lands closest to its family's centre, because an off-centre magnet can move the reading by a thousand counts a quarter turn apart. What it prints has to be something the operator can act on, so the winning pose is named together with the edge to paint:

```
    piece 1 dev -2459 rotation spread 1125 mark: text upside down, paint the edge at the bottom of the text
```

| pose that won | edge to paint |
|---|---|
| text readable | the edge at the top of the text |
| text 90 cw | the edge at the left of the text |
| text upside down | the edge at the bottom of the text |
| text 270 cw | the edge at the right of the text |

The rule given to the player should then be absolute, not relative: **the arrow points to the hinge**. "Pointing up" depends on where the player stands, while the hinge is a fixed feature of the box that anyone can check at a glance.

### Colours, Not Names

The operator handles the pieces by colour, so the whole walk speaks in colours: the prompts, the dump and the analysis all start with `green 4mm`, `blue 4mm`, `red 2mm` or `yellow 2mm`. The name a piece carries in the shared table (`FORWARD`, `TURN_LEFT`, ...) is the bench firmware's business, and it is only printed where it matters — as `green 4mm as TURN_LEFT` — so nobody has to relabel a piece.

That mapping lives in **one array**, `kBandPieces` in `scan.cpp`, and `kJigTypeHints` holds the matching colours. The bands ascend with the raw reading, and this batch reads the greens lowest and the blues highest, so the band order here is:

```
band 0  green 4mm   FORWARD      the most negative
band 1  yellow 2mm  TURN_RIGHT
band 2  -           EMPTY        the slot's own idle
band 3  red 2mm     LOOP_CALL
band 4  blue 4mm    TURN_LEFT    the most positive
```

The names follow the measurement instead of asking twenty magnets to be flipped. Rebuilding the pieces the other way round flips that one array, nothing else — and nothing has to be relabelled on the pieces either way.

### Canonical Order Check

The bands ascend with the raw reading, so the table's columns follow one order and the decode depends on it: `FORWARD < TURN_RIGHT < EMPTY < LOOP_CALL < TURN_LEFT` for this batch. A piece built with the other pole lands in the wrong band, and then every slot would be reported as an overlap without ever saying why. Before emitting anything, the analysis prints the measured order next to the expected one and refuses to produce rows when a family is out of place — the fix is the `kBandPieces` mapping, not a threshold.

### Threshold Placement

Each row is seeded from the compiled `GASPETTO_ADC_SLOT_THRESHOLDS`, so a slot the run could not trust keeps the row it already had instead of dropping to zeros. For the slots it could compute, every boundary sits **at the midpoint of the gap** between the two measured bands, rounded to 5:

```
band of a class = slot idle + median × gain  ±  (spread + play allowance) × gain
boundary between two classes = (upper of the lower + lower of the upper) / 2
```

The midpoint is the fix for what the game does (`min − 150`, which leaves 150 counts on one side and the whole rest of the gap on the other): it splits the risk evenly, so a badly placed piece and an empty slot are equally far from a wrong verdict.

The play allowance covers the fact that a piece lands anywhere within the slot's play, and the field error that follows is proportional to the deviation — the field falls with the cube of the distance, so this is not a fixed count. Keeping it out would make the bands optimistic: the census picks each piece's best of four placements, and a player who does not reproduce that lucky landing would fall outside a band sized without it. `kJigPlayPermille` holds the value, derived from the 588-count spread measured over four orientations on a piece reading about 2700, and meant to be replaced by the 8-placement test.

Two bands that touch mean the slot cannot separate the pieces — the row is refused and reported.

### Idle History

The analysis opens with every idle measurement grouped by slot, each shown as its distance to that slot's median:

```
--- IDLE HISTORY (counts, deltas to the slot median) ---
  slot  1  median 13110  -15, -15, +26, +26, 0
  slot 12  median 12430  -160, +160 <<
```

Two measurements of one slot minutes apart have to agree to a few tens of counts: the rail wanders by that much, not by hundreds. A larger spread means one of them caught a piece lying next to the slot or a hand over it, and every deviation of that family is shifted by the same amount. It is reported, not corrected — dropping the family costs twenty poses, and re-basing it is a judgement call the operator should make with the numbers in front of them.

### Verification Sheet

After the rows, a sheet lists **only the slots that need bench work**, with the absolute counts a well placed sensor would read:

```
--- VERIFICATION SHEET (targets at the median gain) ---
   median gain 1.02
  slot 19  gain 0.38  reads low: bring the sensor closer
            idle 12430 | FORWARD 15440 +-430 | TURN_RIGHT 13720 +-200 | EMPTY 12430 +-150 | LOOP_CALL 11870 +-180 | TURN_LEFT 9820 +-400
```

The targets use the **median gain**, not the slot's own. With its own gain the sheet would always agree with what is read, and moving the sensor would have no visible goal; with the median it names the reading to aim for, so the loop is: read with `<slot>x`, heat and move, read again, stop when close. A slot is flagged when two bands touch, when the smallest gap falls under 300 counts, or when its gain is more than 30 % off the median — in either direction, since too high brings the strongest piece near the sensor's 26400-count ceiling.

### Keys During Jig Mode

| Key | Action |
|-----|--------|
| `SPACE` | Force a capture: lifts the presence and re-arm guards, a settled plateau is still required |
| `d` | Dump every capture as text, one line each |
| `n` | Restart the piece **just measured**: its poses are dropped and it is asked again from orientation 1 |
| `b` | Undo the last capture; its pose comes back on screen |
| `N` | Write off the piece on screen (broken or lost): its four orientations leave the plan |
| `q`, `x` or `j` | Stop the walk, captures kept in RAM |
| `X` | Clear all captures and stop |
| `19k` | Forget slot 19 so the plan asks for it again after a sensor is moved; type it from the shell, not while the walk is running |
| `?` | Show the jig keys |

`printJigHelp()` is the single source of that list: the shell's `?` and the jig's own `?` both call it, so a new key cannot appear in one and not the other.

Captures live in RAM only: a reset loses the session. A full walk is 60 to 120 placements, so it is meant to be done in one sitting.

## Send Pieces (`p`)

The radio link check: one key runs the whole chain production GBox runs when
it validates a program, without running GBox itself.

1. **Scan** — one mean reading per slot, decoded by `pieceForRaw`, the
   single threshold walk every consumer of this firmware uses (monitors,
   scan and send cannot disagree). The walk **stops at the first `EMPTY`**:
   the pieces are a contiguous run, so an empty slot ends the sequence and
   whatever lies after it stays out of the program — what was already read
   is kept. The loop zone (slots 13-20) is read **only when the main zone
   held a `LOOP_CALL`**, since without one those slots never reach the
   program.
2. **Build** — `buildProgramFromPieces` from `shared_types/PieceProgram`
   (symlinked here), the very function the production GBox links
3. **Send** — the resulting 32-byte `CommandPacket` on the `_car_` pipe

The chip is set up exactly like production `RadioController::begin()`: PA
level and data rate from `config_radio.h` (1 MBps), 5-byte address, 32-byte
payload, writing pipe `_car_`. The bench only transmits, so `write()`
returning **true** means the car's radio answered the auto-ack — that is the
delivery proof.

```text
c                    # calibration mode (each read raises the 3V3 sensor rail)
p                    # scan + build + transmit
--- SEND PIECES: scan -> build -> radio ---
   slot 1: raw=14896  FORWARD
   slot 2: raw=13472  EMPTY
[send] slot 2 empty: sequence ends here
[send] no LOOP_CALL in the main zone: loop zone not scanned
[send] program: 1 command(s) [13]
[send] radio.write 32 bytes -> ACK (car answered)
```

The slot lines are the decode itself, so they are the first place to look
when a run is refused: a slot reading `INVALID` (failed read, uncalibrated
row or a value outside every band) **aborts the scan where it stands** and
raises the build-error LEDs, instead of letting a wrong board travel on. A
payload over 31 commands is refused by `buildProgramFromPieces` the same
way. The car can never receive more than it can run.

The verdict also shows on the LEDs (the 5V rail is raised for the animation
and put back after, like the LED menu):

| Feedback | Meaning |
|----------|---------|
| green cascade | the car acknowledged the payload |
| amber blink | empty board, nothing to send |
| red blink | build refused (invalid slot or overflow), nothing transmitted |
| RF error | transmitted but no ACK: the car is absent or not listening |

## Car Stream (`e`)

The other half of the link: `setup()` already gives the bench the real box
layout — pipe 0 for the program bursts to `_car_`, pipe 1 listening on
`_box_` — and this key opens the listening, where the car answers with
telemetry and log fragments.

```text
e                    # any key stops; the radio goes back to transmit only
   listening pipe1 = 5F 62 6F 78 5F
   [TLM] yaw=0.0 tgt=7.0 state=0 speed=50
   [LOG] car-sim [imu] tick=42 ms=8100 yaw=182.0 speed=42
   [stream] quiet, packets=17
--- CAR STREAM stopped, packets=17 ---
```

The session opens with the address it listens on, so it can be read
against the sender's without opening either firmware's menu, and it
refuses to start when `radio.begin()` failed — a dead radio and a silent
car otherwise both look like `quiet, packets=0`.

Log lines are reassembled before printing: the sender splits them in 30
byte pieces and telemetry can land in between. Outside a session nothing
drains the FIFO, so the car's writes report FAILED unless a session (or a
real box) is listening — that difference is the visible proof that the
receiver is there.

## Radio Menu (`t`)

The link diagnostics that `printDetails()` used to attempt: RF24 writes
that dump with `printf_P`, which no core here wires to the port, so it
reaches nothing. This menu prints the same facts with `Serial.print`.

```text
t                    # q quits
--- RADIO MENU ---
   [radio] 1 probe | 2 addresses and config | 3 send test program | q quit
2
   tx       = 5F 63 61 72 5F
   rx pipe1 = 5F 62 6F 78 5F
   pa=LOW rate=1MBPS channel=76 payload=32
3
[radio] test program (MOTOR_STOP) -> ACK (car answered)
```

`1` reads the chip over SPI at that moment, so it says whether the radio
answers *now* rather than repeating the flag `setup()` left. `2` shows
what the firmware configured: RF24 v1.6 has no address getter, and the
configured constants are the thing to compare when two rigs do not hear
each other. `3` sends a single `MOTOR_STOP`, which proves the transmit
path without a scanned board and is harmless whatever the car is doing.

## Hardware Setup

- **Board**: STM32F411CE BlackPill
- **Slots**: 20 piece slots, each with a positive hall sensor wired to one of 20 ADC channels (five ADS1115 modules × 4 channels)
- **I2C buses**: four ADS1115 on hardware I2C1 (PB6/PB7), one on bit-banged I2C3 (PB4/PA8) — the I2C3 bus needs bit-bang recovery when it hangs
- **Slot wiring**: slots 1-4 on the 0x4B module (ADDR→SCL), 5-8 on 0x48 (ADDR→GND), 9-12 on 0x4A (ADDR→SDA), 13-16 on 0x49 (ADDR→VCC), 17-20 on I2C3 0x4A (ADDR→SDA)
- **LED**: 3-APA106 chain (data on PB8), powered from the 5V rail
- **Rails**: 5V LED rail (PB14) and 3V3 sensor rail (PB15), each driven through an open-drain MOSFET gate (LOW = ON)
- **Radio**: NRF24L01 as in production
- **Board LED** (PC13): heartbeat while idle

## Building and Flashing

```bash
cd soft/pio/arduino_box_hw_test

# Typing dfu (the three letters in a row) in the serial monitor jumps to
# the ROM DFU bootloader: the board re-enumerates as "STM32 BOOTLOADER"
# (0483:df11), no button needed. The serial link dies with the jump; a
# power cycle returns to this firmware if you change your mind.
pio run -e arduino_box_hw_test -t upload

# Fallback, or cold board: hold BOOT0 while pressing RESET, then upload

# Wait ~8 seconds for serial port to stabilize after DFU exit
pio device monitor -b 115200
```

## Expected Output

At boot — the firmware ends on a waiting hint (the cycle no longer starts by itself):

```
--- HARDWARE TEST STARTED ---
1. MOSFETs initialized and TURNED OFF.
   5V LED rail ON (settled) before APA106 init
APA106 init: 3 pixel(s) on PB8, first frame sent (all dark)
...
   waiting for a key ('?' for help)
```

Then on `s` (the LED menu and animations can run in between with `l`):

```
[health] cycle start ('c' = calibration, '?' = status)
-> TURNING ON 5V_SWITCHED (LED Cache)...
...
--- ADS1115 TEST DONE ---
Cycle complete. 's' reruns the test.
```

After the health cycle, the firmware idles silently and only wakes to report probe changes or USB keys — `'s'` re-runs the test at will, `'l'` opens the LED test menu, whose `0` runs the power-on/power-off animations the health cycle used to play.
