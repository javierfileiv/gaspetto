# Box HW Test

Hardware validation and piece calibration firmware for the Gaspetto GBox programming board (STM32F411CE BlackPill).

## Purpose

The GBox programming board reads 20 piece slots through five ADS1115 ADCs and lights them with the NeoPixel ring. This firmware runs two runtime test modes over the same build:

1. **Health mode** — power rails, LED strip, five ADS1115 probes and the NRF24 radio are exercised once at boot, then each probe change is reported as it happens
2. **Calibration mode** — a capture of how the positive hall sensor reads each actual piece, used to derive the shared piece threshold table

The production and test firmware must agree on thresholds, so both read the same header `soft/emu-pc/targets/gbox/include/piece_thresholds.h` (symlinked into this project's `include/`).

## Runtime Modes

- **Health** (default at boot) — runs one test cycle, then sits quiet with the board LED heartbeat; re-running it (`s`) re-tests everything and reports probe/radio changes against the previous cycle
- **Calibration** — keeps the 5V LED rail off, powers the 3V3 sensor rail, and allows piece labeling

## USB Commands

| Key | Action |
|-----|--------|
| `h` | Health mode |
| `c` | Calibration mode |
| `s` | Run the active mode test once |
| `?` | Print the active mode and rail states |
| `t` | Piece threshold gap analysis (any mode) |
| `x` | Emit the paste-ready `piece_thresholds.h` block |
| `m` | Continuous readout of a slot (`<slot>m` or `m` for the last addressed) |

Serial is USB CDC at 115200 baud. `delay` calls are time-sliced so typed keys always get through.

## Piece Labeling (Calibration Mode)

Label a slot by placing the piece and typing `<slot><label>`, e.g. `5f`.

Label keys:

- `e` EMPTY piece slot
- `f` FORWARD piece
- `b` BACKWARD piece
- `r` TURN_RIGHT piece
- `l` TURN_LEFT piece
- `L` LOOP_CALL piece

Pieces are detected by a magnet glued inside a hole of the piece — it cannot be placed upside down. Two or three magnet sizes exist; the slot hall sensor reads a distinct voltage window per piece, which is what the threshold table maps to raw ranges.

The firmware takes one reading of the labeled slot with a 15-sample mean, then prints the raw value and count. Clusters accumulate per slot and per piece type; the same piece may be labeled in several slots to cover different sensors.

- After a first label, a bare label key (e.g. `f`) addresses the previously tagged slot — convenient when re-measuring the same piece several times
- The first label of a piece type in a slot is always accepted: the outlier guard compares the fresh reading against that piece type's own cluster only, so swapping pieces in a slot never bounces
- **Outlier guard**: if a re-label of the same piece type strays from `min-20% / max+20%` of the already collected cluster, it is refused with `[OUTLIER IGNORED]` — with the orientation fixed by the glued magnet, a rejection almost always means a badly seated piece or another stray reading
- **Reset**: `<slot>R` (e.g. `5R`) clears all clusters of that slot; bare `R` clears the previously addressed one. Re-label afterwards to rebuild with clean samples

The capture happens on the label key itself — `s` is only a manual rerun of the active mode test.

## Typical Labeling Workflow

One pass over the real pieces, fewest keystrokes:

```text
c            # switch to calibration mode (rails make the pieces readable)

# put the FORWARD piece in slot 1, then type:
1f           # = slot 1, label FORWARD; the firmware reads and stores it

# move the FORWARD piece to slot 5 (different sensor), then type:
5f

# place the piece again in slot 5, for a second measurement, then type:
f            # bare key: re-tags slot 5, sees n=2

# put the BACKWARD piece instead, then type:
5b           # slot 5 gets a BACKWARD cluster; FORWARD stays intact

# piece badly seated (or a hand over the sensor); the firmware refuses it:
[OUTLIER IGNORED] slot 5 BACKWARD raw=4500 expected.raw 2400-3600
5R           # clear slot 5 samples (all labels), then re-measure 5b and 5f

# done over the whole board; check the analysis, then the header:
t            # ranges, ambiguity checks, suggested midpoints
x            # paste-ready #define block for piece_thresholds.h

# something needs rework later; samples survive the modes, then:
R            # targets the last tagged slot, re-tag it right after
```

Practical hints:

- Cover the meaningful extremes first (EMPTY and LOOP_CALL), then the movement pieces between them — overlaps show up on `t` even with partial coverage
- A `[AMBIGUOUS]` line usually means too few samples: add more `'slot<label>'` points for the reported pieces and run `t` again
- All printed values are raw ADS1115 counts (GAIN_ONE, single-ended, 3V3 rail) — the same scale the production decode compares against, so threshold pastes need no conversion
- Samples survive mode switches — entering calibration only resets the per-slot scan statistics, never the labeled clusters; a power cycle or an explicit `R` are the only ways to clear them

## Slot Monitor (`m`)

Continuous readout of one slot while you work it, ~5 readings per second:

```text
5m          # monitor slot 5 (or m alone: last addressed slot)
   [ADS module 2/5: I2C1 0x49, slot 5 = channel 1/4]
   slot 5: raw=14025 min=14019 max=14027
   slot 5: raw=14026 min=14019 max=14027
   slot 5: raw=14024 min=14019 max=14027
   slot 5: raw=14025 min=14019 max=14027
   [ADS module 2/5: I2C1 0x49, slot 5 = channel 1/4]
   slot 5: raw=14025 min=14019 max=14027
   ...
<any key>   # stop, prints a summary (n, mean, min, max, span)
```

The readouts are grouped by four — one header per group reminds which ADS module and channel the slot sits on. The running `min`/`max` are the point of the tool:

- **Clamp check**: a max pinned near the top of the scale (~26400 on the 3V3 rail) means the 49E hall output is saturated — the piece is too strong or too close, not "reading high". Fix the physics (more air gap, glue spacer under the magnet), not the code
- **Variance check**: span across a settled reading set shows the class spread; keep it well below the distance to the next class's range when pasting thresholds
- Readouts sit inside the calibration mode so the labels/parser stay consistent — a stray label key cannot run mid-monitor
- **Self-recovery**: a failed read re-inits the module (I2C1 bus restart + begin, I2C3 bit-bang re-init) and retries the read once — the `[RECOVERED]`/`[WARN]` line tells which bus healed

## Gap Analysis (`t`)

Aggregates every cluster by piece type across all 20 slots, prints each piece range from lowest raw mean to highest, then checks for overlaps:

- Overlapping piece ranges are flagged `[AMBIGUOUS]` — separate them before trusting thresholds
- Non-overlapping neighbours suggest the midpoint as a ready-to-paste `#define GASPETTO_ADC_THRESHOLD_<PIECE>_START`
- The top of the highest labeled piece informs `GASPETTO_ADC_THRESHOLD_LOOP_END`

## Threshold Header (`x`)

Prints a complete `#pragma once` block for `soft/emu-pc/targets/gbox/include/piece_thresholds.h`, directly from the labeled samples:

- The gap between two adjacent labeled piece types becomes the upper piece's `*_START`, placed at the raw midpoint — the same scale the production decode compares against
- Each define carries a comment with the raw gap bounds and the sample count that produced it
- Overlapping ranges cannot drive a define — the block prints `/* AMBIGUOUS: cannot derive ... */` at that spot instead; fix the ambiguity and re-run
- `LOOP_END` sits just above the top of the highest labeled piece; label a LOOP_CALL piece so its own range drives it

Paste the block over the defines of `piece_thresholds.h` — the `static_assert`s at the bottom of that header re-validate strict ordering at build time.

## Hardware Setup

- **Board**: STM32F411CE BlackPill
- **Slots**: 20 piece slots, each with a positive hall sensor wired to one of 20 ADC channels (five ADS1115 modules × 4 channels)
- **I2C buses**: four ADS1115 on hardware I2C1 (PB6/PB7) at 0x48/0x49/0x4A/0x4B (ADDR pin to GND/VCC/SDA/SCL), one on bit-banged I2C3 (PB4/PA8) at 0x4A (ADDR pin to SDA) — the I2C3 bus needs bit-bang recovery when it hangs
- **LED**: NeoPixel strip (PC14), powered from the 5V rail
- **Rails**: 5V LED rail (PB14) and 3V3 sensor rail (PB15), each driven through an open-drain MOSFET gate (LOW = ON)
- **Radio**: NRF24L01 as in production
- **Board LED** (PC13): heartbeat while idle

## Building and Flashing

```bash
cd soft/pio/arduino_box_hw_test

# Put board in DFU mode (hold BOOT0 while pressing RESET)
pio run -e arduino_box_hw_test -t upload

# Wait ~8 seconds for serial port to stabilize after DFU exit
pio device monitor -b 115200
```

## Expected Output

```
--- BOX HW TEST ---
mode: health
--- POWER RAILS TEST CYCLE ---
   5V rail OFF, 3V3 OFF: LEDs must be dark
   [OK]  5V rail must be OFF, LEDs dark
   [OK]  3V3 rail must be OFF, LEDs dark
   ...
   slot 1: raw=3121 n=2
```

After the health cycle, the firmware idles silently and only wakes to report probe changes or USB keys — `'s'` re-runs the test at will.
