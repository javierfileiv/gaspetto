# Box HW Test

Hardware validation and piece calibration firmware for the Gaspetto GBox programming board (STM32F411CE BlackPill).

## Purpose

The GBox programming board reads 20 piece slots through five ADS1115 ADCs and lights them with the NeoPixel ring. This firmware runs two runtime test modes over the same build:

1. **Health mode** — power rails, LED strip, five ADS1115 probes and the NRF24 radio are exercised once at boot, then each probe change is reported as it happens
2. **Calibration mode** — a capture of how the positive hall sensor reads each actual piece, used to derive the per-slot threshold table

The production and test firmware must agree on thresholds, so both read the same header `soft/emu-pc/targets/gbox/include/piece_thresholds.h` (symlinked into this project's `include/`). Both tables live in that header: production `GBox.cpp` still decodes with the legacy global defines, while this firmware decodes with the per-slot rows. Porting production to the per-slot scan and dropping the legacy defines is a planned separate step.

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
| `t` | Piece threshold gap analysis — global legacy view (any mode) |
| `x` | Emit the paste-ready legacy global `#define` block for `piece_thresholds.h` |
| `m` | Continuous readout of a slot (`<slot>m`; bare `m` reuses the last addressed slot) |
| `g` | Start game mode (calibration mode only) |
| `d` | Dump all collected piece clusters as text (restore them after a reflash with `i`) |
| `i` | Import clusters from a paste of the `d` output — clears the table first |
| `dfu` | Type the three letters in a row to jump to the STM32 DFU bootloader |

Serial is USB CDC at 115200 baud. `delay` calls are time-sliced so typed keys always get through.

Typed keys are echoed back so you always see what the board received: `[SP]` marks SPACE, and a new line starts when the board consumes the Enter. The monitor stop echoes its stop key and annotates the burst tail it drains as `[dropped N]`; a cluster import (`i`) echoes each accepted line with a `<<<` prefix instead of echoing the paste character by character.

## Piece Labeling (Calibration Mode)

Label a slot by placing the piece and typing `<slot><label>`, e.g. `5f`.

Label keys:

- `e` EMPTY piece slot
- `f` FORWARD piece
- `b` BACKWARD piece
- `r` TURN_RIGHT piece
- `l` TURN_LEFT piece
- `L` LOOP_CALL piece

Pieces are detected by a magnet glued inside a hole of the piece so it can be placed upside down. Two or three magnet sizes exist; the slot hall sensor reads a distinct voltage window per piece, which is what the threshold table maps to raw ranges.

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
x            # paste-ready legacy global #define block for piece_thresholds.h

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
5m          # monitor slot 5 (bare m: last addressed slot; on a cold board
            # it has none and answers with the type '<slot>m' hint)
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
- **Stop drains the burst**: the key that stops the monitor also drops the rest of the serial burst typed while it ran, so a command typed mid-monitor must be retyped once the monitor has unwound
- **Self-recovery**: a failed read re-inits the module (I2C1 bus restart + begin, I2C3 bit-bang re-init) and retries the read once — the `[RECOVERED]`/`[WARN]` line tells which bus healed

## Gap Analysis (`t`)

Aggregates every cluster by piece type across all 20 slots, prints each piece range from lowest raw mean to highest, then checks for overlaps:

- Overlapping piece ranges are flagged `[AMBIGUOUS]` — separate them before trusting thresholds
- Non-overlapping neighbours suggest the midpoint as a ready-to-paste `#define GASPETTO_ADC_THRESHOLD_<PIECE>_START`
- The top of the highest labeled piece informs `GASPETTO_ADC_THRESHOLD_LOOP_END`
- This is the legacy global view: it drives the shared `#define`s only. Per-slot rows (what this firmware decodes with) come from the game campaign below — on a board whose sensors idle up to 940 counts apart, no single global threshold separates the classes

## Threshold Header (`x`)

Prints a complete `#pragma once` block for `soft/emu-pc/targets/gbox/include/piece_thresholds.h`, directly from the labeled samples:

- The gap between two adjacent labeled piece types becomes the upper piece's `*_START`, placed at the raw midpoint — the same scale the production decode compares against
- Each define carries a comment with the raw gap bounds and the sample count that produced it
- Overlapping ranges cannot drive a define — the block prints `/* AMBIGUOUS: cannot derive ... */` at that spot instead; fix the ambiguity and re-run
- `LOOP_END` sits just above the top of the highest labeled piece; label a LOOP_CALL piece so its own range drives it

Paste the block over the defines of `piece_thresholds.h` — the `static_assert`s at the bottom of that header re-validate strict ordering at build time. These defines drive the legacy production decode; this firmware's own scan decodes with the per-slot rows instead.

## Step-by-Step Calibration (Classic)

Full campaign recipe, in order:

1. Flash the firmware, open the serial monitor, press `c` — calibration mode powers the 3V3 sensor rail and enables capture
2. Run the guided game (next section): `g`, then follow every prompt until the firmware prints the per-slot analysis and the generated rows
3. Paste the generated rows over the `BEGIN/END GENERATED PER-SLOT ROWS` block of `soft/emu-pc/targets/gbox/include/piece_thresholds.h`, rebuild and flash
4. Verify: `s` runs the scan pass — every calibrated slot must decode its placed piece; watch real re-seating variance with `<slot>m` before trusting a boundary
5. A reflash wipes the clusters (RAM only): `d` before flashing, `i` and paste after to restore them

The classic manual path (`<slot><label>` + `t` + `x`) still exists and is useful for spot checks: repeated bare label keys thicken the per-slot clusters, and `t`/`x` summarize them globally. It does not produce the per-slot rows, and the legacy defines it feeds are only what production decodes with until production adopts the per-slot scan.

## Cluster Persistence (`d`/`i`)

The labeled clusters live in RAM and die with every reflash. To survive one:

```text
d            # dump: one line per (slot, piece) as
             #   <slot> <piece> <min> <max> <count> <sum>
             # followed by a line holding only .
i            # start import: clears the table, then consumes every typed
             # character as raw text
<paste the dump exactly as printed, ending with its lone .>
             # import stops, prints "restored N"; a malformed line is
             # reported as [IMPORT] skipped and ignored
```

Paste the `d` output into the terminal while `i` is armed — the board cannot remember its own campaign otherwise, and a partially restored table silently degrades the analysis.

## Game Mode (Guided Calibration)

Game mode replaces manual `<slot><label>` typing with a guided sequence that prevents mislabeling and enforces reading stability. Activate it in calibration mode with `g`.

### How It Works

The firmware walks through all (piece, slot) combinations in a fixed order:

1. **By piece**: EMPTY → FORWARD → BACKWARD → TURN_RIGHT → TURN_LEFT (and LOOP_CALL when the orange piece exists)
2. **By slot**: for each piece, all slots 1–12 — the soldered main zone (`kGameSlotCount` extends to 20 with the loop zone)
3. **By sample**: 8 readings per combination (configurable via `kGameSamplesPerCombo`)

Total: 5 pieces × 12 slots × 8 samples = **480 steps** (576 once LOOP_CALL is added). EMPTY is special: one SPACE confirms the whole board is clear and records every slot at once.

### Sequence

At each step, the firmware:

1. Displays the instruction: `[step/total] slot N, PIECE_HINT [sample/8]`
2. In auto-capture (default, toggle with `a`), polls the current slot and records a sample as soon as the reading settles — the operator only places and removes pieces
3. With auto-capture off, SPACE triggers the capture attempt
4. Either way: 5 rapid readings, stability check (span < 100 counts), then the mean is recorded with the outlier guard (±20%) and the sequence advances
5. If unstable: prints the span and stays on the same step — adjust the piece (or SPACE again with auto off)

### Keys During Game Mode

| Key | Action |
|-----|--------|
| `SPACE` | Record one sample (auto-capture off) or confirm the EMPTY board sweep |
| `a` | Toggle auto-capture (on by default: readings record themselves when they settle) |
| `r` | Reset the current (slot, piece) cluster and retry the step from scratch |
| `q` | Quit game mode and generate thresholds from collected data |
| `?` | Show game mode commands and current instruction |

### Threshold Generation

When the game completes all steps (or you press `q`), the firmware analyzes **each slot's own clusters** — sensors idle up to 940 counts apart across the board, so one slot's clean gap can be the next slot's overlap — and prints one verdict per slot under `--- PER-SLOT THRESHOLD ANALYSIS ---`:

- `[OK] piece[min..max] ... gaps a,b,c,d` — the five classes separated on that sensor; thresholds sit at the midpoints and the printed gaps are the margins
- `[OVERLAP A/B gap=N, row zeroed]` — two classes overlap on this slot; its row decodes INVALID
- `[NO DATA]` / `[INCOMPLETE, row zeroed]` — this slot was not fully recorded
- `[BAD ORDER, row zeroed]` — this slot's means sort the classes differently than the canonical ascending order: a mislabeled piece or a wildly foreign idle level
- `[FORWARD reaches the top of the scale, row zeroed]` — FORWARD sits at or above `LOOP_END`: a saturated reading, not a piece range

A zeroed row decodes INVALID on purpose — an untrusted slot must never inject a false piece. Then `--- GENERATED PER-SLOT ROWS ---` prints the full paste-ready `GASPETTO_ADC_SLOT_THRESHOLDS` table between the `BEGIN/END GENERATED PER-SLOT ROWS` markers. Columns are ascending piece starts; the last one (`LOOP_START`) sits above the slot's highest FORWARD reading plus a seating margin: a single game placement underestimates re-seating variance (swings up to ~1700 counts while moving a piece inside its socket), and the loop band must stay clear of them until the orange piece exists.

### Configuration

Game mode parameters are defined near the top of `main.cpp`:

- `kGameSlotCount` — number of slots covered (12 currently, the soldered main zone; up to 20)
- `kGameSamplesPerCombo` — samples per (piece, slot) combination (default 8)
- `kGameStabilitySpanMax` — max span for stability check (default 100 counts)
- `kGamePieceCount` — number of active pieces (5 currently, bump to 6 when LOOP_CALL piece exists)

To add the LOOP_CALL piece when the orange magnet is ready:

1. Fabricate the orange piece (2mm magnet, likely south pole)
2. Change `kGamePieceCount = 6` in `main.cpp`
3. Rebuild and flash — game mode now walks 576 steps (6 × 12 × 8)

### Example Session

```text
c                    # calibration mode
g                    # start game mode
--- GAME MODE STARTED ---
   480 steps (5 pieces x 12 slots x 8 samples)
   [1/480] slot 1, EMPTY (remove piece) [1/8]
   -> place/adjust, wait for stability, then SPACE
SPACE                # one SPACE for the EMPTY sweep records every slot
...
   [121/480] slot 2, FORWARD (green, 3mm) [1/8]   # auto-capture mode:
   recorded raw=15276 (span=3)                     # no SPACE, the settle does it
...
   [480/480] slot 12, TURN_LEFT (blue, 4mm) [8/8]
   recorded raw=8763 (span=2)

--- GAME MODE ENDED ---

--- PER-SLOT THRESHOLD ANALYSIS ---
   slot 1  [OK] TURN_LEFT[10848..10848] BACKWARD[13104..13106] EMPTY[13472..13472]
              TURN_RIGHT[14224..14224] FORWARD[14896..14896] gaps 2256,366,752,672
   slot 10 [OVERLAP TURN_RIGHT/FORWARD gap=-48, row zeroed]
   ...
   calibrated slots: 11/12

--- GENERATED PER-SLOT ROWS ---
/* --- BEGIN GENERATED PER-SLOT ROWS --- */
static const uint16_t GASPETTO_ADC_SLOT_THRESHOLDS[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS] = {
    /* slot 1 */ { 10848, 11976, 13289, 13848, 14560, 16396 },
    ...
};
/* --- END GENERATED PER-SLOT ROWS --- */
```

Copy the generated rows over the marker block of `soft/emu-pc/targets/gbox/include/piece_thresholds.h` and rebuild. This firmware decodes with those rows right away; production keeps using the legacy global defines until it adopts the same per-slot scan.

## Hardware Setup

- **Board**: STM32F411CE BlackPill
- **Slots**: 20 piece slots, each with a positive hall sensor wired to one of 20 ADC channels (five ADS1115 modules × 4 channels)
- **I2C buses**: four ADS1115 on hardware I2C1 (PB6/PB7), one on bit-banged I2C3 (PB4/PA8) — the I2C3 bus needs bit-bang recovery when it hangs
- **Slot wiring**: slots 1-4 on the 0x4B module (ADDR→SCL), 5-8 on 0x48 (ADDR→GND), 9-12 on 0x4A (ADDR→SDA), 13-16 on 0x49 (ADDR→VCC), 17-20 on I2C3 0x4A (ADDR→SDA)
- **LED**: NeoPixel strip (PC14), powered from the 5V rail
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
