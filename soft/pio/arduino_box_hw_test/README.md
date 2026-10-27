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
| `m` | Continuous readout of a slot with a live decoded piece (`<slot>m`, the number is required) |
| `x` | Frozen monitor: waits without sampling, SPACE opens one clean 3 s measurement (`<slot>x`, the number is required) |
| `g` | Start game mode (calibration only); `<slot>g` or `<slot>g<piece>` scopes a retake |
| `w` | Reprint the per-slot analysis and the paste-ready generated rows |
| `dfu` | Type the three letters in a row to jump to the STM32 DFU bootloader |

Serial is USB CDC at 115200 baud. `delay` calls are time-sliced so typed keys always get through.

Typed keys are echoed back so you always see what the board received: `[SP]` marks SPACE, and a new line starts when the board consumes the Enter. The monitor stop echoes its stop key and annotates the burst tail it drains as `[dropped N]`; a cluster import (`i`) echoes each accepted line with a `<<<` prefix instead of echoing the paste character by character.

## The Pieces

Four played pieces plus the empty board: FORWARD (green, 4 mm N),
TURN_RIGHT (yellow, 1 mm N), TURN_LEFT (blue, 4 mm S), LOOP_CALL (red,
1 mm S). Each carries a magnet glued inside a hole so the piece can be
placed upside down; the slot hall sensor reads a distinct window per pole
and per height, which is what the threshold table maps to raw ranges.
All printed values are raw ADS1115 counts (GAIN_ONE, single-ended, 3V3
rail) — the same scale the production decode compares against, so pasted
thresholds need no conversion.

Capture runs through the guided game mode below; manual per-key labeling
retired with the global-table workflow (gap analysis `t` and header `x` are gone too; `w` reprints the rows any time).

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

1. Flash the firmware, open the serial monitor, press `c` — calibration mode powers the 3V3 sensor rail and enables capture
2. Run the guided game (next section): `g`, then follow every prompt until the firmware prints the per-slot analysis and the generated rows
3. Paste the generated rows over the `BEGIN/END GENERATED PER-SLOT ROWS` block of `soft/emu-pc/targets/gbox/include/piece_thresholds.h`, rebuild and flash
4. Verify: `s` runs the scan pass — every calibrated slot must decode its placed piece; watch real re-seating variance with `<slot>m` before trusting a boundary

A slot that needs rework later: `5R` clears its clusters and a fresh game run
rebuilds them.

## Game Mode (Guided Calibration)

Game mode replaces manual `<slot><label>` typing with a guided sequence that prevents mislabeling and enforces reading stability. Activate it in calibration mode with `g`.

### How It Works

The firmware walks through all (piece, slot) combinations in a fixed order:

1. **By piece**: EMPTY → FORWARD → LOOP_CALL → TURN_RIGHT → TURN_LEFT
2. **By slot**: for each piece, all slots 1–12 — the soldered main zone (`kGameSlotCount` extends to 20 with the loop zone)
3. **By sample**: 8 readings per combination (configurable via `kGameSamplesPerCombo`)

Total: 5 pieces × 12 slots × 8 samples = **480 steps**. EMPTY is special: one SPACE confirms the whole board is clear and records every slot at once.

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
| `SPACE` | Record one sample (auto-capture off) or confirm the EMPTY sweep (whole board, or the scoped slot) |
| `a` | Toggle auto-capture (on by default: readings record themselves when they settle) |
| `r` | Reset the current (slot, piece) cluster and retry the step from scratch |
| `q` | Quit game mode and generate thresholds from collected data |
| `?` | Show game mode commands and current instruction |

### Scoped Retakes

Re-record one sensor without walking the whole campaign. A slot number
prefixes `g`, then one optional piece letter narrows it further:

```text
7R      # clear all slot 7 clusters first
7g      # scope slot 7: waits for a piece letter, or a second g for all five
7gg     # whole slot campaign (33 steps, EMPTY confirmed for slot 7 only)
7gf     # only slot 7 FORWARD: 8 captures, then the campaign ends and emits
```

Piece letters: `e` empty, `f` forward, `L` loop, `r` turn right, `l` turn
left. Any other key cancels the armed scope. On every end — natural or
`q` — the firmware re-emits all twelve rows from the RAM clusters, so the
untouched slots keep the data they already had from an earlier run; a scoped
run on a cold board will honestly print `[NO DATA]` for the others. `w`
reprints analysis and rows at any time without entering the game.

### Threshold Generation

When the game completes all steps (or you press `q`), the firmware analyzes **each slot's own clusters** — sensors idle up to 940 counts apart across the board, so one slot's clean gap can be the next slot's overlap — and prints one verdict per slot under `--- PER-SLOT THRESHOLD ANALYSIS ---`:

- `[OK] piece[min..max] ... gaps a,b,c,d` — the five classes separated on that sensor; thresholds sit at the midpoints and the printed gaps are the margins
- `[OVERLAP A/B gap=N, row zeroed]` — two classes overlap on this slot; its row decodes INVALID
- `[NO DATA]` / `[INCOMPLETE, row zeroed]` — this slot was not fully recorded
- `[BAD ORDER, row zeroed]` — this slot's means sort the classes differently than the canonical ascending order: a mislabeled piece or a wildly foreign idle level

A zeroed row decodes INVALID on purpose — an untrusted slot must never inject a false piece. Then `--- GENERATED PER-SLOT ROWS ---` prints the full paste-ready `GASPETTO_ADC_SLOT_THRESHOLDS` table between the `BEGIN/END GENERATED PER-SLOT ROWS` markers. Columns are ascending piece starts; the loop piece occupies the band below the EMPTY idle, and the last column (`LOOP_START`) is pinned to 65535: no piece plays above FORWARD, so no reading can ever reach that band and FORWARD's range simply extends to the top of the scale.

### Configuration

Game mode parameters are defined in `src/game.cpp`:

- `kGameSlotCount` — number of slots covered (12 currently, the soldered main zone; up to 20)
- `kGameSamplesPerCombo` — samples per (piece, slot) combination (default 8)
- `kGameStabilitySpanMax` — max span for stability check (default 100 counts)
- `kGamePieceCount` — entries the walk visits (5: the empty board plus the four played pieces)

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
   [121/480] slot 2, FORWARD (green, 4mm N) [1/8]   # auto-capture mode:
   recorded raw=15276 (span=3)                     # no SPACE, the settle does it
...
   [480/480] slot 12, TURN_LEFT (blue, 4mm S) [8/8]
   recorded raw=8763 (span=2)

--- GAME MODE ENDED ---

--- PER-SLOT THRESHOLD ANALYSIS ---
   slot 1  [OK] TURN_LEFT[10848..10848] LOOP_CALL[13104..13106] EMPTY[13472..13472]
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
   slot 1: raw=13104 min=13101 max=13107
```

After the health cycle, the firmware idles silently and only wakes to report probe changes or USB keys — `'s'` re-runs the test at will.
