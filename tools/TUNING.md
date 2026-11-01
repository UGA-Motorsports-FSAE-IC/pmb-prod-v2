# Dialling in the throttle body

End-to-end procedure: build the Debug firmware, attach an ST-Link, and use
`pmbtune.py` to measure the hardware and derive the control gains from that
measurement.

Nothing here asks you to guess a number. Every value lands in the `MEASURE ME`
block of `Core/Inc/constants.h`, and every one has a command that measures it
and prints the line to paste.

**Read the safety model (§10) before you command the motor for the first time.**

---

# READ FIRST — values you must set yourself

`pmbtune.py` measures properties of the **throttle body and motor**. It cannot
measure anything about your engine, your sensors' wiring, or your CAN bus —
those you must set correctly first, in `Core/Inc/constants.h`, or everything
below produces confident nonsense.

Work down this list before you start.

### 1. Sensor calibration — the one that invalidates everything

```c
#define TPS1_OPERATION_LB   700     // positive slope sensor
#define TPS1_OPERATION_UB   3615
#define TPS2_OPERATION_LB   3140    // opposite slope: LB > UB is correct here
#define TPS2_OPERATION_UB   225
#define APPS1_OPERATION_LB  966     // opposite slope sensor
#define APPS1_OPERATION_UB  1935
#define APPS2_OPERATION_LB  2120    // positive slope sensor
#define APPS2_OPERATION_UB  2980
```

**Every position number in this document is derived from these.** The firmware
converts raw TPS counts to permille with a straight line fitted between
`OPERATION_LB` and `OPERATION_UB`. If those endpoints are wrong, then `pv` is
wrong, `limits` reports the wrong stops, `ident` computes a gain in the wrong
units, and the derived gains are wrong — with no symptom that points at the
cause.

**Verify before tuning:** run `pmbtune.py monitor`, move the plate by hand from
fully closed to fully open, and confirm `pv` sweeps approximately 0 → 1000. If
it tops out at 600, or goes negative, or runs backwards, fix these constants
first. Same check for `pedal` against the throttle pedal.

Note `TPS2` and `APPS1` are opposite-slope sensors, so their `LB` is numerically
larger than their `UB`. That is intentional — the `SLOPE`/`INTERCEPT` macros
handle it. Don't "fix" it.

### 2. CAN frame layout

```c
#define TPS1_CAN_ID 215   /* + APPS2, TPS2, BS1 share this frame */
#define RPM_CAN_ID  1520
#define COOLANT_CAN_ID 1522
/* ... and the matching *_CAN_OFFSET for each */
```

If an ID or offset is wrong you will see a value that never changes, or garbage.
`monitor` catches this in ten seconds. Note RPM and coolant are **big-endian**
on this bus while the sensors are little-endian; the accessor macros differ
accordingly.

### 3. Engine values — set these from your engine, not from this file

These defaults came from the previous firmware. They are **not** measured and
`pmbtune.py` will not touch them.

| Constant | Default | What it means |
|---|---|---|
| `RPM_HOT_IDLE_TARGET` | 2000 | target idle, warm |
| `RPM_COLD_IDLE_TARGET` | 2000 | target idle, cold — usually should be *higher* than hot |
| `RPM_TOO_LOW_THRESHOLD` | 1500 | below this, idle switches to the aggressive save gain |
| `RPM_HOT_IDLE_MAX_THROTTLE` | 200 | ceiling on idle plate opening, permille |
| `RPM_COLD_IDLE_MAX_THROTTLE` | 200 | same, cold |
| `IDLE_TEMP_STEPDOWN_THRESHOLD` | 170 | °F at which hot idle values take over |
| `STARTER_RPM` | 300 | above this the engine is considered running, not cranking |
| `DOWNSHIFT_MAX_RPM` | 9000 | no blip above this — **set from your rev limit** |
| `BLIP_RPM_LOW` / `_HIGH` | 4000 / 9000 | rpm range the blip schedule spans |
| `GAS_PEDAL_IDLE_THRESHOLD` | 100 | below 10% pedal, the idle loop owns the target |

`RPM_COLD_IDLE_TARGET` being equal to the hot target is almost certainly wrong
for a real engine. Decide it deliberately before §12.

### 4. Motor driver limits

| Where | Constant | Default | Note |
|---|---|---|---|
| `constants.h` | `MAX_THROTTLE_MOTOR_PWM` | 1000 | duty ceiling in normal running |
| CubeMX (`tim.c`) | TIM16 PSC / ARR | 0 / 999 | gives a **48 kHz** carrier |

**The 48 kHz carrier is an open question.** Some brushed H-bridge drivers will
not switch that fast and will overheat or simply not drive. Confirm against
your Pololu part's datasheet. If it needs a lower carrier, change the prescaler
in CubeMX — do **not** hand-edit `tim.c`, it is generated. Nothing in the
control loop assumes 1000 counts; `board.h` reads full scale from the timer at
runtime.

### 5. Safety thresholds — review, don't inherit

```c
#define MAX_THROTTLE_INTENDED_VS_ACTUAL_DEVIATION 200  // 20%
#define MAX_GAS_PEDAL_SENSORS_DEVIATION           100  // 10%
#define MAX_THROTTLE_SENSORS_DEVIATION            100
#define HARD_BRAKING_PERCENTAGE_THRESHOLD         160
#define THROTTLEBODY_FAILURE_TIMEOUT              200  // ms
```

Check `MAX_GAS_PEDAL_SENSORS_DEVIATION` against the APPS plausibility
requirement in the current rulebook for your competition and year. I have not
verified these values against any specific rules edition — it is a
scrutineering item, not a tuning preference, so confirm it against the actual
text rather than trusting the inherited 10%.

Also be aware `THROTTLEBODY_FAILURE_TIMEOUT` (200 ms) interacts with the jam
guard's `THROTTLE_STALL_TIMEOUT_MS` (150 ms): you get roughly one unjam attempt
before the safety path takes over. That ordering is intentional. See the note in
`constants.h`.

### 6. Known wiring discrepancy

The CubeMX labels for **PA6/PA7 are reversed** relative to the harness.
`board.h` is authoritative (PA6 = downshift, PA7 = upshift) and `main.h`'s
`upshift_Pin`/`downshift_Pin` are wrong. Verify with a meter before you trust
either, and fix the `.ioc` labels when you next regenerate.

---

## Contents

| | Step | Engine | Needs |
|---|---|---|---|
| §0 | READ FIRST — values you must set yourself | — | — |
| §1 | What you need | — | — |
| §2 | One-time setup | — | — |
| §3 | Build and flash Debug | — | — |
| §4 | Confirm the link works | off | probe |
| §5 | `rate` — measure the real CAN rates | on or off | probe |
| §6 | `limits` — the jam fix | **off** | probe + motor |
| §7 | `stiction` — breakaway duty | **off** | probe + motor |
| §8 | `ff` — return-spring feedforward | **off** | probe + motor |
| §9 | `ident` → `derive` → `verify` — the gains | **off** | probe + motor |
| §10 | Safety model | — | — |
| §11 | Backward (closing) gains | **off** | probe + motor |
| §12 | Idle loop | **running** | probe |
| §13 | Downshift blip | **running** | probe |
| §14 | Locking the values in | — | — |
| §15 | Troubleshooting | — | — |
| §16 | Command reference | — | — |

---

## 1. What you need

**Hardware**

- ST-Link V2, V2-1, or V3 (any of them — this uses raw SWD, not a virtual COM
  port, so a bare V2 clone is fine)
- 4 wires to the board: `SWDIO` → **PA13**, `SWCLK` → **PA14**, `GND` → GND,
  and optionally `NRST`
- The car powering the board as normal

> **Do not** feed the ST-Link's 3.3 V pin into the board while the car is also
> powering it. Connect ground and the two signal lines only.

**Software**

- `arm-none-eabi-gcc` + `cmake` + `ninja` (you already build with these)
- Python 3.10 or newer

Works on Linux, macOS and Windows. Everything in `pmbtune.py` is pure Python
over SWD, so there is no platform-specific code — see §2 for the shell-syntax
differences and the one Windows driver gotcha.

On Windows, make sure `arm-none-eabi-gcc`, `cmake` and `ninja` are all on
`PATH` in the shell you build from. If you normally build inside STM32CubeIDE,
the easiest route is to install the Arm GNU Toolchain plus CMake and Ninja
standalone, since the CMake presets here are driven from the command line.

---

## 2. One-time setup

### Conventions used in this document

Commands are written for Linux/macOS. **Everything in `pmbtune.py` is pure
Python and works identically on Windows** — only the shell syntax differs:

| This document writes | On Windows (PowerShell) type |
|---|---|
| `python3` | `python` (or `py`) |
| `source .venv/bin/activate` | `.venv\Scripts\Activate.ps1` |
| a trailing `\` to continue a line | a trailing backtick `` ` ``, or just put it on one line |
| `cmd a \| tee f.txt` | `cmd a > f.txt` |
| `bash tests/run.sh` | see "Running the self-checks" below |

Forward slashes in paths work fine in PowerShell, so `--elf
build/Debug/pmb-prod-v2.elf` needs no change. Everything else — every
subcommand, every flag — is identical on all three platforms.

Use PowerShell, not `cmd.exe`. The old `cmd` shell has no `tee` and awkward
quoting; nothing below depends on PowerShell specifically, it is just less
painful.

### Linux / macOS

```bash
cd ~/Projects/FSAE/pmb-prod-v2

python3 -m venv .venv
source .venv/bin/activate
pip install -r tools/requirements.txt
```

### Windows (PowerShell)

```powershell
cd $HOME\Projects\FSAE\pmb-prod-v2

python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r tools\requirements.txt
```

If activation is blocked by execution policy, either run
`Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` for that session,
or skip the venv entirely and `pip install --user -r tools\requirements.txt`.

Install Python from python.org with **"Add python.exe to PATH"** ticked — the
Microsoft Store build sandboxes USB access in ways that can stop pyOCD seeing
the probe.

### Both: tell pyOCD about the target

pyOCD needs a device description for the STM32C0 before it knows the target:

```
pyocd pack find stm32c092
pyocd pack install stm32c092rctx
```

Check the probe is seen:

```
pyocd list
```

You should get a line naming your ST-Link. If not:

- **Linux** — USB permissions, see §15.
- **Windows** — driver binding. **Do not fight this**: §15 has three fixes, and
  the first (`--backend gdb`) needs no driver change at all — it routes memory
  access through an `ST-LINK_gdbserver` that uses ST's own driver. Read §15
  before installing anything.

If you end up on the gdb backend, add `--backend gdb` to every `pmbtune.py`
command in this document. Nothing else changes.

### Running the self-checks

Confirm the host-side maths is sound before you trust anything it prints. Both
suites must say "passed" — they are regression tests for real defects
(including a noise-bias bug that made the identifier report 50%-wrong damping),
so if they fail, the gains this tool derives are not trustworthy.

```bash
bash tests/run.sh          # Linux, macOS, Git Bash, WSL
```

The script skips gracefully rather than failing if a host C compiler or numpy
is missing, and prints a summary of what actually ran. Check that summary — a
skip is not a pass.

On Windows you can run the same script from **Git Bash** (ships with Git for
Windows) or WSL. The C half needs a *host* compiler; `arm-none-eabi-gcc` is a
cross-compiler and cannot build it. If you do not want to install MSYS2/MinGW
gcc, run the halves by hand in PowerShell:

```powershell
# control math (needs any host gcc/clang; skip if you have none)
gcc -std=c11 -Wall -Wextra -Werror -I Core\Inc `
    tests\test_pid.c Core\Src\pid.c -o test_pid.exe
.\test_pid.exe

# tuning math (needs only Python + numpy, so this one always runs)
python tests\test_pmbtune.py
```

The second one is the more important of the two for trusting derived gains, and
it has no compiler dependency at all.

---

## 3. Build and flash Debug

The telemetry ring and setpoint injection are **Debug-only**. A Release build
has no `pmb_log`, no injection path, and `pmbtune.py` will refuse to connect to
it. That is deliberate: nothing external can command the throttle on a Release
build.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Watch the tail of the build for the memory report:

```
Memory region   Used Size  Region Size  %age Used
           RAM:     14 KB        30 KB     46.7%
```

The 4 KB telemetry ring lives in there. If RAM overflows, drop
`PMB_LOG_SAMPLES` in `Core/Inc/tuning.h` from 256 to 128 (you lose history
depth, not sample rate).

Flash it however you normally do, or:

```bash
pyocd flash --target stm32c092rctx build/Debug/pmb-prod-v2.elf
```

> `pmbtune.py` reads `build/Debug/pmb-prod-v2.elf` by default to resolve symbol
> addresses. **Keep that ELF in sync with what is actually flashed.** If you
> rebuild without reflashing, the addresses will be wrong and you will be
> reading and writing garbage. Pass `--elf <path>` if yours lives elsewhere.

---

## 4. Confirm the link works

Engine off, ignition on so the board and CAN sensors are powered.

```bash
python3 tools/pmbtune.py params
```

Expect the whole live parameter block:

```
  kp_fwd             1900
  ki_fwd             35
  kd_fwd             0
  ...
  soft_min           20
  soft_max           980
```

This is read-only and completely safe — it touches nothing but RAM reads.

If it prints `pmb_tune.magic is 0x00000000` you have a Release build flashed, or
a stale ELF. If it says a symbol is missing, same thing.

Now watch live telemetry and move the pedal by hand:

```bash
python3 tools/pmbtune.py monitor
```

```
t=   142380ms  sp=   20  pv=   18  err=     2  u=     0  rpm=     0  pedal=   12
```

`pedal` must track the throttle pedal and `pv` must track the plate. If `pv`
never changes, the control loop is not getting TPS frames and nothing below
will work. Ctrl-C to stop.

**Note on flag order:** `--i-understand`, `--ceiling`, `--elf` and
`--target-name` are global and must come **before** the subcommand.
`pmbtune.py --i-understand limits` works; `pmbtune.py limits --i-understand` is
an error.

---

## 5. `rate` — do this before anything else

Everything downstream assumes you know how fast TPS and rpm actually arrive.
The old firmware ran the PID at a fixed 1 kHz against a CAN value updating far
slower, so most samples were stale repeats.

```bash
python3 tools/pmbtune.py rate --seconds 10
```

```
  control loop samples : 4180
  period  median/mean  : 2.00 / 2.39 ms
  period  min/max      : 1 / 10 ms
  effective rate       : 418 Hz
  pv   changes every   : 2.0 ms (~500 Hz of real new data)
  rpm  changes every   : 20.0 ms (~50 Hz of real new data)
```

**How it works.** Purely passive — it reads the telemetry ring and looks at
timestamps. The loop period is the difference between consecutive `t_ms`
values. The "real new data" figure is different and more useful: it diffs the
timestamps *only across samples where the value actually changed*, which
separates how often the loop runs from how often the sensor gives it something
new. A loop running at 500 Hz on data arriving at 50 Hz is processing the same
number nine times out of ten.

Act on it:

- **`pv` changes every N ms** is your true position sample rate. If it is far
  slower than the loop rate, the derivative term is mostly noise. Set
  `THROTTLE_CONTROL_TIMEOUT_MS` comfortably above it (it is the watchdog, not
  the schedule — the loop is event-driven now).
- **`rpm` changes every N ms** is what `PID_RPM_DT` should be. If rpm arrives
  at 50 Hz, a 10 ms idle loop re-processes each value four times. Harmless for
  the PI terms, but it makes `PID_RPM_D` meaningless — leave idle D at 0 unless
  `PID_RPM_DT` matches the real interval.

Write both numbers down.

---

## 6. `limits` — measure the stops (this is the jam fix)

> **ENGINE OFF.** Ignition on. Hands clear of the throttle body.

This is the most important step. The plate jams because the old firmware
commanded position 0, which drove it into the closed mechanical stop, and the
unbounded integral then pinned full PWM into that stop. The new firmware clamps
every target into `[THROTTLE_SOFT_MIN, THROTTLE_SOFT_MAX]` — but those are
placeholders (20/980) until you run this.

```bash
python3 tools/pmbtune.py --i-understand --ceiling 300 limits
```

```
ENGINE OFF. Creeping to each mechanical stop under a PWM ceiling...
  closed stop rests at 8 permille
  open stop rests at 994 permille

  #define THROTTLE_SOFT_MIN       23
  #define THROTTLE_SOFT_MAX       979

Measured stops 8..994 with a 15 permille margin on each end.
```

**How it finds it.** Drives a constant open-loop duty — `--ceiling` counts, no
PID involved — first toward the closed stop, then toward the open stop. The
plate runs to each end and stays there. It takes the median of the last 20
position samples as the resting position, which rejects the sensor noise that a
single reading would pick up. Then it subtracts `--margin` from each end so the
usable window sits clear of both stops.

The reason this fixes the jam is that the firmware now clamps every target into
that window, so no code path — pedal map, idle, blip, closure, or SWD injection
— can command the plate against a stop for the integral to wind up against.

**Choosing `--ceiling`:** start at 300. If the plate does not reach the open
stop, the spring is stronger than 300 counts of duty — raise in steps of 100.
Do not start high; the point of the ceiling is that the plate arrives at the
stop gently.

**Choosing `--margin`:** default 15 permille (1.5%) off each end. Raise it to
25–30 if the plate has ever stuck. The cost of a larger margin is a little lost
travel at both ends; the cost of too small a margin is the jam coming back.

### Apply it live instead of reflashing

You will measure five or six things in this session. Rather than reflash
between each, push the measurement into the live parameter block and keep
going:

```bash
python3 tools/pmbtune.py params --set soft_min=23 --set soft_max=979
```

That takes effect immediately. Everything measured afterwards then respects the
real travel limits. You will paste the whole set into `constants.h` once at the
end (§14).

> Live parameters are RAM. They are lost on reset. Nothing is permanent until
> §14.

---

## 7. `stiction` — breakaway duty

> **ENGINE OFF.**

A geared DC throttle body needs a minimum duty before it moves at all. Below
it, the motor does nothing — which looks exactly like "the gains aren't doing
anything".

```bash
python3 tools/pmbtune.py --i-understand --ceiling 400 stiction
```

```
ENGINE OFF. Ramping duty until the plate breaks free...
  opening: breakaway at 112 counts (501 -> 523 permille)
  closing: breakaway at 64 counts (523 -> 498 permille)

  #define THROTTLE_STICTION_PWM   112
  #define THROTTLE_DIR_HYSTERESIS 56
```

**How it finds it.** Ramps the open-loop duty up from 0 in small steps, holding
each briefly, and records the first duty at which the position has moved more
than `--motion` permille away from where it started. Then repeats driving the
other direction. The ramp is deliberately slow so what it catches is the
*static* breakaway, not an inertial overshoot from a sudden step.

It reports both directions and recommends the larger. Closing is usually lower
because the return spring helps.

The hysteresis recommendation is half the breakaway duty, and the reasoning is
in the output: below that figure a direction reversal cannot move the plate
anyway, so refusing it costs nothing and stops the H-bridge chattering back and
forth over TPS noise.

```bash
python3 tools/pmbtune.py params --set stiction_pwm=112 --set dir_hysteresis=56
```

---

## 8. `ff` — return-spring feedforward

> **ENGINE OFF.**

Holding the plate at a given angle takes a roughly linear amount of effort
against the return spring. Supplying that directly means the integral only has
to correct the residual, instead of rediscovering the entire spring curve after
every move.

```bash
python3 tools/pmbtune.py --i-understand --ceiling 500 ff --points 9 --dwell 2.0
```

```
ENGINE OFF. Holding positions closed-loop and recording hold duty...
    43 permille -> held   45, duty   92.0
   160 permille -> held  158, duty  121.0
   ...
   860 permille -> held  857, duty  402.0

  hold duty = 0.3621 * position + 78.4   (R2 = 0.982)

  #define THROTTLE_FF_SLOPE       362
  #define THROTTLE_FF_OFFSET      78

  hold duty at mid-travel (501 permille) is 260 counts.
  Pass that to ident:   pmbtune.py ... ident --bias 260
```

**How it finds it.** Commands a series of hold positions spread across the
usable travel, closed-loop so the plate actually gets there. At each one it
waits `--dwell` seconds, then takes the median duty and median position over
the **last third** of the dwell — the settled part, discarding the transient
getting there. That gives one (position, duty) pair per point. A least-squares
straight line through those pairs is the spring curve: slope and offset.

Positions the plate could not hold within `--tolerance` are reported and
skipped rather than fitted, so a point that ran into a stop cannot bend the
line.

The firmware then adds `(slope × target)/1000 + offset` to the controller
output, so the PID starts each move already supplying roughly the right holding
effort and only has to correct the remainder.

**Read the R².** Above ~0.95 the spring is behaving linearly and the fit is
good. Below 0.8 either the holds had not settled (raise `--dwell`) or the
spring is genuinely nonlinear, in which case use the feedforward anyway but
expect the integral to do more work at the extremes.

Points that could not be held get skipped and reported — if most of them skip,
your `--ceiling` is too low for closed-loop holding.

**Keep the mid-travel hold duty.** §9 needs it.

```bash
python3 tools/pmbtune.py params --set ff_slope=362 --set ff_offset=78
```

---

## 9. The gains: `ident` → `derive` → `verify`

### 9a. `ident` — identify the plant

> **ENGINE OFF.** This commands open-loop PWM.

```bash
python3 tools/pmbtune.py --i-understand --ceiling 500 \
    ident --bias 260 --amplitude 180 --period 0.8 --seconds 15 \
          --out plant.json --csv ident-raw.csv
```

**How it works.** This is the mathematically substantial step, so it is worth
understanding what it is doing.

1. **Excite.** It parks the plate mid-travel closed-loop, then switches to open
   loop and applies a square wave of `±amplitude` counts *around* `--bias`.
   Open loop matters: a closed-loop test measures the loop you already have, not
   the plant underneath it.

2. **Fit a discrete model.** A throttle plate is a second-order system — rotor
   and plate inertia, damping, and a return spring — so it is fitted as

   ```
   pv[n] = a1·pv[n-1] + a2·pv[n-2] + b1·u[n-1-d] + b2·u[n-2-d]
   ```

   where `d` is transport delay in whole samples. The coefficients come from a
   least-squares fit, and `d` from a search over candidate integer delays.

3. **Remove the noise bias.** Plain least squares is *biased* here, because its
   own regressors (`pv[n-1]`, `pv[n-2]`) carry sensor noise, which correlates
   with the residual and drags the estimate toward excess damping. This is not
   hypothetical: at 0.8 permille of TPS noise it reported damping 0.59 against a
   true 0.40. So the fit is iterated with **instrumental variables** — the
   noisy regressors are replaced by the previous iterate's simulated output,
   which is driven by the input alone and therefore uncorrelated with the
   output noise. Converges in 2–3 passes and recovers 0.40 correctly.

4. **Score honestly.** Candidates are ranked on **free-run simulation** fit:
   the model is driven by the recorded input only, with no access to the real
   position, and compared against what actually happened. The conventional
   one-step-ahead R² is useless as a gate — it sits at 0.9999 for almost any
   model because `pv[n-1]` does the predicting, and it happily hid the damping
   error above.

5. **Convert to physical terms.** The discrete poles are the roots of
   `z² − a1·z − a2`; mapping them back with `s = ln(z)/Ts` gives the continuous
   pole pair, from which `ωn = |s|` and `ζ = −Re(s)/ωn`. DC gain is
   `(b1+b2)/(1−a1−a2)`, in permille of travel per PWM count.

**`--bias` is not optional in practice.** It is the holding duty from §8. With
no bias the mean open-loop effort is zero, the spring parks the plate on the
closed stop, and you identify the stop instead of the plate. The tool warns you
if you leave it at 0.

**Constraint:** `|bias| + amplitude ≤ ceiling`, or the firmware clips the
excitation and the model is distorted. Here 260 + 180 = 440 ≤ 500. ✓

**`--amplitude`** should be large enough to move the plate well clear of
stiction (§7) but small enough to stay off both stops. Start around
`0.7 × breakaway` and check the resulting travel in the output.

```
  samples        : 6240 at 2.00 ms
  simulation R2  : 0.9948  (free-run, not one-step)
  DC gain K      : 1.4802 permille per PWM count
  natural freq   : 48.9 rad/s (7.8 Hz)
  damping zeta   : 0.412
  deadtime       : 6.0 ms (3 samples)

  model written to plant.json
```

**The R² gate is the whole point.** This is a *free-run simulation* fit — the
model is driven by the input alone and compared against reality. The usual
one-step-ahead R² is near 1.0 for almost any model and will happily hide a
50%-wrong damping ratio.

- **R² > 0.95** — good, proceed.
- **R² 0.90–0.95** — usable, treat the derived gains as a starting point.
- **R² < 0.90** — `derive` will refuse. Do not force it. Causes, in order of
  likelihood: the plate hit a stop (lower amplitude or fix the bias), the
  excitation was too small to beat stiction (raise amplitude), or TPS is
  arriving too slowly (go back to §5).

### 9b. `derive` — gains from the model

Offline. No probe, no hardware, nothing is written to the target.

```bash
python3 tools/pmbtune.py derive plant.json --out gains.json
```

```
plant: K=1.4802 wn=48.9 zeta=0.412 theta=6.0ms  (sim R2=0.9948)

IMC-PID (Rivera/Morari, 2nd-order process), tau = 20.4 ms
predicted on the identified model under the firmware's own fixed-point
controller, step 200 -> 600 permille:

     lambda        Kc      rise   overshoot   settle2%
      12.0ms   0.6469      24ms       26.2%      172ms
      15.0ms   0.5440      26ms       23.4%      174ms
      ...
      37.9ms   0.2535      44ms        2.8%      192ms   <-- meets budget
      47.3ms   0.2068      54ms        0.1%      210ms   <-- meets budget

selected lambda = 37.9 ms (fastest with overshoot <= 5.0%)
  Kc = 0.2535   Ti = 15.9 ms   Td = 24.7 ms
  predicted: rise 44 ms, overshoot 2.8%, settle(2%) 192 ms

Firmware values (stored x1000; Kd is per millisecond, see pid.h):
  #define PID_FORWARD_P   254
  #define PID_FORWARD_I   15919
  #define PID_FORWARD_D   6274
```

**How it works.** Two stages.

**Stage 1 — the formulas.** For a second-order process with deadtime, IMC-PID
(Rivera/Morari) gives the gains in closed form as a function of one design
parameter λ, the desired closed-loop time constant. With `τ = 1/ωn`:

```
Kc = 2·ζ·τ / (K · (λ + θ))        Ti = 2·ζ·τ        Td = τ / (2·ζ)
```

Those convert to the firmware's stored form as `Kp = Kc`, `Ki = Kc/Ti`,
`Kd = Kc·Td`, each scaled by 1000 (and Kd expressed per millisecond — see
`pid.h` for why).

**Stage 2 — simulate before believing.** Rather than trust a formula, each
candidate λ is run through a closed-loop simulation: the identified ARX model
as the plant, and a **bit-for-bit Python mirror of the firmware's own
`pid_step()`** as the controller. So the predicted response includes the
integer truncation, the integral clamp, conditional anti-windup, the deadtime
and the output limits — not an idealised continuous PID that the firmware does
not actually implement. Rise time, overshoot and settling time are measured off
that simulated response.

λ is the single tuning knob — larger is slower and more robust. The tool sweeps
it and picks the **fastest λ that keeps predicted overshoot within budget**.

The mirror is kept honest by `tests/test_pmbtune.py`, which checks it against
the same cases as the C test suite. If the two ever drift apart, the predictions
become meaningless, so that test failing is a hard stop.

The budget defaults to 5% because overshoot on a throttle plate means
travelling toward a mechanical stop. Change it deliberately:

```bash
python3 tools/pmbtune.py derive plant.json --max-overshoot 2   # calmer
python3 tools/pmbtune.py derive plant.json --lam 0.025         # pin λ yourself
```

Note the scope statement the tool prints: this is **one** model around **one**
operating point, and the excitation swung both ways around the bias, so the fit
averages opening and closing. These numbers go in the FORWARD set. See §11.

### 9c. `verify` — measured vs predicted

> **ENGINE OFF.** This steps the throttle closed-loop.

```bash
python3 tools/pmbtune.py --i-understand --ceiling 700 \
    verify gains.json --model plant.json --seconds 12 --period 2.0 \
           --csv verify-raw.csv
```

```
current gains: {'kp_fwd': 1900, 'ki_fwd': 35, 'kd_fwd': 0}
applied kp_m=254 ki_m=15919 kd_m=6274

measured over 5 rising steps:
  rise_10_90_ms        47.0  (min 44.0, max 52.0)
  overshoot_pct         3.4  (min 2.1, max 5.0)
  settle_2pct_ms      205.0  (min 188.0, max 230.0)

predicted from the model:
  rise_10_90_ms        44.0
  overshoot_pct         2.8
  settle_2pct_ms      192.0

restored previous gains (pass --keep to leave them applied)
```

**How it works.** Writes the candidate gains into the live parameter block over
SWD, then injects a square-wave position setpoint and records the result. It
locates each setpoint edge moving *toward* `--sp-to` and, on the segment after
it, measures rise time (10–90% of the commanded span), overshoot (peak beyond
the target as a percentage of the span) and 2% settling time. It reports the
median across all captured steps plus the spread, then prints the same three
metrics as predicted by §9b for direct comparison.

**This is the proof step.** Measured within ~15% of predicted means the model
describes the real plate and the gains are sound.

If they disagree badly, **the model is wrong, not the gains** — go back to
`ident` and get the R² up. Do not start hand-twiddling λ to chase the
measurement; that is exactly how you end up back at guessed numbers.

`verify` restores the previous gains when it exits. Add `--keep` to leave them
applied for a drive, remembering they are RAM-only until §14.

---

## 10. Safety model

Read this once.

**Nothing drives the motor without `--i-understand`.** `rate`, `log`,
`monitor`, `params` and `derive` never command anything.

**The PWM ceiling is enforced in firmware, not here.** `--ceiling` is written
into the target and the firmware clamps `|PWM|` to it on every control cycle.
A bug in this script cannot exceed it. The ceiling starts at **0** after every
reset, so injection cannot drive until a host explicitly raises it.

**Injection dies if the script does.** The host must bump a heartbeat word; if
it goes stale for 250 ms the firmware latches injection off by itself. So
Ctrl-C, a crash, an unplugged probe, or a USB glitch all release the throttle
within a quarter second. It does not hold the last commanded value.

**Safety paths always win.** If any implausibility is active — sensor
deviation, CAN timeout, throttle-not-at-intended — injection is ignored
entirely for that cycle and the safety target applies.

**Injected positions are clamped like any other target**, into the soft travel
limits. The injection path cannot command a hard stop even if you ask it to.

**Release builds cannot be injected into at all.** The code is not compiled in.

What this does *not* protect against: you are commanding a real throttle plate
with a real engine attached. Engine off for §6–§11. Keep hands clear. Know
where the kill switch is.

---

## 11. Backward (closing) gains

The closing direction is a genuinely different plant — the return spring
assists instead of fighting. The firmware keeps three gain sets for this
reason: opening, closing, and closing below `THROTTLE_LOW_REGION` (near the
closed stop, where the spring does nearly all the work).

`ident` does **not** separate opening from closing. Doing that rigorously needs
a segment-wise fit, which this tool does not do, so do not pretend the backward
numbers are derived.

Procedure:

1. Start the backward set equal to the forward set.
2. Step downward and watch for overshoot past the target on closing.
3. Reduce `kp_rev` until closing is clean. The spring assists, so it usually
   lands lower than forward.
4. Set `kp_rev_low` lower again — typically half of `kp_rev` — so the plate
   does not slam the last few percent into the closed stop.

```bash
# apply a candidate live and watch it
python3 tools/pmbtune.py params --set kp_rev=180 --set ki_rev=12000
python3 tools/pmbtune.py --i-understand --ceiling 700 \
    verify gains.json --sp-from 600 --sp-to 200 --keep

python3 tools/pmbtune.py params --set kp_rev_low=90 --set ki_rev_low=6000
```

Note the reversed `--sp-from`/`--sp-to`: that makes the measured step a closing
one.

Record what you settle on and why. These are empirical, not derived, and the
next person needs to know the difference.

---

## 12. Idle loop

> **ENGINE RUNNING.** Warmed up. Do not touch the pedal.

The old idle controller was pure proportional with a zero base, so at target
rpm it commanded 0% throttle — it could not hold an idle by construction, only
react once the idle had already sagged, then close the plate again. It is now
`position = base_opening + PI(rpm error)`.

**The base opening is the missing piece and it is engine-specific.** Until it
is set, idle still will not hold.

Get the engine idling by whatever means you currently use (idle screw, pedal,
starting fluid — however you do it), hold it steady, then:

```bash
python3 tools/pmbtune.py idleff --seconds 15
```

```
ENGINE RUNNING AT IDLE. Recording the plate opening that holds it.
  idle rpm      : 1980
  plate opening : 64.2 permille

  #define IDLE_FF_HOT  64   /* if the engine was warm */
  #define IDLE_FF_COLD 64   /* re-measure this one cold */
```

**How it finds it.** Reads only — commands nothing. It takes the median rpm
over the window, keeps just the samples where rpm sits within `--rpm-band` of
that median (i.e. the genuinely steady stretches, discarding any hunting), and
reports the median plate position across those samples. That position is, by
definition, the opening that holds the idle with no net rpm error — which is
exactly the feedforward base term. If fewer than a quarter of the samples
qualify as steady, it refuses rather than averaging a hunting idle.

**Run it twice**: once warm for `IDLE_FF_HOT`, once from cold start for
`IDLE_FF_COLD`. A cold engine needs more opening. If you only ever measure
warm, cold starts will sag.

If it refuses with "idle never held still", the idle is hunting too much to
measure. Steady it first.

```bash
python3 tools/pmbtune.py params --set idle_ff_hot=64 --set idle_ff_cold=64
```

Then tune the trim. Start with P only and leave I at 0:

```bash
python3 tools/pmbtune.py params --set idle_kp=500 --set idle_ki=0
python3 tools/pmbtune.py log idle-test.csv --seconds 30
```

Blip the pedal and let it drop back to idle, repeatedly, while that logs. Then
look at the `rpm` column: if idle returns slowly, raise `idle_kp`; if it hunts
up and down, lower it. Only add `idle_ki` (small — start around 50) once P
alone is stable, to remove a steady offset from the target.

`idle_kp_save` applies below `RPM_TOO_LOW_THRESHOLD` and should be more
aggressive — it is the catch for an idle about to stall.

---

## 13. Downshift blip

> **ENGINE RUNNING.** On a stand or with the car safely supported.

The blip target is now scheduled against engine speed, with both endpoints
defaulting to the old fixed 800 (so the schedule is flat and behaviour is
unchanged until you change it).

```bash
python3 tools/pmbtune.py log blip-test.csv --seconds 20
```

Perform downshifts while that runs, then look at `sp`, `pv` and `rpm` around
each shift. You want engine speed to come up enough to match the lower gear
without flaring past it.

```bash
# lower blip at low rpm, bigger at high rpm
python3 tools/pmbtune.py params --set blip_target_low=650 --set blip_target_high=850
```

`BLIP_RPM_LOW` / `BLIP_RPM_HIGH` in `constants.h` set the rpm range the
schedule interpolates across.

Two things are enforced regardless of what you set: the blip is a tick deadline
the throttle loop checks itself (so a stalled shift task cannot leave the
throttle open), capped at `BLIP_MAX_DURATION_MS`, and the target is clamped to
the soft travel limits like any other.

**Also check the upshift cut with a scope on PA8.** It previously never
happened — the implausibility task re-drove that pin every 10 ms and cancelled
the 200 ms cut almost immediately. Confirm PA8 now stays low for the full
`UPSHIFT_CUT_DURATION`.

---

## 14. Locking the values in

Everything so far is RAM and dies on reset. Make it permanent.

**1. Dump the final live set:**

```bash
python3 tools/pmbtune.py params > tuned-params.txt   # then open the file
```

(Plain redirect rather than `tee`, so the same line works in PowerShell.)

**2. Paste into `Core/Inc/constants.h`.** Each live parameter maps to one
`#define` in the `MEASURE ME` block:

| live parameter | `constants.h` |
|---|---|
| `soft_min` / `soft_max` | `THROTTLE_SOFT_MIN` / `THROTTLE_SOFT_MAX` |
| `stiction_pwm` | `THROTTLE_STICTION_PWM` |
| `dir_hysteresis` | `THROTTLE_DIR_HYSTERESIS` |
| `ff_slope` / `ff_offset` | `THROTTLE_FF_SLOPE` / `THROTTLE_FF_OFFSET` |
| `kp_fwd` / `ki_fwd` / `kd_fwd` | `PID_FORWARD_P` / `_I` / `_D` |
| `kp_rev` / `ki_rev` / `kd_rev` | `PID_BACKWARD_P` / `_I` / `_D` |
| `kp_rev_low` / `ki_rev_low` | `PID_LOW_BACKWARD_P` / `_I` |
| `i_max` / `deadband` | `THROTTLE_PID_I_MAX` / `THROTTLE_PID_DEADBAND` |
| `d_lpf_shift` | `THROTTLE_D_LPF_SHIFT` |
| `idle_ff_hot` / `idle_ff_cold` | `IDLE_FF_HOT` / `IDLE_FF_COLD` |
| `idle_kp` / `idle_ki` / `idle_kd` | `PID_RPM_P` / `_I` / `_D` |
| `idle_kp_save` / `idle_i_max` | `PID_RPM_SAVE_P` / `PID_RPM_I_MAX` |
| `blip_target_low` / `blip_target_high` | `SHIFT_BLIP_TARGET_LOW_RPM` / `SHIFT_BLIP_TARGET_HIGH_RPM` |

Delete the `PLACEHOLDER` comment on each line you replace, so the next person
can see at a glance what has been measured and what has not.

**3. Commit `plant.json`, `gains.json` and your raw CSVs** alongside the
change. The model is the evidence for the gains; without it the numbers are
indistinguishable from guesses six months from now.

**4. Rebuild both configurations and re-verify:**

```bash
cmake --build --preset Debug
cmake --build --preset Release
```

Flash **Debug** and re-run `verify` one final time. This confirms the pasted
constants match what you tuned live.

Then flash **Release** for actual running. Release is `-O2`, so the control
loop executes faster than the `-Og` Debug build you tuned on. The gains are
rate-independent (the loop measures its own `dt`), so this should not move the
response — but confirm it by re-running `rate` on the Release build and
checking the loop period is the same. Release has no injection, so `verify`
will not work against it; use `log` and real pedal inputs.

---

## 15. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `no debug probe found` (Linux) | USB permissions. Add `SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"` to `/etc/udev/rules.d/99-stlink.rules`, then `sudo udevadm control --reload && sudo udevadm trigger`, and replug. |
| `no debug probe found` (Windows) | Driver binding — see the note below this table. |
| `pmb_tune.magic is 0x...` | Release build flashed, or the ELF does not match the flash. Rebuild Debug, reflash, retry. |
| `symbol 'pmb_params' missing` | Built without `PMB_TUNING`, i.e. not the Debug preset. |
| `target sample is N bytes, host expects 16` | `tuning.h` and `pmbtune.py` are out of sync. You changed one without the other. |
| `almost no samples` | Control loop is not running, or not reaching `pmb_log_push`. Check the task started and CAN is alive. |
| `pv` never changes in `monitor` | No TPS frames. Check CAN wiring and that frame ID 215 is present. Nothing below §5 will work until this is fixed. |
| `ident` R² below 0.9 | In order: plate hit a stop (check `--bias`, lower `--amplitude`), excitation too small for stiction, TPS too slow. |
| `no lambda keeps overshoot under 5%` | The model is almost certainly wrong — check R² first. Only then raise `--max-overshoot`. |
| Plate buzzes or chatters audibly | `dir_hysteresis` too low. Raise it toward the breakaway duty from §7. |
| Plate hunts around the target | Add a small `deadband` (start at 5 permille), or the derived gains are too hot — re-`derive` with a lower `--max-overshoot`. |
| Throttle relay keeps opening | `throttle_not_at_intended_implausibility` is tripping. The plate is not reaching its target — look at §15's stiction and ceiling rows, and check the jam guard is not firing. |
| Jam guard fires repeatedly | Something is mechanically wrong. Do not raise the thresholds to silence it; that guard is protecting the motor from a stalled-rotor burn. |
| `no such ELF` | Pass `--elf build/Debug/pmb-prod-v2.elf` explicitly. |
| pyOCD cannot find the target type | `pyocd pack find stm32c092` then `pyocd pack install <name>`. Or pass `--target-name` with whatever `pack find` reported. |

**Only one program may own the probe.** You cannot run `monitor` in one
terminal and `verify` in another.

### If pyOCD cannot see the probe on Windows

This is the one Windows-specific hurdle. Understand the cause, then pick one of
three fixes — **you do not have to change drivers.**

**The cause.** pyOCD talks to the ST-Link through **libusb**, which on Windows
needs a WinUSB-compatible driver bound to the device. ST's own driver (the one
STM32CubeProgrammer and CubeIDE use) is not one. So a probe that works
perfectly in ST's tools can be completely invisible to pyOCD. ST-Link V2-1 and
V3 units often enumerate as WinUSB already and just work; standalone V2 dongles
and clones usually do not.

---

#### Option A — `--backend gdb` (no driver change, recommended first try)

`pmbtune.py` can get its memory reads and writes through a **GDB server**
instead of driving USB itself. The server owns the probe and uses ST's own
driver, so nothing needs rebinding and CubeProgrammer/CubeIDE keep working.

`ST-LINK_gdbserver` ships with **STM32CubeCLT** and **STM32CubeIDE**. In one
terminal:

```powershell
ST-LINK_gdbserver --swd --attach --port-number 61234
```

Then in another, add `--backend gdb` to every command:

```powershell
pmb --backend gdb params
pmb --backend gdb --i-understand --ceiling 300 limits
```

> **Check the flag spelling with `ST-LINK_gdbserver --help`** — I have not
> verified these against your CubeCLT version. What matters is that you select
> **SWD** and **attach without reset or halt**. If the server halts the core on
> connect, `pmbtune.py` will not un-halt it: by design this tool issues no run
> control whatsoever, so it can never stop a running engine. You would see
> `rate` report no samples.

OpenOCD works the same way if you already have it running:

```
openocd -f interface/stlink.cfg -f target/stm32c0x.cfg
pmb --backend gdb --gdb-port 3333 params
```

(OpenOCD's default GDB port is 3333; ST-LINK_gdbserver's is 61234.)

The backend only ever issues `m` (read memory) and `M` (write memory), both of
which go over the AHB access port and do not require halting the core. Reads
are chunked to fit the server's advertised packet size automatically.

---

#### Option B — rebind the driver with Zadig

Replace the ST-Link's driver with WinUSB using [Zadig](https://zadig.akeo.ie/):
*Options → List All Devices*, select the ST-Link interface, choose **WinUSB**,
Replace Driver. Then `--backend pyocd` works normally.

> **The trade-off:** binding WinUSB generally stops ST's own tools from seeing
> that probe until you revert the driver in Device Manager. If you flash with
> STM32CubeProgrammer you will be switching back and forth — or sidestep it
> entirely by flashing with `pyocd flash` (§3) and leaving the probe on WinUSB
> permanently.

---

#### Option C — WSL2 with USB passthrough

Attach the probe into WSL2 with `usbipd-win`, then run the whole session inside
Linux, where the udev rule from the table above applies. Heavier setup, but
once working it behaves exactly like the Linux instructions in this document.

---

**Which to pick.** Try A first: it is a one-flag change, needs no admin rights,
and leaves every other tool working. Use B if you want the simpler one-process
setup and do not mind the driver dance. Use C if you would rather run the Linux
path end to end.

---

## 16. Command reference

Global flags come **before** the subcommand:

```
--elf PATH            built ELF for symbol lookup (default build/Debug/...)
--backend pyocd|gdb   how to reach the probe (default pyocd; use gdb on
                      Windows if pyocd cannot bind the probe -- see §15)
--gdb-host HOST       --backend gdb only (default 127.0.0.1)
--gdb-port N          --backend gdb only (default 61234; OpenOCD uses 3333)
--target-name NAME    pyocd target type (default stm32c092rctx)
--frequency HZ        SWD clock, pyocd backend only (default 4000000)
--i-understand        consent to commanding the throttle motor
--ceiling N           hard PWM cap enforced in firmware (default 400)
```

| Command | Motor | Engine | Purpose |
|---|---|---|---|
| `params [--set k=v ...]` | no | any | read / write the live parameter block |
| `monitor` | no | any | live one-line telemetry |
| `log OUT.csv [--seconds N]` | no | any | stream telemetry to CSV |
| `rate [--seconds N]` | no | any | measure real control and CAN rates |
| `limits [--seconds N] [--margin N]` | **yes** | off | measure the mechanical stops |
| `stiction [--seconds N] [--motion N]` | **yes** | off | breakaway duty |
| `ff [--points N] [--dwell S] [--tolerance N]` | **yes** | off | spring feedforward fit |
| `ident --bias N [--amplitude N] [--period S] [--seconds N] [--out F] [--csv F]` | **yes** | off | plant model |
| `derive MODEL [--lam S] [--max-overshoot P] [--sp-from N] [--sp-to N] [--out F] [--force]` | no | — | gains, offline |
| `verify GAINS [--model F] [--sp-from N] [--sp-to N] [--period S] [--keep] [--csv F]` | **yes** | off | measured vs predicted |
| `idleff [--seconds N] [--rpm-band N]` | no | **running** | idle base opening |

### The short version

The numeric values below are the worked examples from this document — replace
each with what your own hardware reports at that step.

**Linux / macOS / Git Bash:**

```bash
# setup
source .venv/bin/activate
cmake --preset Debug && cmake --build --preset Debug
pyocd flash --target stm32c092rctx build/Debug/pmb-prod-v2.elf

P="python3 tools/pmbtune.py"

# measure (engine off)
$P rate --seconds 10
$P --i-understand --ceiling 300 limits
$P params --set soft_min=23 --set soft_max=979
$P --i-understand --ceiling 400 stiction
$P params --set stiction_pwm=112 --set dir_hysteresis=56
$P --i-understand --ceiling 500 ff --points 9 --dwell 2.0
$P params --set ff_slope=362 --set ff_offset=78

# derive and prove
$P --i-understand --ceiling 500 ident --bias 260 --amplitude 180 --out plant.json
$P derive plant.json --out gains.json
$P --i-understand --ceiling 700 verify gains.json --model plant.json

# idle (engine running)
$P idleff --seconds 15

# then paste everything into constants.h, rebuild, reflash, re-verify
```

**Windows (PowerShell):**

```powershell
# setup
.venv\Scripts\Activate.ps1
cmake --preset Debug; cmake --build --preset Debug
pyocd flash --target stm32c092rctx build/Debug/pmb-prod-v2.elf

function pmb { python tools\pmbtune.py @args }

# If pyocd cannot see the probe (see Section 15), start a gdbserver in another
# terminal and add --backend gdb to the function instead:
#   ST-LINK_gdbserver --swd --attach --port-number 61234
#   function pmb { python tools\pmbtune.py --backend gdb @args }

# measure (engine off)
pmb rate --seconds 10
pmb --i-understand --ceiling 300 limits
pmb params --set soft_min=23 --set soft_max=979
pmb --i-understand --ceiling 400 stiction
pmb params --set stiction_pwm=112 --set dir_hysteresis=56
pmb --i-understand --ceiling 500 ff --points 9 --dwell 2.0
pmb params --set ff_slope=362 --set ff_offset=78

# derive and prove
pmb --i-understand --ceiling 500 ident --bias 260 --amplitude 180 --out plant.json
pmb derive plant.json --out gains.json
pmb --i-understand --ceiling 700 verify gains.json --model plant.json

# idle (engine running)
pmb idleff --seconds 15
```

The `function pmb { ... @args }` line is the PowerShell equivalent of the
`P="..."` shortcut — define it once and every command in this document works by
replacing `python3 tools/pmbtune.py` with `pmb`.
