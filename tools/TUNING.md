# Dialling in the throttle body

End-to-end procedure: build the Debug firmware, attach an ST-Link, and use
`pmbtune.py` to measure the hardware and derive the control gains from that
measurement.

Nothing here asks you to guess a number. Every value lands in the `MEASURE ME`
block of `Core/Inc/constants.h`, and every one has a command that measures it
and prints the line to paste.

**Read the safety model (§10) before you command the motor for the first time.**

---

## Contents

| | Step | Engine | Needs |
|---|---|---|---|
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

---

## 2. One-time setup

```bash
cd ~/Projects/FSAE/pmb-prod-v2

python3 -m venv .venv
source .venv/bin/activate
pip install -r tools/requirements.txt
```

pyOCD needs a device description for the STM32C0 before it knows the target:

```bash
pyocd pack find stm32c092
pyocd pack install stm32c092rctx
```

Check the probe is seen:

```bash
pyocd list
```

You should get a line naming your ST-Link. If not, it is a USB permissions
problem on Linux — see §15.

Finally, confirm the host-side maths is sound before you trust anything it
prints:

```bash
bash tests/run.sh
```

Both suites must say "passed". These are regression tests for real defects
(including a noise-bias bug that made the identifier report 50%-wrong damping),
so if they fail, the gains this tool derives are not trustworthy.

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

λ is the single tuning knob — larger is slower and more robust. The tool sweeps
it and picks the **fastest λ that keeps predicted overshoot within budget**,
simulating each candidate through a bit-for-bit mirror of the firmware's own
fixed-point controller, so the prediction includes the integer truncation,
clamps and deadtime.

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

This reads only — it commands nothing.

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
python3 tools/pmbtune.py params | tee tuned-params.txt
```

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
| `no debug probe found` | Probe not enumerated. `pyocd list`. On Linux add a udev rule: `SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"` in `/etc/udev/rules.d/99-stlink.rules`, then `sudo udevadm control --reload && sudo udevadm trigger`, and replug. |
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

---

## 16. Command reference

Global flags come **before** the subcommand:

```
--elf PATH            built ELF for symbol lookup (default build/Debug/...)
--target-name NAME    pyocd target type (default stm32c092rctx)
--frequency HZ        SWD clock (default 4000000)
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

```bash
# setup
source .venv/bin/activate
cmake --preset Debug && cmake --build --preset Debug
pyocd flash --target stm32c092rctx build/Debug/pmb-prod-v2.elf

# measure (engine off)
P="python3 tools/pmbtune.py"
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
