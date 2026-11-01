#!/usr/bin/env python3
"""
Offline check of the identification and gain-derivation math in pmbtune.py.

Builds a synthetic throttle body with KNOWN parameters, simulates open-loop
PWM steps through it, and asserts that `ident` recovers those parameters and
that `derive` produces a stable closed loop.  No hardware, no probe.

If this fails, every gain pmbtune prints is suspect.

    python3 tests/test_pmbtune.py
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

from pmbtune import (  # noqa: E402
    Plant, PidMirror, _arx_rows, arx_to_plant, fit_arx, fit_arx_iv, imc_pid,
    simulate_step, step_metrics,
)

# Ground truth: roughly a real electronic throttle body.
# ~8 Hz natural frequency, lightly damped by the return spring, 6 ms of
# transport delay from CAN plus driver turn-on.
TRUE_K = 1.5          # permille of travel per PWM count at DC
TRUE_WN = 2 * math.pi * 8.0
TRUE_ZETA = 0.40
TRUE_THETA = 0.006    # seconds
TS = 0.002            # 2 ms sampling, i.e. a 500 Hz TPS stream


def simulate_plant(u_sampled, ts=TS, substeps=10, noise=0.0, seed=1):
    """Integrate the continuous 2nd-order plant and sample it at `ts`.

    Deliberately NOT built as an ARX model: the fit has to recover the
    parameters of a sampled continuous system, which is the real situation.
    """
    rng = np.random.default_rng(seed)
    h = ts / substeps
    delay_steps = int(round(TRUE_THETA / h))
    x1 = x2 = 0.0
    y = np.zeros(len(u_sampled))
    hist = [0.0] * (delay_steps + 1)

    for k, u in enumerate(u_sampled):
        for _ in range(substeps):
            hist.append(float(u))
            ud = hist.pop(0)
            a = (TRUE_K * TRUE_WN**2 * ud
                 - TRUE_WN**2 * x1
                 - 2 * TRUE_ZETA * TRUE_WN * x2)
            x2 += a * h
            x1 += x2 * h
        y[k] = x1 + (rng.normal(0, noise) if noise else 0.0)
    # The firmware reports integer permille, so quantise like the real sensor.
    return np.round(y)


def square_wave(n, amplitude, half_period_samples):
    u = np.zeros(n)
    for k in range(n):
        u[k] = amplitude if (k // half_period_samples) % 2 == 0 else -amplitude
    return u


def test_identification_recovers_the_plant():
    n = 6000                      # 12 s at 2 ms
    u = square_wave(n, 250, 500)  # +/-250 counts, 1 s period
    y = simulate_plant(u, noise=0.8)

    coeffs, delay, r2 = fit_arx(u, y, max_delay=20)
    p = arx_to_plant(coeffs, delay, r2, TS)

    print(f"  sim R2        {p.r2:.5f}")
    print(f"  K        {p.K:7.3f}  (true {TRUE_K:.3f})")
    print(f"  wn       {p.wn:7.2f}  (true {TRUE_WN:.2f})")
    print(f"  zeta     {p.zeta:7.3f}  (true {TRUE_ZETA:.3f})")
    print(f"  theta    {p.theta*1000:7.2f} ms (true {TRUE_THETA*1000:.2f} ms)")

    assert p.r2 > 0.99, p.r2
    assert abs(p.K - TRUE_K) / TRUE_K < 0.10, p.K
    assert abs(p.wn - TRUE_WN) / TRUE_WN < 0.15, p.wn
    assert abs(p.zeta - TRUE_ZETA) < 0.10, p.zeta
    # Deadtime is quantised to whole samples, so one sample of error is exact.
    assert abs(p.theta - TRUE_THETA) <= 1.5 * TS, p.theta
    return p


def test_identification_is_robust_to_sensor_noise():
    """Regression test for a real defect in this tool.

    Ordinary least squares is biased by output noise, because its regressors
    include the noisy y[n-1] and y[n-2].  At only 0.8 permille of TPS noise the
    plain-LS fit reported zeta = 0.59 against a true 0.40 -- while scoring
    0.99997 on one-step R2, so nothing looked wrong.  Gains derived from that
    model would have been far too aggressive on the real plate.
    """
    for noise in (0.0, 1.0, 3.0, 5.0):
        u = square_wave(6000, 250, 500)
        y = simulate_plant(u, noise=noise, seed=7)
        coeffs, delay, r2 = fit_arx(u, y, max_delay=8)
        p = arx_to_plant(coeffs, delay, r2, TS)
        print(f"  noise {noise:3.1f} permille -> simR2 {p.r2:.5f}  "
              f"wn {p.wn:6.2f}  zeta {p.zeta:.3f}")
        assert abs(p.zeta - TRUE_ZETA) < 0.06, (noise, p.zeta)
        assert abs(p.wn - TRUE_WN) / TRUE_WN < 0.08, (noise, p.wn)


def test_iv_beats_plain_least_squares():
    """Pins down that the instrumental-variable step is actually doing work."""
    u = square_wave(6000, 250, 500)
    y = simulate_plant(u, noise=1.5, seed=3)
    d = 2
    rows = _arx_rows(u, y, d, 2 + d)
    ls_coeffs, *_ = np.linalg.lstsq(rows, y[2 + d:], rcond=None)

    p_ls = arx_to_plant(ls_coeffs, d, 0.0, TS)
    p_iv = arx_to_plant(fit_arx_iv(u, y, d), d, 0.0, TS)

    print(f"  plain LS zeta {p_ls.zeta:.3f} | IV zeta {p_iv.zeta:.3f} "
          f"| true {TRUE_ZETA:.3f}")
    assert abs(p_iv.zeta - TRUE_ZETA) < abs(p_ls.zeta - TRUE_ZETA)
    assert abs(p_iv.zeta - TRUE_ZETA) < 0.05


def test_bad_data_is_rejected():
    """A plate pinned against a stop must not silently produce a model."""
    n = 2000
    u = square_wave(n, 250, 500)
    y = np.full(n, 980.0)          # stuck: no response at all

    coeffs, delay, r2 = fit_arx(u, y, max_delay=10)
    # Either the fit reports garbage R2, or the conversion refuses it.
    try:
        p = arx_to_plant(coeffs, delay, r2, TS)
        assert p.r2 < 0.9, f"a stuck plate produced R2={p.r2}"
    except SystemExit:
        pass  # arx_to_plant rejected it outright, which is also correct
    print("  stuck-plate data rejected")


def test_derive_gives_a_stable_loop():
    p = test_identification_recovers_the_plant()

    g = imc_pid(p)
    print(f"\n  lambda {g.lam*1000:.1f} ms -> Kc={g.kc:.4f} "
          f"Ti={g.ti*1000:.1f}ms Td={g.td*1000:.1f}ms")
    print(f"  firmware: kp_m={g.kp_m} ki_m={g.ki_m} kd_m={g.kd_m}")

    assert g.kp_m > 0 and g.ki_m > 0
    assert g.kd_m >= 0

    sim = simulate_step(p, g, 200, 600, 1.5)
    m = step_metrics(sim["t"], sim["y"], 200, 600)
    print(f"  predicted rise {m['rise_10_90_ms']:.0f} ms, "
          f"overshoot {m['overshoot_pct']:.1f}%, "
          f"settle {m['settle_2pct_ms']:.0f} ms")

    # Stability: it must actually arrive and stay there.
    assert abs(sim["y"][-1] - 600) < 20, sim["y"][-1]
    assert m["overshoot_pct"] < 50, m["overshoot_pct"]
    assert np.all(np.isfinite(sim["y"]))
    # And it must never command past the soft limits.
    assert sim["y"].min() >= 20 - 1 and sim["y"].max() <= 980 + 1


def test_lambda_trades_speed_for_overshoot():
    """The single tuning knob has to behave monotonically, or it is not a knob."""
    p = test_identification_recovers_the_plant()
    rises = []
    for lam in (0.02, 0.05, 0.12):
        g = imc_pid(p, lam)
        sim = simulate_step(p, g, 200, 600, 2.0)
        m = step_metrics(sim["t"], sim["y"], 200, 600)
        rises.append(m["rise_10_90_ms"])
        print(f"  lambda {lam*1000:5.0f} ms -> rise {m['rise_10_90_ms']:6.0f} ms, "
              f"overshoot {m['overshoot_pct']:5.1f}%")
    assert rises[0] < rises[-1], rises


def test_pid_mirror_matches_the_c_semantics():
    """Spot-check the Python mirror against the same cases as test_pid.c."""
    # No integral dead zone: e = 1 still accumulates.
    pid = PidMirror(0, 35, 0, -1000, 1000, 1000)
    for _ in range(100):
        pid.step(1, 0, 1)
    assert pid.i_acc == 3500, pid.i_acc

    # Anti-windup: saturated and pushing further in -> integral frozen.
    pid = PidMirror(1000, 100000, 0, -100, 100, 1000)
    for _ in range(100):
        assert pid.step(1000, 0, 1) == 100
    assert pid.i_acc == 0

    # No derivative kick on a setpoint step.
    pid = PidMirror(0, 0, 1000, -1000, 1000, 1000)
    pid.pv_prev, pid.primed = 500, True
    assert pid.step(600, 500, 1) == 0
    assert pid.step(600, 510, 1) == -10

    # Truncation toward zero, matching C integer division.
    assert PidMirror._trunc(-7, 2) == -3
    assert PidMirror._trunc(7, 2) == 3
    assert PidMirror._trunc(7, -2) == -3
    print("  mirror matches pid.c on all shared cases")


if __name__ == "__main__":
    for fn in (test_identification_recovers_the_plant,
               test_identification_is_robust_to_sensor_noise,
               test_iv_beats_plain_least_squares,
               test_bad_data_is_rejected,
               test_derive_gives_a_stable_loop,
               test_lambda_trades_speed_for_overshoot,
               test_pid_mirror_matches_the_c_semantics):
        print(f"\n{fn.__name__}")
        fn()
    print("\nall pmbtune math checks passed")
