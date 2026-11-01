#!/usr/bin/env python3
"""
pmbtune -- measure the throttle body, derive gains from the measurement, prove
them before they go in the car.

Transport is SWD only.  The firmware writes 16-byte samples into a RAM ring at
the control rate; this reads that ring in bulk.  Nothing is framed, nothing is
parsed, and it works with any ST-Link.  Parameters are plain RAM globals, so
changing a gain is a memory write -- there is no command protocol.

Addresses come out of the ELF symbol table, so a relink cannot desynchronise
the host from the target.

Two ways to reach the probe:

    --backend pyocd   (default) direct USB. Needs a WinUSB-bound driver on
                      Windows, which ST's own driver is not.
    --backend gdb     via a running ST-LINK_gdbserver or OpenOCD on TCP. That
                      server owns the probe, so on Windows it uses ST's driver
                      and nothing needs rebinding:
                        ST-LINK_gdbserver --swd --attach --port-number 61234
                        pmbtune.py --backend gdb ...

    pmbtune.py rate                  what rate does TPS/rpm actually arrive at
    pmbtune.py limits                measure the mechanical stops
    pmbtune.py stiction              measure the breakaway duty
    pmbtune.py ff                    fit the return-spring feedforward
    pmbtune.py idleff                measure the idle base opening (engine on)
    pmbtune.py ident                 open-loop steps -> plant model
    pmbtune.py derive  model.json    model -> gains, with a predicted response
    pmbtune.py verify  gains.json    closed-loop steps -> measured vs predicted
    pmbtune.py log     out.csv       stream telemetry
    pmbtune.py monitor               live one-line view
    pmbtune.py params [--set k=v]    read / write the live parameter block

SAFETY
    Everything that drives the throttle requires --i-understand and holds a
    --ceiling on PWM.  The firmware auto-disables injection if this script
    stops bumping its heartbeat for 250 ms, so a crash here releases the
    throttle rather than leaving it commanded.

    `ident` is intended for ENGINE OFF.  It commands open-loop PWM.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# --------------------------------------------------------------------------
# target layout -- must match Core/Inc/tuning.h
# --------------------------------------------------------------------------

PARAM_FIELDS = [
    "kp_fwd", "ki_fwd", "kd_fwd",
    "kp_rev", "ki_rev", "kd_rev",
    "kp_rev_low", "ki_rev_low",
    "i_max", "deadband", "d_lpf_shift",
    "ff_slope", "ff_offset",
    "stiction_pwm", "dir_hysteresis",
    "soft_min", "soft_max",
    "idle_kp", "idle_ki", "idle_kd", "idle_kp_save", "idle_i_max",
    "idle_ff_hot", "idle_ff_cold",
    "blip_target_low", "blip_target_high",
]

TUNE_FIELDS = [
    "magic", "version", "sample_count", "sample_size", "write_idx",
    "inject_key", "heartbeat", "inject_mode", "inject_value", "pwm_ceiling",
]

TUNE_MAGIC = 0x544D4250
INJECT_KEY = 0x5A33C0DE
INJECT_OFF, INJECT_POSITION, INJECT_PWM = 0, 1, 2

SAMPLE_FMT = "<IhhhhHh"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
assert SAMPLE_SIZE == 16, SAMPLE_SIZE
SAMPLE_COLS = ["t_ms", "sp", "pv", "pv2", "u", "rpm", "pedal"]


# --------------------------------------------------------------------------
# transport
# --------------------------------------------------------------------------

def elf_symbols(elf_path: Path) -> dict[str, tuple[int, int]]:
    try:
        from elftools.elf.elffile import ELFFile
    except ImportError:
        raise SystemExit(
            "pyelftools is required to resolve symbol addresses.\n"
            "  pip install -r tools/requirements.txt"
        ) from None

    try:
        with open(elf_path, "rb") as fh:
            elf = ELFFile(fh)
            tab = elf.get_section_by_name(".symtab")
            if tab is None:
                raise SystemExit(
                    f"{elf_path} has no symbol table (built with -g0?)"
                )
            out = {}
            for sym in tab.iter_symbols():
                if sym.name:
                    out[sym.name] = (sym["st_value"], sym["st_size"])
            return out
    except SystemExit:
        raise
    except OSError as exc:
        raise SystemExit(f"could not read {elf_path}: {exc}") from None
    except Exception as exc:
        raise SystemExit(
            f"{elf_path} does not look like an ELF file ({exc}).\n"
            "Point --elf at build/Debug/pmb-prod-v2.elf."
        ) from None


def find_elf(explicit: str | None) -> Path:
    if explicit:
        p = Path(explicit)
        if not p.exists():
            raise SystemExit(f"no such ELF: {p}")
        return p
    root = Path(__file__).resolve().parent.parent
    for cand in ("build/Debug/pmb-prod-v2.elf", "build/Release/pmb-prod-v2.elf"):
        p = root / cand
        if p.exists():
            return p
    raise SystemExit("could not find a built ELF; pass --elf")


# --------------------------------------------------------------------------
# backends
#
# The whole tool needs exactly two operations from a debug probe: read a block
# of bytes, and write one 32-bit word -- both while the target keeps running.
# Everything else is built on those, so supporting a second probe stack is a
# small class rather than a rewrite.
#
#   pyocd  default. Cross-platform, but on Windows it needs a WinUSB-bound
#          driver, which ST's own driver is not.
#   gdb    talks to an ST-LINK_gdbserver (or OpenOCD) over TCP. That server
#          uses ST's driver, so it is the Windows answer when pyocd cannot see
#          the probe and you do not want to rebind drivers with Zadig.
# --------------------------------------------------------------------------

class PyocdBackend:
    """Attaches without halting -- the car keeps running."""

    name = "pyocd"

    def __init__(self, target_name: str, frequency: int):
        from pyocd.core.helpers import ConnectHelper

        self.session = ConnectHelper.session_with_chosen_probe(
            options={
                "target_override": target_name,
                # attach, do not halt: this runs against a live engine
                "connect_mode": "attach",
                "resume_on_disconnect": True,
                "frequency": frequency,
            },
            blocking=False,
        )
        if self.session is None:
            raise SystemExit(
                "no debug probe found.\n"
                "On Windows this usually means the driver is not WinUSB-bound; "
                "see TUNING.md section 15.\nYou can instead run an "
                "ST-LINK_gdbserver and pass --backend gdb, which works with "
                "ST's own driver."
            )
        self.session.open()
        self._t = self.session.target

    def read_block(self, addr: int, nbytes: int) -> bytes:
        return bytes(self._t.read_memory_block8(addr, nbytes))

    def write_u32(self, addr: int, value: int):
        self._t.write_memory(addr, value & 0xFFFFFFFF)

    def close(self):
        self.session.close()


def rsp_frame(payload: str) -> bytes:
    """Wrap a GDB remote-serial-protocol payload: $<data>#<checksum>."""
    csum = sum(payload.encode()) & 0xFF
    return f"${payload}#{csum:02x}".encode()


def rsp_checksum_ok(payload: str, csum_hex: str) -> bool:
    return (sum(payload.encode()) & 0xFF) == int(csum_hex, 16)


class GdbBackend:
    """Memory access via the GDB remote protocol over TCP.

    Point this at a gdbserver that is already attached to the running target:

        ST-LINK_gdbserver --swd --attach --port-number 61234
        # or: openocd -f interface/stlink.cfg -f target/stm32c0x.cfg

    The server owns the probe, so on Windows it uses ST's driver and no
    rebinding is needed.  Reads and writes go over the AHB access port and do
    not halt the core, which is what lets this run against a live engine --
    but see the --attach note: if the server halts on connect, nothing here
    will un-halt it for you.

    Only `m` (read memory) and `M` (write memory) are used.  No run control is
    issued at all, deliberately: this tool must never be able to stop an engine
    that is running.
    """

    name = "gdb"

    def __init__(self, host: str, port: int, timeout: float = 5.0):
        import socket

        try:
            self.sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as exc:
            raise SystemExit(
                f"could not connect to a gdbserver at {host}:{port} ({exc}).\n"
                "Start one first, e.g.:\n"
                "  ST-LINK_gdbserver --swd --attach --port-number 61234\n"
                "Check the exact flags with `ST-LINK_gdbserver --help`; it "
                "ships with STM32CubeCLT / CubeIDE."
            ) from None
        self.sock.settimeout(timeout)
        self._buf = b""

        # Negotiate, mostly to learn the maximum packet size so block reads can
        # be chunked to fit.  Hex encoding doubles the payload, so the usable
        # byte count is less than half of PacketSize.
        self.max_read = 256
        try:
            reply = self._command("qSupported:multiprocess+")
            for feat in reply.split(";"):
                if feat.startswith("PacketSize="):
                    size = int(feat.split("=", 1)[1], 16)
                    self.max_read = max(64, min(1024, (size - 32) // 2))
        except Exception:
            pass  # a server that will not negotiate still answers m/M

    # ---- RSP plumbing ----
    def _recv_packet(self) -> str:
        while True:
            start = self._buf.find(b"$")
            end = self._buf.find(b"#", start + 1)
            if start >= 0 and end >= 0 and len(self._buf) >= end + 3:
                payload = self._buf[start + 1:end].decode(errors="replace")
                csum = self._buf[end + 1:end + 3].decode()
                self._buf = self._buf[end + 3:]
                self.sock.sendall(b"+")
                if not rsp_checksum_ok(payload, csum):
                    raise SystemExit("gdbserver packet failed its checksum")
                return payload
            chunk = self.sock.recv(4096)
            if not chunk:
                raise SystemExit("gdbserver closed the connection")
            self._buf += chunk

    def _command(self, payload: str) -> str:
        self.sock.sendall(rsp_frame(payload))
        # _recv_packet scans for '$', so any '+'/'-' acks in front of the reply
        # are skipped on the way.  An empty reply means "unsupported command"
        # and must be returned, not waited past -- doing that deadlocks.
        return self._recv_packet()

    def read_block(self, addr: int, nbytes: int) -> bytes:
        out = bytearray()
        remaining = nbytes
        cursor = addr
        while remaining > 0:
            run = min(remaining, self.max_read)
            reply = self._command(f"m{cursor:x},{run:x}")
            if reply.startswith("E") and len(reply) <= 3:
                raise SystemExit(
                    f"gdbserver refused a read at 0x{cursor:08X} ({reply}). "
                    "Is it attached to the target?"
                )
            data = bytes.fromhex(reply)
            if not data:
                raise SystemExit(f"gdbserver returned no data at 0x{cursor:08X}")
            out += data
            cursor += len(data)
            remaining -= len(data)
        return bytes(out)

    def write_u32(self, addr: int, value: int):
        word = struct.pack("<I", value & 0xFFFFFFFF)
        reply = self._command(f"M{addr:x},4:{word.hex()}")
        if reply != "OK":
            raise SystemExit(
                f"gdbserver refused a write at 0x{addr:08X} ({reply})"
            )

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def make_backend(args):
    if args.backend == "gdb":
        return GdbBackend(args.gdb_host, args.gdb_port)
    return PyocdBackend(args.target_name, args.frequency)


class Target:
    """Live connection to the firmware's parameter block and telemetry ring.

    Takes a factory rather than a live backend so the ELF is validated before
    any hardware is touched, and so a failure during the post-connect handshake
    still releases the probe.
    """

    def __init__(self, elf: Path, backend_factory):
        self.syms = elf_symbols(elf)
        for needed in ("pmb_params", "pmb_tune", "pmb_log"):
            if needed not in self.syms:
                raise SystemExit(
                    f"symbol '{needed}' missing from {elf}.\n"
                    "Build the Debug preset: PMB_TUNING is Debug-only."
                )

        self.params_addr = self.syms["pmb_params"][0]
        self.tune_addr = self.syms["pmb_tune"][0]
        self.log_addr = self.syms["pmb_log"][0]

        self.backend = backend_factory()
        try:
            tune = self.read_tune()
            if tune["magic"] != TUNE_MAGIC:
                raise SystemExit(
                    f"pmb_tune.magic is 0x{tune['magic']:08X}, expected "
                    f"0x{TUNE_MAGIC:08X}. Stale ELF, or a Release build is "
                    "flashed."
                )
            if tune["sample_size"] != SAMPLE_SIZE:
                raise SystemExit(
                    f"target sample is {tune['sample_size']} bytes, host "
                    f"expects {SAMPLE_SIZE}: tuning.h and pmbtune.py are out "
                    "of sync"
                )
            self.n_samples = tune["sample_count"]
            self._cursor = tune["write_idx"]
            self._hb = 0
        except BaseException:
            self.backend.close()
            raise

    def close(self):
        try:
            self.inject_off()
        finally:
            self.backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- raw ----
    def _read_words(self, addr: int, count: int) -> list[int]:
        raw = self.backend.read_block(addr, count * 4)
        return list(struct.unpack("<" + "I" * count, raw))

    def _write_word(self, addr: int, value: int):
        self.backend.write_u32(addr, value)

    # ---- params ----
    def read_params(self) -> dict[str, int]:
        words = self._read_words(self.params_addr, len(PARAM_FIELDS))
        signed = [w - (1 << 32) if w >= (1 << 31) else w for w in words]
        return dict(zip(PARAM_FIELDS, signed))

    def write_param(self, name: str, value: int):
        if name not in PARAM_FIELDS:
            raise SystemExit(f"unknown parameter '{name}'")
        self._write_word(self.params_addr + 4 * PARAM_FIELDS.index(name), int(value))

    def write_params(self, values: dict[str, int]):
        for k, v in values.items():
            self.write_param(k, v)

    # ---- control block ----
    def read_tune(self) -> dict[str, int]:
        words = self._read_words(self.tune_addr, len(TUNE_FIELDS))
        out = dict(zip(TUNE_FIELDS, words))
        for k in ("inject_mode", "inject_value", "pwm_ceiling"):
            if out[k] >= (1 << 31):
                out[k] -= 1 << 32
        return out

    def _tune_addr_of(self, name: str) -> int:
        return self.tune_addr + 4 * TUNE_FIELDS.index(name)

    def heartbeat(self):
        """Must be called more often than every 250 ms while injecting."""
        self._hb = (self._hb + 1) & 0xFFFFFFFF
        self._write_word(self._tune_addr_of("heartbeat"), self._hb)

    def inject(self, mode: int, value: int, ceiling: int):
        self._write_word(self._tune_addr_of("pwm_ceiling"), ceiling)
        self._write_word(self._tune_addr_of("inject_value"), value)
        self._write_word(self._tune_addr_of("inject_mode"), mode)
        self.heartbeat()
        self._write_word(self._tune_addr_of("inject_key"), INJECT_KEY)

    def inject_off(self):
        try:
            self._write_word(self._tune_addr_of("inject_mode"), INJECT_OFF)
            self._write_word(self._tune_addr_of("inject_key"), 0)
            self._write_word(self._tune_addr_of("pwm_ceiling"), 0)
        except Exception:
            pass  # closing down; the heartbeat timeout is the real backstop

    # ---- telemetry ring ----
    def read_new(self) -> np.ndarray:
        """Samples written since the last call, oldest first.

        write_idx is published after the payload, so a racing read can never
        return a half-written sample.  Anything older than one ring length is
        gone; the caller is told by the gap in t_ms.
        """
        widx = self.read_tune()["write_idx"]
        n = self.n_samples
        start = max(self._cursor, widx - n)
        if widx <= start:
            return np.zeros((0, len(SAMPLE_COLS)))

        rows = []
        i = start
        while i < widx:
            slot = i % n
            run = min(widx - i, n - slot)
            raw = self.backend.read_block(
                self.log_addr + slot * SAMPLE_SIZE, run * SAMPLE_SIZE)
            for k in range(run):
                rows.append(struct.unpack_from(SAMPLE_FMT, raw, k * SAMPLE_SIZE))
            i += run
        self._cursor = widx
        return np.array(rows, dtype=float)

    def collect(self, seconds: float, poll: float = 0.02,
                on_poll=None) -> np.ndarray:
        """Gather telemetry for `seconds`, calling on_poll(elapsed) each poll.

        on_poll is where injection setpoints get stepped, so the heartbeat and
        the sampling share one thread and pyocd is never touched concurrently.
        """
        out = []
        t0 = time.monotonic()
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= seconds:
                break
            if on_poll is not None:
                on_poll(elapsed)
            chunk = self.read_new()
            if len(chunk):
                out.append(chunk)
            time.sleep(poll)
        out.append(self.read_new())
        out = [c for c in out if len(c)]
        return np.vstack(out) if out else np.zeros((0, len(SAMPLE_COLS)))


def frame(rows: np.ndarray) -> dict[str, np.ndarray]:
    """Column view, de-duplicated and sorted by target timestamp."""
    if not len(rows):
        return {c: np.zeros(0) for c in SAMPLE_COLS}
    rows = rows[np.argsort(rows[:, 0], kind="stable")]
    _, keep = np.unique(rows[:, 0], return_index=True)
    rows = rows[np.sort(keep)]
    return {c: rows[:, i] for i, c in enumerate(SAMPLE_COLS)}


# --------------------------------------------------------------------------
# identification
# --------------------------------------------------------------------------

@dataclass
class Plant:
    """Second-order continuous model of plate position vs PWM.

        G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * exp(-theta*s)

    K     permille of travel per PWM count at DC
    wn    rad/s
    zeta  dimensionless
    theta deadtime, seconds
    ts    identification sample period, seconds
    arx   discrete coefficients [a1, a2, b1, b2]
    r2    FREE-RUN simulation fit, not one-step prediction. See simulation_r2.
    """
    K: float
    wn: float
    zeta: float
    theta: float
    ts: float
    arx: list[float] = field(default_factory=list)
    r2: float = 0.0

    def to_json(self) -> str:
        return json.dumps(self.__dict__, indent=2)

    @staticmethod
    def from_json(path: Path) -> "Plant":
        return Plant(**json.loads(Path(path).read_text()))


def _arx_rows(u: np.ndarray, y: np.ndarray, d: int, n0: int) -> np.ndarray:
    return np.column_stack([
        y[n0 - 1:-1],
        y[n0 - 2:-2],
        u[n0 - 1 - d:len(y) - 1 - d],
        u[n0 - 2 - d:len(y) - 2 - d],
    ])


def simulate_arx(coeffs, u: np.ndarray, seed, delay: int):
    """Free-run the discrete model from the input alone. None if it diverges."""
    a1, a2, b1, b2 = coeffs
    n = len(u)
    y = np.zeros(n)
    y[0], y[1] = float(seed[0]), float(seed[1])
    for k in range(2, n):
        u1 = u[k - 1 - delay] if k - 1 - delay >= 0 else 0.0
        u2 = u[k - 2 - delay] if k - 2 - delay >= 0 else 0.0
        y[k] = a1 * y[k - 1] + a2 * y[k - 2] + b1 * u1 + b2 * u2
        if not math.isfinite(y[k]) or abs(y[k]) > 1e7:
            return None
    return y


def simulation_r2(coeffs, u: np.ndarray, y: np.ndarray, delay: int) -> float:
    """Fit quality of a FREE-RUN simulation, driven by the input only.

    This is the honest metric and the one used as the quality gate.  The usual
    one-step-ahead R2 is near 1.0 for almost any model, because y[n-1] carries
    the prediction -- a badly wrong model scored 0.99997 one-step while getting
    the damping ratio off by 50%.
    """
    sim = simulate_arx(coeffs, u, y, delay)
    if sim is None:
        return -math.inf
    denom = np.sum((y - y.mean()) ** 2)
    if denom <= 0:
        return -math.inf
    return float(1.0 - np.sum((y - sim) ** 2) / denom)


def fit_arx_iv(u: np.ndarray, y: np.ndarray, d: int, iters: int = 4):
    """Iterative instrumental-variable fit of

        y[n] = a1*y[n-1] + a2*y[n-2] + b1*u[n-1-d] + b2*u[n-2-d]

    Plain least squares is BIASED here, badly.  Its regressors include the
    noisy y[n-1] and y[n-2], so the regressors are correlated with the
    residual and the estimate is pulled toward excess damping.  With a TPS
    stream carrying only ~1 permille of noise this shifted an identified
    damping ratio from 0.40 to 0.59 -- and the gains derived from that would
    have been far too aggressive for the real plant.

    The fix is textbook (Soderstrom & Stoica): replace the noisy regressors
    with instruments that correlate with the true output but not with its
    noise.  The simulated output of the previous iterate is exactly that,
    since it is driven by the input alone.  Converges in 2-3 passes.
    """
    n0 = 2 + d
    rows = _arx_rows(u, y, d, n0)
    target = y[n0:]

    theta, *_ = np.linalg.lstsq(rows, target, rcond=None)

    for _ in range(iters):
        sim = simulate_arx(theta, u, y, d)
        if sim is None:
            break
        instruments = _arx_rows(u, sim, d, n0)
        try:
            nxt = np.linalg.solve(instruments.T @ rows, instruments.T @ target)
        except np.linalg.LinAlgError:
            break
        if not np.all(np.isfinite(nxt)):
            break
        theta = nxt

    return theta


def fit_arx(u: np.ndarray, y: np.ndarray, max_delay: int = 20):
    """Identify the plant, searching over integer deadtime.

    Returns (coeffs, delay, simulation_r2).  The deadtime search matters: a
    throttle body has real transport delay (CAN latency plus driver turn-on),
    and absorbing it into the dynamics produces a model that lies about the
    achievable bandwidth.  Candidates are scored on free-run simulation fit,
    not one-step prediction.
    """
    best = None
    for d in range(max_delay + 1):
        if len(y) <= 2 + d + 10:
            break
        coeffs = fit_arx_iv(u, y, d)
        r2 = simulation_r2(coeffs, u, y, d)
        if best is None or r2 > best[2]:
            best = (coeffs, d, r2)
    if best is None:
        raise SystemExit("not enough data to fit a model")
    return best


def arx_to_plant(coeffs, delay: int, r2: float, ts: float) -> Plant:
    a1, a2, b1, b2 = coeffs

    dc_den = 1.0 - a1 - a2
    if abs(dc_den) < 1e-9:
        raise SystemExit(
            "identified model has (near) unity DC pole -- the plate was "
            "probably against a stop for the whole test"
        )
    K = (b1 + b2) / dc_den

    # discrete poles -> continuous
    roots = np.roots([1.0, -a1, -a2]).astype(complex)
    if np.any(np.abs(roots) < 1e-12):
        raise SystemExit("degenerate pole in the identified model")
    s = np.log(roots) / ts
    s = s[np.argsort(-np.abs(s.imag))]  # prefer the oscillatory pair
    wn = float(abs(s[0]))
    zeta = float(-s[0].real / wn) if wn > 0 else 1.0

    if wn <= 0 or not math.isfinite(wn):
        raise SystemExit("identification produced a non-physical model")

    return Plant(K=float(K), wn=wn, zeta=zeta, theta=delay * ts, ts=ts,
                 arx=[float(c) for c in coeffs], r2=float(r2))


# --------------------------------------------------------------------------
# gain derivation + simulation
# --------------------------------------------------------------------------

@dataclass
class Gains:
    """Firmware-scale gains (x1000), plus the continuous values they encode."""
    kp_m: int
    ki_m: int
    kd_m: int
    kc: float
    ti: float
    td: float
    lam: float

    def to_json(self) -> str:
        return json.dumps(self.__dict__, indent=2)


def imc_pid(p: Plant, lam: float | None = None) -> Gains:
    """IMC-PID for a second-order process (Rivera/Morari).

    With  G(s) = K / (tau^2 s^2 + 2 zeta tau s + 1) * exp(-theta s),
    tau = 1/wn, and a closed-loop filter time constant lambda:

        Kc = 2*zeta*tau / (K * (lambda + theta))
        Ti = 2*zeta*tau
        Td = tau / (2*zeta)

    lambda is the single knob: larger is slower and more robust.  The default
    is the usual robustness floor -- no faster than the deadtime allows, and no
    faster than half the plant's own time constant.
    """
    tau = 1.0 / p.wn
    if lam is None:
        lam = max(2.0 * p.theta, 0.5 * tau)
    if lam <= 0:
        raise SystemExit("lambda must be positive")
    if p.K == 0:
        raise SystemExit("identified DC gain is zero; cannot derive gains")

    kc = 2.0 * p.zeta * tau / (p.K * (lam + p.theta))
    ti = 2.0 * p.zeta * tau
    td = tau / (2.0 * p.zeta)

    ki = kc / ti            # output / (permille * second)
    kd = kc * td            # output * second / permille

    return Gains(
        kp_m=int(round(1000.0 * kc)),
        ki_m=int(round(1000.0 * ki)),
        kd_m=int(round(1e6 * kd)),   # firmware Kd is per millisecond, see pid.h
        kc=kc, ti=ti, td=td, lam=lam,
    )


class PidMirror:
    """Bit-for-bit mirror of pid_step() in Core/Src/pid.c.

    Integer division truncates toward zero in both C and Python's int(),
    which is what // does NOT do for negatives -- hence the explicit trunc
    helper.  Keep this in step with pid.c or the predictions mean nothing.
    """

    def __init__(self, kp_m, ki_m, kd_m, out_min, out_max, i_max,
                 deadband=0, d_lpf_shift=0):
        self.kp_m, self.ki_m, self.kd_m = kp_m, ki_m, kd_m
        self.out_min, self.out_max = out_min, out_max
        self.i_max, self.deadband, self.d_lpf_shift = i_max, deadband, d_lpf_shift
        self.i_acc = 0
        self.d_filt = 0
        self.pv_prev = 0
        self.primed = False

    @staticmethod
    def _trunc(a, b):
        q = abs(a) // abs(b)
        return q if (a >= 0) == (b > 0) else -q

    def step(self, sp, pv, dt_ms, ff=0):
        dt_ms = max(1, int(dt_ms))
        e = int(sp) - int(pv)

        rate = self._trunc((pv - self.pv_prev) * 1000, dt_ms) if self.primed else 0
        self.pv_prev = pv
        self.primed = True

        d_target = -self._trunc(self.kd_m * rate, 1000)
        if self.d_lpf_shift == 0:
            self.d_filt = d_target
        else:
            self.d_filt += self._trunc(d_target - self.d_filt, 1 << self.d_lpf_shift)

        if -self.deadband <= e <= self.deadband:
            return max(self.out_min,
                       min(self.out_max, ff + self._trunc(self.i_acc, 1000000)))

        i_delta = self.ki_m * e * dt_ms
        i_lim = min(self.i_max * 1000000, 2**31 - 1)
        i_new = max(-i_lim, min(i_lim, self.i_acc + i_delta))

        base_m = ff * 1000 + self.kp_m * e + self.d_filt
        u_m = base_m + self._trunc(i_new, 1000)
        if (u_m > self.out_max * 1000 and i_delta > 0) or \
           (u_m < self.out_min * 1000 and i_delta < 0):
            i_new = self.i_acc
            u_m = base_m + self._trunc(i_new, 1000)
        self.i_acc = i_new

        return max(self.out_min, min(self.out_max, self._trunc(u_m, 1000)))


def simulate_step(p: Plant, g: Gains, sp_from: int, sp_to: int,
                  seconds: float, pwm_max: int = 1000,
                  soft=(20, 980)) -> dict[str, np.ndarray]:
    """Closed-loop step response of the identified model under the firmware's
    own fixed-point controller.  Used to predict before committing gains.

    This simulates the ARX model directly rather than the continuous fit, so
    the prediction includes whatever the real plant actually did, including the
    deadtime."""
    a1, a2, b1, b2 = p.arx
    dt_ms = max(1, int(round(p.ts * 1000)))
    n = int(seconds / p.ts)
    delay = int(round(p.theta / p.ts))

    pid = PidMirror(g.kp_m, g.ki_m, g.kd_m, -pwm_max, pwm_max, pwm_max)
    y = np.full(n, float(sp_from))
    u = np.zeros(n)
    sp = np.full(n, float(sp_to))
    sp[0] = sp_from

    for k in range(2, n):
        cmd = pid.step(sp[k], int(round(y[k - 1])), dt_ms)
        u[k] = max(-pwm_max, min(pwm_max, cmd))
        ud1 = u[k - 1 - delay] if k - 1 - delay >= 0 else 0.0
        ud2 = u[k - 2 - delay] if k - 2 - delay >= 0 else 0.0
        y[k] = a1 * y[k - 1] + a2 * y[k - 2] + b1 * ud1 + b2 * ud2
        y[k] = max(soft[0], min(soft[1], y[k]))

    return {"t": np.arange(n) * p.ts, "y": y, "u": u, "sp": sp}


def step_metrics(t: np.ndarray, y: np.ndarray, y0: float, y1: float) -> dict:
    """Rise time (10-90%), overshoot, 2% settling time, and IAE."""
    span = y1 - y0
    if abs(span) < 1e-9:
        return {}

    def cross(frac):
        lvl = y0 + frac * span
        hit = np.where((y - lvl) * np.sign(span) >= 0)[0]
        return float(t[hit[0]]) if len(hit) else float("nan")

    peak = float(np.max(y) if span > 0 else np.min(y))
    overshoot = (peak - y1) / span * 100.0

    tol = 0.02 * abs(span)
    settle = float("nan")
    for i in range(len(y) - 1, -1, -1):
        if abs(y[i] - y1) > tol:
            settle = float(t[i + 1]) if i + 1 < len(t) else float("nan")
            break
    else:
        settle = 0.0

    return {
        "rise_10_90_ms": (cross(0.9) - cross(0.1)) * 1000.0,
        "overshoot_pct": overshoot,
        "settle_2pct_ms": settle * 1000.0,
        "iae": float(np.sum(np.abs(y1 - y)) * (t[1] - t[0]) if len(t) > 1 else 0),
    }


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------

def require_consent(args, what: str):
    if not args.i_understand:
        raise SystemExit(
            f"'{what}' drives the throttle motor.\n"
            "Re-run with --i-understand once the area is clear.\n"
            "ident/stiction/ff/limits are ENGINE OFF procedures."
        )


def cmd_rate(target: Target, args):
    print(f"sampling {args.seconds:.0f}s of telemetry (no injection)...")
    data = frame(target.collect(args.seconds))
    t = data["t_ms"]
    if len(t) < 10:
        raise SystemExit("almost no samples -- is the control loop running?")

    dt = np.diff(t)
    print(f"  control loop samples : {len(t)}")
    print(f"  period  median/mean  : {np.median(dt):.2f} / {dt.mean():.2f} ms")
    print(f"  period  min/max      : {dt.min():.0f} / {dt.max():.0f} ms")
    print(f"  effective rate       : {1000.0 / np.mean(dt):.0f} Hz")

    for name in ("pv", "rpm"):
        v = data[name]
        chg = np.diff(t)[np.diff(v) != 0]
        if len(chg):
            print(f"  {name:4s} changes every  : {np.median(chg):.1f} ms "
                  f"(~{1000.0 / np.median(chg):.0f} Hz of real new data)")
        else:
            print(f"  {name:4s}: never changed during the window")

    print("\nSet PID_RPM_DT and sanity-check THROTTLE_CONTROL_TIMEOUT_MS")
    print("against the real rpm/TPS rates above.")


def _ramp_to_motion(target: Target, args, opening: bool):
    """Creep PWM up until the plate moves. Returns (breakaway_pwm, start, end)."""
    pwm = 0
    start = None
    moved_at = None
    last = None
    step = max(1, args.ceiling // 60)

    def on_poll(_elapsed):
        nonlocal pwm
        target.inject(INJECT_PWM, pwm if opening else -pwm, args.ceiling)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds and pwm <= args.ceiling:
        on_poll(0)
        time.sleep(0.05)
        d = frame(target.read_new())
        if len(d["pv"]):
            pos = float(d["pv"][-1])
            if start is None:
                start = pos
            if moved_at is None and abs(pos - start) > args.motion:
                moved_at = pwm
            last = pos
        pwm += step

    target.inject(INJECT_PWM, 0, args.ceiling)
    return moved_at, start, last


def cmd_stiction(target: Target, args):
    require_consent(args, "stiction")
    print("ENGINE OFF. Ramping duty until the plate breaks free...")
    try:
        out = {}
        for opening in (True, False):
            pwm, start, end = _ramp_to_motion(target, args, opening)
            label = "opening" if opening else "closing"
            if pwm is None:
                print(f"  {label}: no motion up to ceiling {args.ceiling}")
            else:
                print(f"  {label}: breakaway at {pwm} counts "
                      f"({start:.0f} -> {end:.0f} permille)")
                out[label] = pwm
            time.sleep(0.5)
    finally:
        target.inject_off()

    if out:
        rec = max(out.values())
        print(f"\n  #define THROTTLE_STICTION_PWM   {rec}")
        print(f"  #define THROTTLE_DIR_HYSTERESIS {max(1, rec // 2)}")
        print("\nThe hysteresis is half the breakaway duty: below that a "
              "reversal cannot move the plate anyway, so refusing it only "
              "saves the bridge from chattering.")


def cmd_limits(target: Target, args):
    require_consent(args, "limits")
    print("ENGINE OFF. Creeping to each mechanical stop under a PWM ceiling...")
    seen_min, seen_max = None, None
    try:
        for opening, name in ((False, "closed"), (True, "open")):
            drive = args.ceiling if opening else -args.ceiling

            def on_poll(_e):
                target.inject(INJECT_PWM, drive, args.ceiling)

            data = frame(target.collect(args.seconds / 2, on_poll=on_poll))
            if not len(data["pv"]):
                raise SystemExit("no telemetry during limit sweep")
            rest = float(np.median(data["pv"][-20:]))
            print(f"  {name} stop rests at {rest:.0f} permille")
            if opening:
                seen_max = rest
            else:
                seen_min = rest
    finally:
        target.inject_off()

    margin = args.margin
    lo = int(math.ceil(seen_min + margin))
    hi = int(math.floor(seen_max - margin))
    print(f"\n  #define THROTTLE_SOFT_MIN       {lo}")
    print(f"  #define THROTTLE_SOFT_MAX       {hi}")
    print(f"\nMeasured stops {seen_min:.0f}..{seen_max:.0f} with a {margin} "
          "permille margin on each end. This is the jam fix: no target may")
    print("reach a hard stop, so the integral cannot wedge the plate into one.")


def cmd_ff(target: Target, args):
    require_consent(args, "ff")
    print("ENGINE OFF. Holding positions closed-loop and recording hold duty...")
    pts = []
    params = target.read_params()
    lo, hi = params["soft_min"], params["soft_max"]
    try:
        for pos in np.linspace(lo + 20, hi - 20, args.points):
            pos = int(pos)

            def on_poll(_e):
                target.inject(INJECT_POSITION, pos, args.ceiling)

            data = frame(target.collect(args.dwell, on_poll=on_poll))
            if len(data["pv"]) < 5:
                continue
            tail = max(5, len(data["pv"]) // 3)
            held = float(np.median(data["pv"][-tail:]))
            duty = float(np.median(data["u"][-tail:]))
            if abs(held - pos) < args.tolerance:
                pts.append((held, duty))
                print(f"  {pos:4d} permille -> held {held:4.0f}, "
                      f"duty {duty:6.1f}")
            else:
                print(f"  {pos:4d} permille -> only reached {held:4.0f}, skipped")
    finally:
        target.inject_off()

    if len(pts) < 3:
        raise SystemExit("not enough usable hold points to fit a line")

    x = np.array([p[0] for p in pts])
    y = np.array([p[1] for p in pts])
    slope, offset = np.polyfit(x, y, 1)
    pred = slope * x + offset
    r2 = 1.0 - np.sum((y - pred) ** 2) / np.sum((y - y.mean()) ** 2)

    print(f"\n  hold duty = {slope:.4f} * position + {offset:.1f}   (R2 = {r2:.3f})")
    print(f"\n  #define THROTTLE_FF_SLOPE       {int(round(slope * 1000))}")
    print(f"  #define THROTTLE_FF_OFFSET      {int(round(offset))}")

    mid = (lo + hi) // 2
    print(f"\n  hold duty at mid-travel ({mid} permille) is "
          f"{slope * mid + offset:.0f} counts.")
    print(f"  Pass that to ident:   pmbtune.py ... ident "
          f"--bias {int(round(slope * mid + offset))}")
    if r2 < 0.8:
        print("\nR2 is low: the spring is not behaving linearly, or the holds "
              "had not settled. Increase --dwell before trusting this.")


def cmd_idleff(target: Target, args):
    print("ENGINE RUNNING AT IDLE. Recording the plate opening that holds it.")
    print("Do not touch the pedal.")
    data = frame(target.collect(args.seconds))
    if len(data["rpm"]) < 20:
        raise SystemExit("not enough samples")

    rpm = data["rpm"]
    pv = data["pv"]
    stable = np.abs(rpm - np.median(rpm)) < args.rpm_band
    if stable.sum() < len(rpm) // 4:
        raise SystemExit(
            f"idle never held still (rpm {rpm.min():.0f}..{rpm.max():.0f}). "
            "Steady the idle by hand first, then re-run."
        )

    opening = float(np.median(pv[stable]))
    print(f"  idle rpm      : {np.median(rpm[stable]):.0f}")
    print(f"  plate opening : {opening:.1f} permille")
    print(f"\n  #define IDLE_FF_HOT  {int(round(opening))}   "
          "/* if the engine was warm */")
    print(f"  #define IDLE_FF_COLD {int(round(opening))}   "
          "/* re-measure this one cold */")
    print("\nThis is the base opening the idle PI trims around. Until it is "
          "set, the idle loop commands 0% at target rpm and cannot hold idle.")


def cmd_ident(target: Target, args):
    require_consent(args, "ident")
    print("ENGINE OFF. Small open-loop PWM steps inside the soft limits.")
    params = target.read_params()
    lo, hi = params["soft_min"], params["soft_max"]
    mid = (lo + hi) // 2

    # Park mid-travel closed-loop, then step open loop around that point.
    def park(_e):
        target.inject(INJECT_POSITION, mid, args.ceiling)

    target.collect(1.0, on_poll=park)

    amp = args.amplitude
    bias = args.bias
    period = args.period

    if bias == 0:
        print("\n  WARNING: --bias is 0, so the mean open-loop effort is zero "
              "and the\n  return spring will drag the plate onto the closed "
              "stop for much of the\n  test. Identifying against a hard stop "
              "produces a meaningless model.\n  Run `ff` first and pass its "
              "hold duty at mid-travel as --bias.\n")

    rows = []
    try:
        def on_poll(elapsed):
            phase = int(elapsed / period) % 2
            drive = bias + (amp if phase == 0 else -amp)
            target.inject(INJECT_PWM, drive, args.ceiling)

        rows = target.collect(args.seconds, on_poll=on_poll)
    finally:
        target.inject_off()

    data = frame(rows)
    if len(data["pv"]) < 50:
        raise SystemExit("not enough samples to identify a model")

    t = data["t_ms"]
    ts = float(np.median(np.diff(t))) / 1000.0
    coeffs, delay, r2 = fit_arx(data["u"], data["pv"], args.max_delay)
    plant = arx_to_plant(coeffs, delay, r2, ts)

    print(f"\n  samples        : {len(t)} at {ts*1000:.2f} ms")
    print(f"  simulation R2  : {plant.r2:.4f}  (free-run, not one-step)")
    print(f"  DC gain K      : {plant.K:.4f} permille per PWM count")
    print(f"  natural freq   : {plant.wn:.1f} rad/s "
          f"({plant.wn/(2*math.pi):.1f} Hz)")
    print(f"  damping zeta   : {plant.zeta:.3f}")
    print(f"  deadtime       : {plant.theta*1000:.1f} ms ({delay} samples)")

    if plant.r2 < 0.9:
        print("\n  WARNING: R2 below 0.9. The model does not describe the "
              "plate well -- do not derive gains from it. Usually this means "
              "the plate hit a stop, the step was too small to overcome "
              "stiction, or TPS is arriving too slowly (run `rate`).")

    Path(args.out).write_text(plant.to_json())
    print(f"\n  model written to {args.out}")
    if args.csv:
        write_csv(args.csv, data)
        print(f"  raw data written to {args.csv}")


def cmd_derive(target: Target | None, args):
    plant = Plant.from_json(Path(args.model))
    if plant.r2 < 0.9 and not args.force:
        raise SystemExit(
            f"model simulation R2 is {plant.r2:.3f}; re-run ident or --force"
        )

    print(f"plant: K={plant.K:.4f} wn={plant.wn:.1f} zeta={plant.zeta:.3f} "
          f"theta={plant.theta*1000:.1f}ms  (sim R2={plant.r2:.4f})")

    tau = 1.0 / plant.wn
    if args.lam:
        lambdas = [args.lam]
    else:
        # Geometric sweep from the IMC robustness floor outward.  The textbook
        # default (lambda = max(2*theta, 0.5*tau)) is the *fastest* defensible
        # choice, not necessarily an acceptable one: on this plant it predicts
        # ~28% overshoot, and overshoot on a throttle plate means travelling
        # toward a mechanical stop.  So sweep and pick against a budget.
        floor = max(2.0 * plant.theta, 0.5 * tau)
        lambdas = [floor * (1.25 ** k) for k in range(16)]

    print(f"\nIMC-PID (Rivera/Morari, 2nd-order process), tau = {tau*1000:.1f} ms")
    print(f"predicted on the identified model under the firmware's own "
          f"fixed-point controller,\nstep {args.sp_from} -> {args.sp_to} permille:\n")
    print(f"  {'lambda':>9}  {'Kc':>8}  {'rise':>8}  {'overshoot':>10}  "
          f"{'settle2%':>9}")

    candidates = []
    for lam in lambdas:
        g = imc_pid(plant, lam)
        sim = simulate_step(plant, g, args.sp_from, args.sp_to, args.horizon)
        m = step_metrics(sim["t"], sim["y"], args.sp_from, args.sp_to)
        if not m:
            continue
        ok = (m["overshoot_pct"] <= args.max_overshoot
              and math.isfinite(m["rise_10_90_ms"]))
        print(f"  {lam*1000:8.1f}ms {g.kc:8.4f}  {m['rise_10_90_ms']:6.0f}ms  "
              f"{m['overshoot_pct']:9.1f}%  {m['settle_2pct_ms']:7.0f}ms"
              f"{'' if not ok else '   <-- meets budget'}")
        if ok:
            candidates.append((lam, g, m))

    if not candidates:
        raise SystemExit(
            f"no lambda in the sweep keeps overshoot under "
            f"{args.max_overshoot}%.\nRaise --max-overshoot, or the model is "
            "wrong -- check the ident simulation R2 first."
        )

    # Fastest rise among those meeting the overshoot budget.
    lam, g, m = min(candidates, key=lambda c: c[2]["rise_10_90_ms"])
    print(f"\nselected lambda = {lam*1000:.1f} ms "
          f"(fastest with overshoot <= {args.max_overshoot}%)")
    print(f"  Kc = {g.kc:.4f}   Ti = {g.ti*1000:.1f} ms   Td = {g.td*1000:.1f} ms")
    print(f"  predicted: rise {m['rise_10_90_ms']:.0f} ms, "
          f"overshoot {m['overshoot_pct']:.1f}%, "
          f"settle(2%) {m['settle_2pct_ms']:.0f} ms")

    print("\nFirmware values (stored x1000; Kd is per millisecond, see pid.h):")
    print(f"  #define PID_FORWARD_P   {g.kp_m}")
    print(f"  #define PID_FORWARD_I   {g.ki_m}")
    print(f"  #define PID_FORWARD_D   {g.kd_m}")
    print("\nScope of this fit: ONE linear model of the plant around the "
          "operating point\nthat `ident` ran at. The excitation swings both "
          "ways around the holding bias,\nso the fit averages the opening and "
          "closing directions -- it does not separate\nthem.")
    print("\nSo: put these in the FORWARD (opening) gain set, start the "
          "BACKWARD set at the\nsame values, then trim the backward set "
          "empirically with `verify`. The spring\nassists closing, so the "
          "backward gains usually end up lower. Separating the two\n"
          "rigorously needs a segment-wise fit, which this tool does not do -- "
          "do not\npretend the backward numbers are derived.")

    Path(args.out).write_text(g.to_json())
    print(f"\n  gains written to {args.out}")
    print("  Nothing has been written to the target. Run `verify` next.")


def cmd_verify(target: Target, args):
    require_consent(args, "verify")
    g = json.loads(Path(args.gains).read_text())
    plant = Plant.from_json(Path(args.model)) if args.model else None

    saved = target.read_params()
    print("current gains:", {k: saved[k] for k in
                             ("kp_fwd", "ki_fwd", "kd_fwd")})

    try:
        target.write_params({"kp_fwd": g["kp_m"], "ki_fwd": g["ki_m"],
                             "kd_fwd": g["kd_m"]})
        print(f"applied kp_m={g['kp_m']} ki_m={g['ki_m']} kd_m={g['kd_m']}")

        half = args.period / 2.0

        def on_poll(elapsed):
            phase = int(elapsed / half) % 2
            target.inject(INJECT_POSITION,
                          args.sp_to if phase == 0 else args.sp_from,
                          args.ceiling)

        data = frame(target.collect(args.seconds, on_poll=on_poll))
    finally:
        target.inject_off()
        if not args.keep:
            target.write_params({k: saved[k] for k in
                                 ("kp_fwd", "ki_fwd", "kd_fwd")})
            print("restored previous gains (pass --keep to leave them applied)")

    if len(data["pv"]) < 30:
        raise SystemExit("not enough telemetry to evaluate")

    t = (data["t_ms"] - data["t_ms"][0]) / 1000.0

    # Find the edges that move TOWARD sp_to, whichever direction that is, so
    # that swapping --sp-from/--sp-to measures a closing step rather than
    # silently scoring a rising edge against falling-step expectations.
    direction = 1.0 if args.sp_to > args.sp_from else -1.0
    edges = np.where(np.diff(data["sp"]) * direction > 0)[0]
    if not len(edges):
        raise SystemExit(
            f"no setpoint edge toward {args.sp_to} captured "
            f"(expected steps {args.sp_from} -> {args.sp_to})"
        )

    mets = []
    for e in edges:
        seg = slice(e, min(len(t), e + int(half / (t[1] - t[0]))))
        m = step_metrics(t[seg] - t[seg][0], data["pv"][seg],
                         args.sp_from, args.sp_to)
        if m:
            mets.append(m)

    print(f"\nmeasured over {len(mets)} rising steps:")
    for key in ("rise_10_90_ms", "overshoot_pct", "settle_2pct_ms"):
        vals = [m[key] for m in mets if not math.isnan(m.get(key, float("nan")))]
        if vals:
            print(f"  {key:16s} {np.median(vals):8.1f}  "
                  f"(min {min(vals):.1f}, max {max(vals):.1f})")

    if plant is not None:
        gg = Gains(**{k: g[k] for k in
                      ("kp_m", "ki_m", "kd_m", "kc", "ti", "td", "lam")})
        sim = simulate_step(plant, gg, args.sp_from, args.sp_to, half)
        pm = step_metrics(sim["t"], sim["y"], args.sp_from, args.sp_to)
        print("\npredicted from the model:")
        for key in ("rise_10_90_ms", "overshoot_pct", "settle_2pct_ms"):
            print(f"  {key:16s} {pm.get(key, float('nan')):8.1f}")
        print("\nIf measured and predicted disagree badly the model is wrong, "
              "not the gains. Re-run ident before touching lambda.")

    if args.csv:
        write_csv(args.csv, data)
        print(f"\nraw data written to {args.csv}")


def write_csv(path: str, data: dict[str, np.ndarray]):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(SAMPLE_COLS)
        for row in zip(*[data[c] for c in SAMPLE_COLS]):
            w.writerow([int(v) for v in row])


def cmd_log(target: Target, args):
    print(f"logging {args.seconds:.0f}s to {args.out} ...")
    data = frame(target.collect(args.seconds))
    write_csv(args.out, data)
    print(f"wrote {len(data['t_ms'])} samples")


def cmd_monitor(target: Target, args):
    print("ctrl-c to stop")
    try:
        while True:
            d = frame(target.read_new())
            if len(d["t_ms"]):
                print(f"\rt={d['t_ms'][-1]:9.0f}ms  sp={d['sp'][-1]:5.0f}  "
                      f"pv={d['pv'][-1]:5.0f}  err={d['sp'][-1]-d['pv'][-1]:6.0f}  "
                      f"u={d['u'][-1]:6.0f}  rpm={d['rpm'][-1]:6.0f}  "
                      f"pedal={d['pedal'][-1]:5.0f}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def cmd_params(target: Target, args):
    if args.set:
        updates = {}
        for item in args.set:
            k, _, v = item.partition("=")
            updates[k.strip()] = int(v)
        target.write_params(updates)
        print("wrote:", updates)
    for k, v in target.read_params().items():
        print(f"  {k:18s} {v}")


# --------------------------------------------------------------------------

NEEDS_TARGET = {"rate", "limits", "stiction", "ff", "idleff", "ident",
                "verify", "log", "monitor", "params"}


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", help="path to the built ELF (default: build/Debug)")
    ap.add_argument("--backend", choices=("pyocd", "gdb"), default="pyocd",
                    help="how to reach the probe. 'pyocd' talks to it directly "
                         "(needs a WinUSB-bound driver on Windows). 'gdb' talks "
                         "to a running ST-LINK_gdbserver or OpenOCD over TCP, "
                         "which works with ST's own Windows driver")
    ap.add_argument("--gdb-host", default="127.0.0.1",
                    help="--backend gdb only")
    ap.add_argument("--gdb-port", type=int, default=61234,
                    help="--backend gdb only; ST-LINK_gdbserver defaults to 61234")
    ap.add_argument("--target-name", default="stm32c092rctx",
                    help="pyocd target type; run `pyocd pack find stm32c0` "
                         "and `pyocd pack install ...` if unknown")
    ap.add_argument("--frequency", type=int, default=4_000_000, help="SWD clock")
    ap.add_argument("--i-understand", action="store_true",
                    help="consent to commanding the throttle motor")
    ap.add_argument("--ceiling", type=int, default=400,
                    help="hard PWM cap while injecting (counts, default 400)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("rate", help="measure real control and CAN rates")
    p.add_argument("--seconds", type=float, default=5.0)

    p = sub.add_parser("limits", help="measure the mechanical stops")
    p.add_argument("--seconds", type=float, default=6.0)
    p.add_argument("--margin", type=int, default=15,
                   help="permille of margin to keep off each stop")

    p = sub.add_parser("stiction", help="measure breakaway duty")
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--motion", type=int, default=15,
                   help="permille of movement that counts as 'moved'")

    p = sub.add_parser("ff", help="fit the return-spring feedforward")
    p.add_argument("--points", type=int, default=7)
    p.add_argument("--dwell", type=float, default=1.5)
    p.add_argument("--tolerance", type=int, default=40)

    p = sub.add_parser("idleff", help="measure the idle base opening")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--rpm-band", type=float, default=150.0)

    p = sub.add_parser("ident", help="open-loop steps -> plant model")
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--amplitude", type=int, default=250,
                   help="open-loop PWM perturbation amplitude, counts")
    p.add_argument("--bias", type=int, default=0,
                   help="holding PWM the perturbation swings around. Set this "
                        "to the hold duty at mid-travel reported by `ff`, or "
                        "the spring parks the plate on the closed stop and the "
                        "model is garbage")
    p.add_argument("--period", type=float, default=1.0,
                   help="seconds per half cycle")
    p.add_argument("--max-delay", type=int, default=20)
    p.add_argument("--out", default="plant.json")
    p.add_argument("--csv")

    p = sub.add_parser("derive", help="model -> gains (offline, no hardware)")
    p.add_argument("model")
    p.add_argument("--lam", type=float,
                   help="pin IMC lambda, seconds (default: sweep and select)")
    p.add_argument("--max-overshoot", type=float, default=5.0,
                   help="overshoot budget in %% used to select lambda; "
                        "overshoot on a throttle plate means heading for a "
                        "mechanical stop (default 5)")
    p.add_argument("--sp-from", type=int, default=200)
    p.add_argument("--sp-to", type=int, default=600)
    p.add_argument("--horizon", type=float, default=1.0)
    p.add_argument("--force", action="store_true")
    p.add_argument("--out", default="gains.json")

    p = sub.add_parser("verify", help="apply gains and measure the response")
    p.add_argument("gains")
    p.add_argument("--model", help="also print the prediction for comparison")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--period", type=float, default=2.0)
    p.add_argument("--sp-from", type=int, default=200)
    p.add_argument("--sp-to", type=int, default=600)
    p.add_argument("--keep", action="store_true",
                   help="leave the gains applied on exit")
    p.add_argument("--csv")

    p = sub.add_parser("log", help="stream telemetry to CSV")
    p.add_argument("out")
    p.add_argument("--seconds", type=float, default=10.0)

    sub.add_parser("monitor", help="live one-line view")

    p = sub.add_parser("params", help="read/write the live parameter block")
    p.add_argument("--set", action="append", metavar="NAME=VALUE")

    args = ap.parse_args(argv)
    handler = globals()[f"cmd_{args.cmd}"]

    if args.cmd not in NEEDS_TARGET:
        return handler(None, args)

    with Target(find_elf(args.elf), lambda: make_backend(args)) as tgt:
        return handler(tgt, args)


if __name__ == "__main__":
    sys.exit(main())
