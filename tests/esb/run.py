#!/usr/bin/env python3
"""Run the ESB link in simulation: dongle + both halves, real firmware, real keymap.

    python3 tests/esb/run.py setup <workspace>                  # once: fetch + build the simulator
    python3 tests/esb/run.py test  <workspace>                  # build the firmware, run every scenario
    python3 tests/esb/run.py test  <workspace> --only slow1000  # one scenario
    python3 tests/esb/run.py list

See tests/esb/README.md for what this covers and what it cannot.
"""
import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MODES = ROOT / "config" / "modes"
MODULES = [ROOT / "extra-modules" / "hfxo-always-on", ROOT / "extra-modules" / "zmk-split-esb"]
BOARD = "nrf52_bsim//zmk_test_mock"
DEVICES = ("central", "left", "right")
CONFS = {
    "central": [MODES / "esb.conf", HERE / "central.conf"],
    "left": [MODES / "esb.conf", MODES / "esb_half.conf", MODES / "esb_left.conf"],
    "right": [MODES / "esb.conf", MODES / "esb_half.conf", MODES / "esb_right.conf"],
}
# HID usage -> what run.py calls the key
KEYS = {0x50: "Left", 0x0D: "J", 0x2C: "Space", 0x08: "E", 0x0C: "I", 0x15: "R", 0x18: "U", 0x1A: "W", 0x12: "O"}

# Keys by (row, column) of the 10x4 matrix. Columns 0-4 are the left half.
THUMB = (3, 3)  # position 30: Nav on hold, Space on tap
RIGHT_J = (1, 6)  # position 16: J, Left on the Nav layer
IDLE = {"left": (0, 0), "right": (0, 9)}


def thumb_cycles(cycles, right_shift_ms=0):
    """The left thumb is held for a second and the right half taps J meanwhile.

    cycles: (second the thumb goes down, ms until J follows). A delay under
    200 ms decides the hold-tap through the other key, a longer one through its
    timer; either way the answer is Left. A thumb event that is lost or late
    types J instead - what a keyboard with a struggling link does.
    """
    left = [e for s, d in cycles for e in ((s * 1000, *THUMB, 1), (s * 1000 + 1000, *THUMB, 0))]
    right = [e for s, d in cycles
             for e in ((s * 1000 + d + right_shift_ms, *RIGHT_J, 1), (s * 1000 + d + 80 + right_shift_ms, *RIGHT_J, 0))]
    return left, right


def rolls(rounds, start_ms=4000, step_ms=35, hold_ms=50):
    """Both hands alternate, each key pressed before the previous one is up."""
    order = [("left", (0, 2), "E"), ("right", (0, 7), "I"), ("left", (0, 3), "R"),
             ("right", (0, 6), "U"), ("left", (0, 1), "W"), ("right", (0, 8), "O")]
    seq, typed, t = {"left": [], "right": []}, [], start_ms
    for _ in range(rounds):
        for half, key, name in order:
            seq[half] += [(t, *key, 1), (t + hold_ms, *key, 0)]
            typed.append(name)
            t += step_ms
        t += 400
    return seq["left"], seq["right"], typed


SHORT = [(4, 300), (7, 120), (12, 300), (20, 60), (30, 500), (34, 250)]
# Spread over 12 minutes, dense where the 24-bit RTC wraps (512 s).
LONG = list(zip([4, 30, 100, 200, 300, 400, 480, 500, 508, 511, 513, 516, 520, 530, 560, 600,
                 640, 690, 710], [300, 120, 300, 60, 500, 250] * 4))
LOSS = "-defmodem=Magic -argsdefmodem -PER="


@dataclass
class Scenario:
    what: str
    left: list
    right: list
    typed: list  # what the dongle has to type
    args: dict = field(default_factory=dict)  # simulator arguments per device, and "phy"
    conf: dict = field(default_factory=dict)  # extra Kconfig lines per device
    healthy: tuple = (0, 1)  # halves that must sync once and stay
    dead: tuple = ()  # halves the dongle must never get a usable report from
    settle: int = 2  # seconds until the halves have to be connected
    stalls: bool = False  # the dongle's watchdog has to have fired


def cycles_scenario(what, cycles, **kw):
    left, right = thumb_cycles(cycles)
    return Scenario(what, left, right, ["Left"] * len(cycles), **kw)


def make_scenarios():
    s = {
        "ideal": cycles_scenario("perfect clocks, no packet loss", SHORT),
        "slow80": cycles_scenario("left half 80 ppm slow (failed up to 1365f2b)", SHORT,
                                  args={"left": "-xo_drift=-80e-6"}),
        "slow1000": cycles_scenario("left half 1000 ppm slow (failed up to 27a86b8)", SHORT,
                                    args={"left": "-xo_drift=-1000e-6"}),
        "fast1000": cycles_scenario("right half 1000 ppm fast", SHORT, args={"right": "-xo_drift=1000e-6"}),
        "dongle1000": cycles_scenario("dongle 1000 ppm fast: both halves are slow", SHORT,
                                      args={"central": "-xo_drift=1000e-6"}),
        "loss30": cycles_scenario("30 % of all packets lost, clocks apart", SHORT,
                                  args={"left": "-xo_drift=-300e-6", "right": "-xo_drift=200e-6", "phy": LOSS + "0.3"}),
        "long": cycles_scenario("12 minutes across the RTC wrap, 5 % loss", LONG,
                                args={"left": "-xo_drift=30e-6", "right": "-xo_drift=-30e-6", "phy": LOSS + "0.05"}),
        "stall": cycles_scenario("the dongle misses a slot every 3 s and has to restart its slot sequence", SHORT,
                                 conf={"central": ["CONFIG_ZMK_ESB_SIM_STALL_EVERY=3000"]}, stalls=True),
    }
    left, right, typed = rolls(8)
    s["rolls"] = Scenario("both hands rolling, 35 ms from key to key, 5 % loss", left, right, typed,
                          args={"left": "-xo_drift=40e-6", "right": "-xo_drift=-40e-6", "phy": LOSS + "0.05"})
    # A half that is switched on 10 s after the others. Its own clock starts
    # then, so its key times are 10 s later than written; the right half's are
    # moved to match.
    left, right = thumb_cycles(SHORT[:4], right_shift_ms=10000)
    s["late"] = Scenario("left half switched on 10 s after the dongle", left, right, ["Left"] * 4,
                         args={"left": "-start_offset=10e6"}, settle=13)
    # Firmware of another release on the left half. Its thumb never arrives,
    # so J is all that is typed - plainly broken, not nearly working.
    left, right = thumb_cycles(SHORT)
    other = ["CONFIG_ZMK_ESB_PROTOCOL_VERSION=99"]
    s["other-half"] = Scenario("left half with another protocol version: it must not sync", left, right,
                               ["J"] * len(SHORT), conf={"left": other}, healthy=(1,), dead=(0,))
    s["old-half"] = Scenario("left half that syncs to any dongle: the dongle must not take its keys", left, right,
                             ["J"] * len(SHORT), healthy=(1,), dead=(0,),
                             conf={"left": other + ["CONFIG_ZMK_ESB_SIM_SKIP_VERSION_CHECK=y"]})
    return s


def sh(cmd, cwd=None, env=None, log=None):
    out = open(log, "w") if log else None
    r = subprocess.run(cmd, cwd=cwd, env=env, stdout=out, stderr=subprocess.STDOUT if out else None)
    if r.returncode:
        if log:
            print("".join(open(log, errors="replace").readlines()[-30:]), file=sys.stderr)
        raise SystemExit(f"failed: {' '.join(map(str, cmd))}")


def sim_env(ws):
    env = dict(os.environ)
    env.update(BSIM_OUT_PATH=str(ws / "tools" / "bsim"), BSIM_COMPONENTS_PATH=str(ws / "tools" / "bsim" / "components"))
    # The firmware is compiled for this machine. Where a Zephyr SDK is set up
    # (the CI container) it knows how to do that; elsewhere the host compiler does.
    if "ZEPHYR_TOOLCHAIN_VARIANT" not in env and "ZEPHYR_SDK_INSTALL_DIR" not in env:
        env["ZEPHYR_TOOLCHAIN_VARIANT"] = "host"
    return env


def setup(ws):
    """A west workspace of the ESB manifest with BabbleSim.

    Its own directory outside the repository, or the repository root itself,
    which is where CI keeps the workspace it builds the firmware in.
    """
    ws.mkdir(parents=True, exist_ok=True)
    if ws != ROOT:
        shutil.copytree(ROOT / "config-esb", ws / "config-esb", dirs_exist_ok=True)
    if not (ws / ".west").exists():
        sh(["west", "init", "-l", "config-esb"], cwd=ws)
    sh(["west", "config", "manifest.group-filter", "--", "+babblesim"], cwd=ws)
    sh(["west", "update", "--fetch-opt=--filter=tree:0", "--narrow"], cwd=ws)
    sh(["west", "zephyr-export"], cwd=ws)
    sh(["make", "everything", f"-j{os.cpu_count()}"], cwd=ws / "tools" / "bsim", log=ws / "bsim-build.log")
    sh([sys.executable, str(HERE / "patch_model.py")], cwd=ws / "modules" / "bsim_hw_models" / "nrf_hw_models")
    print(f"simulator ready in {ws}")


def mock_events(seq, idle):
    """Absolute (ms, row, col, press) -> devicetree overlay for the mock kscan.

    The mock waits ev[0].msec, fires ev[0], waits ev[0].msec again, fires
    ev[1], waits ev[1].msec, ... so msec is the gap after an event, and a dummy
    release of an idle key halfway to the first event sets the start. Gaps are
    15 bits, so long pauses get more of those dummies (which change no key
    state and are never sent).
    """
    filled, t = [], 0
    for e in sorted(seq):
        while e[0] - t > 30000:
            t += 25000
            filled.append((t, *idle, 0))
        filled.append(e)
        t = e[0]
    seq = [(filled[0][0] // 2, *idle, 0)] + filled
    out = []
    for i, (at, row, col, press) in enumerate(seq):
        gap = seq[i + 1][0] - at if i + 1 < len(seq) else 1
        assert 0 < gap < 32768, gap
        out.append(f"ZMK_MOCK_{'PRESS' if press else 'RELEASE'}({row},{col},{gap})")
    return ("#include <dt-bindings/zmk/kscan_mock.h>\n\n&kscan {\n    events = <\n        "
            + "\n        ".join(out) + "\n    >;\n};\n")


def firmware_key(device, scenario):
    """What one device's firmware depends on. The key events are compiled into a half."""
    overlay = mock_events(getattr(scenario, device), IDLE[device]) if device != "central" else ""
    conf = "\n".join(scenario.conf.get(device, []))
    return device, overlay, conf


def build(ws, out, key):
    device, overlay, conf = key
    bdir = out / "build" / f"{device}-{hashlib.sha1((overlay + conf).encode()).hexdigest()[:10]}"
    bdir.mkdir(parents=True, exist_ok=True)
    confs = list(CONFS[device])
    if conf:
        (bdir / "extra.conf").write_text(conf + "\n")
        confs.append(bdir / "extra.conf")
    args = [f"-DZMK_CONFIG={HERE}", "-DZMK_EXTRA_MODULES=" + ";".join(map(str, MODULES)),
            "-DEXTRA_CONF_FILE=" + ";".join(map(str, confs)), "-DDTS_EXTRA_CPPFLAGS=-DKB_NO_BLE"]
    if overlay:
        (bdir / "events.overlay").write_text(overlay)
        args.append(f"-DEXTRA_DTC_OVERLAY_FILE={bdir / 'events.overlay'}")
    sh(["west", "build", "-p", "-s", "zmk/app", "-d", str(bdir / "b"), "-b", BOARD, "--", *args],
       cwd=ws, env=sim_env(ws), log=bdir / "build.log")
    return bdir / "b" / "zephyr" / "zmk.exe"


def run(ws, out, name, scenario, exes):
    logs = out / name
    logs.mkdir(parents=True, exist_ok=True)
    sim, bin_dir = f"esb_{name}_{os.getpid()}", ws / "tools" / "bsim" / "bin"
    last = max(e[0] for e in scenario.left + scenario.right) / 1000
    shift = max((float(m[1]) / 1e6 for a in scenario.args.values() for m in re.finditer(r"start_offset=([\d.e]+)", a)),
                default=0)
    procs = [subprocess.Popen([str(exes[firmware_key(d, scenario)]), f"-s={sim}", f"-d={i}", *scenario.args.get(d, "").split()],
                              stdout=open(logs / f"{d}.log", "w"), stderr=subprocess.STDOUT, cwd=bin_dir)
             for i, d in enumerate(DEVICES)]
    subprocess.run([str(bin_dir / "bs_2G4_phy_v1"), f"-s={sim}", "-D=3", f"-sim_length={int(last + shift + 3)}e6",
                    *scenario.args.get("phy", "").split()],
                   stdout=open(logs / "phy.log", "w"), stderr=subprocess.STDOUT, cwd=bin_dir)
    for p in procs:
        p.wait()
    return logs


TS = re.compile(r"@(\d+):(\d+):(\d+\.\d+)")


def stamp(line):
    m = TS.search(line)
    return int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3])


def check(name, scenario, logs):
    """What the dongle typed and what the link statistics say. Returns the failures."""
    typed, halves, stalled, errors = [], {}, 0, 0

    def half(n):
        return halves.setdefault(int(n), dict(down=0, lost=0, refused=0, reports=0, scans=0, timeouts=0,
                                              retries=0, wait=0, ppm=0))

    for d in DEVICES:
        errors += sum("ERROR" in l or "ASSERT" in l for l in open(logs / f"{d}.log", errors="replace"))
    for l in open(logs / "central.log", errors="replace"):
        m = re.search(r"hid_listener_keycode_pressed: usage_page 0x07 keycode 0x(\w+)", l)
        if m:
            typed.append(KEYS.get(int(m[1], 16), "0x" + m[1]))
        m = re.search(r"ESB half (\d) (up|DOWN) rssi -\d+ answers \d+ keys \d+ dup \d+ lost\(total\) (\d+) "
                      r"skipped \d+ stalled\(total\) (\d+) refused (\d+)", l)
        if m:
            h = half(m[1])
            h["down"] += m[2] == "DOWN" and stamp(l) > scenario.settle
            h["lost"], stalled = int(m[3]), int(m[4])
            h["refused"] += int(m[5])
        m = re.search(r"ESB half (\d) reports: timeouts (\d+) retries (\d+) scans\(total\) (\d+) wait (\d+) ms "
                      r"power -?\d+ dBm drift (-?\d+) ppm", l)
        if m:
            h = half(m[1])
            h["reports"] += 1
            h["timeouts"] += int(m[2])
            h["retries"] += int(m[3])
            h["scans"], h["ppm"] = int(m[4]), int(m[6])
            h["wait"] = max(h["wait"], int(m[5]))

    fails = []
    if errors:
        fails.append(f"{errors} simulator error(s) in the logs")
    if typed != scenario.typed:
        fails.append(f"typed     {' '.join(typed) or 'nothing'}")
        fails.append(f"should be {' '.join(scenario.typed)}")
    for h in scenario.healthy:
        s = half(h)
        if s["scans"] != 1 or s["lost"] or s["down"] or not s["reports"]:
            fails.append(f"half {h} lost sync: scans {s['scans']} (1 is the start-up one), "
                         f"dropped by the dongle {s['lost']}, seconds down {s['down']}")
    for h in scenario.dead:
        if half(h)["reports"]:
            fails.append(f"half {h} was accepted by the dongle")
    if scenario.stalls and not stalled:
        fails.append("the stall was not injected, or the watchdog did not see it")
    if not scenario.stalls and stalled:
        fails.append(f"the dongle's slot sequence stopped {stalled} time(s)")

    print(f"{'FAIL' if fails else 'ok  '} {name:11}", end=" ")
    print("  ".join(f"half {h}: wait {s['wait']} ms, {s['timeouts']} timeouts, {s['retries']} retries, "
                    f"drift {s['ppm']} ppm" if s["reports"] else f"half {h}: not accepted, {s['refused']} refused"
                    for h, s in sorted(halves.items())) + (f"  restarts {stalled}" if stalled else ""))
    for f in fails:
        print(f"       {f}")
    return fails


def test(ws, out, only):
    scenarios = make_scenarios()
    if only:
        scenarios = {n: scenarios[n] for n in only}
    keys = sorted({firmware_key(d, s) for s in scenarios.values() for d in DEVICES})
    print(f"building {len(keys)} firmware images ...", flush=True)
    with ThreadPoolExecutor(2) as pool:
        exes = dict(zip(keys, pool.map(lambda k: build(ws, out, k), keys)))
    print(f"running {len(scenarios)} scenarios ...", flush=True)
    with ThreadPoolExecutor(max(1, (os.cpu_count() or 2) // 2)) as pool:
        logs = dict(zip(scenarios, pool.map(lambda i: run(ws, out, i[0], i[1], exes), scenarios.items())))
    failed = [n for n, s in scenarios.items() if check(n, s, logs[n])]
    print()
    for n, s in scenarios.items():
        print(f"  {n:11} {s.what}")
    print(f"logs: {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["setup", "test", "list"])
    ap.add_argument("workspace", type=Path, nargs="?")
    ap.add_argument("--only", action="append", help="run this scenario (may be repeated)")
    a = ap.parse_args()
    if a.command == "list":
        for n, s in make_scenarios().items():
            print(f"{n:11} {s.what}")
        sys.exit(0)
    if not a.workspace:
        ap.error("the workspace directory is missing")
    ws = a.workspace.resolve()
    if a.command == "setup":
        setup(ws)
    else:
        sys.exit(test(ws, ws / "esb-test", a.only))
