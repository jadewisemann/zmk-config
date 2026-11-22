#!/usr/bin/env python3
"""Run the ESB link in simulation: dongle + both halves, real firmware, real keymap.

    python3 tests/esb/run.py setup  <workspace>     # once: fetch + build the simulator
    python3 tests/esb/run.py test   <workspace>     # build the firmware, run every scenario
    python3 tests/esb/run.py test   <workspace> --long          # also 12 simulated minutes
    python3 tests/esb/run.py test   <workspace> --only slow1000 # one scenario, logs kept

See tests/esb/README.md for what this covers and what it cannot.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MODES = ROOT / "config" / "modes"
MODULES = [ROOT / "extra-modules" / "hfxo-always-on", ROOT / "extra-modules" / "zmk-split-esb"]
BOARD = "nrf52_bsim//zmk_test_mock"

# The test presses two keys: the left thumb (Nav on hold, Space on tap) and J
# on the right half, which is Left on the Nav layer.
THUMB = (3, 3)  # row, column in the 10x4 matrix: position 30
RIGHT_KEY = (1, 6)  # position 16
IDLE_LEFT, IDLE_RIGHT = (0, 0), (0, 9)

# (second the thumb goes down, ms until the right key follows). The thumb is
# held for a second; the right key for 80 ms. Delays under 200 ms decide the
# hold-tap through the other key, longer ones through its timer.
SHORT = [(4, 300), (7, 120), (12, 300), (20, 60), (30, 500), (34, 250)]
# Spread over 12 minutes, dense where the 24-bit RTC wraps (512 s).
LONG = list(zip([4, 30, 100, 200, 300, 400, 480, 500, 508, 511, 513, 516, 520, 530, 560, 600,
                 640, 690, 710], [300, 120, 300, 60, 500, 250] * 4))

# name: (cycles, device arguments, phy arguments, what it stands for)
SCENARIOS = {
    "ideal": (SHORT, {}, "", "perfect clocks, no packet loss"),
    "slow80": (SHORT, {"left": "-xo_drift=-80e-6"}, "",
               "left half 80 ppm slow: what the firmware before 008572a could not hold"),
    "slow1000": (SHORT, {"left": "-xo_drift=-1000e-6"}, "",
                 "left half 1000 ppm slow: what the firmware before the re-sync ramp could not hold"),
    "fast1000": (SHORT, {"right": "-xo_drift=1000e-6"}, "", "right half 1000 ppm fast"),
    "dongle1000": (SHORT, {"central": "-xo_drift=1000e-6"}, "", "dongle 1000 ppm fast: both halves are slow"),
    "loss30": (SHORT, {"left": "-xo_drift=-300e-6", "right": "-xo_drift=200e-6"},
               "-defmodem=Magic -argsdefmodem -PER=0.3", "30 % of all packets lost, clocks apart"),
}
LONG_SCENARIOS = {
    "long": (LONG, {"left": "-xo_drift=30e-6", "right": "-xo_drift=-30e-6"},
             "-defmodem=Magic -argsdefmodem -PER=0.05", "12 minutes across the RTC wrap, 5 % loss"),
}


def sh(cmd, cwd=None, env=None, log=None):
    out = open(log, "w") if log else None
    r = subprocess.run(cmd, cwd=cwd, env=env, stdout=out or None, stderr=subprocess.STDOUT if out else None)
    if r.returncode:
        if log:
            print("".join(open(log, errors="replace").readlines()[-25:]), file=sys.stderr)
        raise SystemExit(f"failed: {' '.join(map(str, cmd))}")


def sim_env(ws):
    env = dict(os.environ)
    env.update(BSIM_OUT_PATH=str(ws / "tools" / "bsim"),
               BSIM_COMPONENTS_PATH=str(ws / "tools" / "bsim" / "components"),
               ZEPHYR_TOOLCHAIN_VARIANT="host")
    return env


def setup(ws):
    """A west workspace of the ESB manifest with BabbleSim, outside the repository."""
    ws.mkdir(parents=True, exist_ok=True)
    if not (ws / ".west").exists():
        shutil.copytree(ROOT / "config-esb", ws / "config-esb", dirs_exist_ok=True)
        sh(["west", "init", "-l", "config-esb"], cwd=ws)
        sh(["west", "config", "manifest.group-filter", "--", "+babblesim"], cwd=ws)
    else:
        shutil.copy(ROOT / "config-esb" / "west.yml", ws / "config-esb" / "west.yml")
    sh(["west", "update", "--fetch-opt=--filter=tree:0", "--narrow"], cwd=ws)
    sh(["make", "everything", f"-j{os.cpu_count()}"], cwd=ws / "tools" / "bsim")
    sh([sys.executable, str(HERE / "patch_model.py")], cwd=ws / "modules" / "bsim_hw_models" / "nrf_hw_models")
    print(f"simulator ready in {ws}")


def events(seq, idle):
    """Absolute (ms, row, col, press) -> kscan-mock events.

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


def build(ws, out, cycles, tag):
    """Dongle + two halves for one set of key cycles. The events are compiled into the halves."""
    bdir = out / f"build-{tag}"
    bdir.mkdir(parents=True, exist_ok=True)
    left = [x for s, d in cycles for x in ((s * 1000, *THUMB, 1), (s * 1000 + 1000, *THUMB, 0))]
    right = [x for s, d in cycles for x in ((s * 1000 + d, *RIGHT_KEY, 1), (s * 1000 + d + 80, *RIGHT_KEY, 0))]
    (bdir / "left.overlay").write_text(events(left, IDLE_LEFT))
    (bdir / "right.overlay").write_text(events(right, IDLE_RIGHT))
    targets = {
        "central": ([MODES / "esb.conf", HERE / "central.conf"], None),
        "left": ([MODES / "esb.conf", MODES / "esb_half.conf", MODES / "esb_left.conf"], bdir / "left.overlay"),
        "right": ([MODES / "esb.conf", MODES / "esb_half.conf", MODES / "esb_right.conf"], bdir / "right.overlay"),
    }

    def one(name):
        confs, overlay = targets[name]
        args = [f"-DZMK_CONFIG={HERE}", "-DZMK_EXTRA_MODULES=" + ";".join(map(str, MODULES)),
                "-DEXTRA_CONF_FILE=" + ";".join(map(str, confs)), "-DDTS_EXTRA_CPPFLAGS=-DKB_NO_BLE"]
        if overlay:
            args.append(f"-DEXTRA_DTC_OVERLAY_FILE={overlay}")
        sh(["west", "build", "-p", "-s", "zmk/app", "-d", str(bdir / name), "-b", BOARD, "--", *args],
           cwd=ws, env=sim_env(ws), log=bdir / f"{name}.log")

    with ThreadPoolExecutor(3) as pool:
        list(pool.map(one, targets))
    return bdir


TS = re.compile(r"@(\d+):(\d+):(\d+\.\d+)")


def stamp(line):
    m = TS.search(line)
    return int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3])


def run(ws, bdir, out, name, cycles, dev_args, phy_args):
    logs = out / name
    logs.mkdir(parents=True, exist_ok=True)
    sim, bin_dir = f"esb_{name}_{os.getpid()}", ws / "tools" / "bsim" / "bin"
    seconds = cycles[-1][0] + 3
    procs = [subprocess.Popen([str(bdir / n / "zephyr" / "zmk.exe"), f"-s={sim}", f"-d={i}", *dev_args.get(n, "").split()],
                              stdout=open(logs / f"{n}.log", "w"), stderr=subprocess.STDOUT, cwd=bin_dir)
             for i, n in enumerate(("central", "left", "right"))]
    subprocess.run([str(bin_dir / "bs_2G4_phy_v1"), f"-s={sim}", "-D=3", f"-sim_length={seconds}e6", *phy_args.split()],
                   stdout=open(logs / "phy.log", "w"), stderr=subprocess.STDOUT, cwd=bin_dir)
    for p in procs:
        p.wait()
    return logs


def check(name, logs, cycles):
    """What the dongle typed and what the link statistics say. Returns the list of failures."""
    typed, halves, errors = [], {}, 0
    for n in ("central", "left", "right"):
        errors += sum("ERROR" in l or "ASSERT" in l for l in open(logs / f"{n}.log", errors="replace"))
    for l in open(logs / "central.log", errors="replace"):
        m = re.search(r"hid_listener_keycode_pressed: usage_page 0x07 keycode 0x(\w+)", l)
        if m:
            typed.append({0x50: "Left", 0x0D: "J", 0x2C: "Space"}.get(int(m[1], 16), m[1]))
        m = re.search(r"ESB half (\d) (up|DOWN) rssi -\d+ answers \d+ keys \d+ dup \d+ lost\(total\) (\d+)", l)
        if m:
            h = halves.setdefault(int(m[1]), dict(down=0, lost=0, scans=0, timeouts=0, retries=0, wait=0, ppm=0))
            # The first second is the start-up scan.
            h["down"] += m[2] == "DOWN" and stamp(l) > 2
            h["lost"] = int(m[3])
        m = re.search(r"ESB half (\d) reports: timeouts (\d+) retries (\d+) scans\(total\) (\d+) wait (\d+) ms "
                      r"power -?\d+ dBm drift (-?\d+) ppm", l)
        if m:
            h = halves.setdefault(int(m[1]), dict(down=0, lost=0, scans=0, timeouts=0, retries=0, wait=0, ppm=0))
            h["timeouts"] += int(m[2])
            h["retries"] += int(m[3])
            h["scans"], h["ppm"] = int(m[4]), int(m[6])
            h["wait"] = max(h["wait"], int(m[5]))

    fails = []
    if errors:
        fails.append(f"{errors} simulator error(s) in the logs")
    if typed != ["Left"] * len(cycles):
        fails.append(f"typed {' '.join(typed) or 'nothing'} instead of {len(cycles)} x Left")
    for h in (0, 1):
        s = halves.get(h)
        if not s:
            fails.append(f"half {h}: no statistics")
            continue
        if s["scans"] != 1 or s["lost"] or s["down"]:
            fails.append(f"half {h} lost sync: scans {s['scans']} (1 is the start-up one), "
                         f"dropped by the dongle {s['lost']}, seconds down {s['down']}")
    print(f"{'FAIL' if fails else 'ok  '} {name:11}", end=" ")
    print("  ".join(f"half {h}: wait {s['wait']} ms, {s['timeouts']} timeouts, {s['retries']} retries, drift {s['ppm']} ppm"
                    for h, s in sorted(halves.items())))
    for f in fails:
        print(f"       {f}")
    return fails


def test(ws, out, long, only):
    scenarios = dict(SCENARIOS, **(LONG_SCENARIOS if long or only in LONG_SCENARIOS else {}))
    if only:
        scenarios = {only: scenarios[only]}
    builds = {}
    for cycles in {id(c): c for c, *_ in scenarios.values()}.values():
        tag = "long" if cycles is LONG else "short"
        print(f"building dongle, left and right ({tag} key sequence) ...", flush=True)
        builds[id(cycles)] = build(ws, out, cycles, tag)

    def one(item):
        name, (cycles, dev_args, phy_args, _) = item
        return name, cycles, run(ws, builds[id(cycles)], out, name, cycles, dev_args, phy_args)

    with ThreadPoolExecutor(max(1, (os.cpu_count() or 2) // 2)) as pool:
        results = list(pool.map(one, scenarios.items()))
    failed = [name for name, cycles, logs in results if check(name, logs, cycles)]
    for name, (_, _, _, what) in scenarios.items():
        print(f"  {name:11} {what}")
    print(f"logs: {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["setup", "test"])
    ap.add_argument("workspace", type=Path)
    ap.add_argument("--long", action="store_true", help="also run the 12 minute scenario")
    ap.add_argument("--only", help="run one scenario")
    a = ap.parse_args()
    ws = a.workspace.resolve()
    if a.command == "setup":
        setup(ws)
    else:
        sys.exit(test(ws, ws / "esb-test", a.long, a.only))
