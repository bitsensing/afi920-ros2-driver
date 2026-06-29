#!/usr/bin/env python3
"""Run live AFI920 feature-data checks from the ROS workspace.

This wrapper reuses the SDK live conformance suite so the ROS repo can verify
the same real-radar contract without duplicating protocol test logic.
It exercises only the public stream surface (RDI, SHII, SPI, CSII).
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
from pathlib import Path


DEFAULT_IP = "192.168.10.150"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def sdk_candidates(explicit: str | None) -> list[Path]:
    root = repo_root()
    candidates: list[Path] = []

    if explicit:
        candidates.append(Path(explicit))

    env_root = os.environ.get("AFI920_SDK_ROOT")
    if env_root:
        candidates.append(Path(env_root))

    candidates.extend(
        [
            root.parent / "afi920_sdk",
        ]
    )

    seen: set[Path] = set()
    unique: list[Path] = []
    for item in candidates:
        resolved = item.expanduser().resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(resolved)
    return unique


def find_sdk_root(explicit: str | None) -> Path:
    for candidate in sdk_candidates(explicit):
        if (candidate / "test" / "test_live_conformance.py").is_file():
            return candidate

    searched = "\n  ".join(str(p) for p in sdk_candidates(explicit))
    raise FileNotFoundError(
        "Could not find afi920_sdk/test/test_live_conformance.py. "
        "Set --sdk-root or AFI920_SDK_ROOT.\nSearched:\n  " + searched
    )


def default_python(sdk_root: Path) -> str:
    win_venv = sdk_root / "python" / ".venv" / "Scripts" / "python.exe"
    posix_venv = sdk_root / "python" / ".venv" / "bin" / "python"
    if win_venv.is_file():
        return str(win_venv)
    if posix_venv.is_file():
        return str(posix_venv)
    return sys.executable


def detect_local_ip(target_ip: str) -> str:
    """Return the local source IP used to route to *target_ip*."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((target_ip, 1))
        return sock.getsockname()[0]
    except OSError:
        return ""
    finally:
        sock.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "ips",
        nargs="*",
        default=[DEFAULT_IP],
        help=f"Radar IP(s) to verify. Default: {DEFAULT_IP}",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=8.0,
        help="Stream capture duration in seconds. Default: 8",
    )
    parser.add_argument("--bind", default="", help="Local IP for discovery broadcast")
    parser.add_argument("--skip-csii", action="store_true", help="Skip CSII send test")
    parser.add_argument("--sdk-root", default="", help="Path to afi920_sdk")
    parser.add_argument("--python", default="", help="Python executable to run the SDK test")
    args = parser.parse_args()

    sdk_root = find_sdk_root(args.sdk_root or None)
    py = args.python or default_python(sdk_root)
    test_script = sdk_root / "test" / "test_live_conformance.py"

    cmd = [
        py,
        str(test_script),
        *args.ips,
        "--duration",
        str(args.duration),
    ]
    bind_ip = args.bind
    if not bind_ip and args.ips:
        bind_ip = detect_local_ip(args.ips[0])
    if bind_ip:
        cmd.extend(["--bind", bind_ip])
    if args.skip_csii:
        cmd.append("--skip-csii")

    print(f"SDK root: {sdk_root}", flush=True)
    print(f"Python:   {py}", flush=True)
    print("Command:  " + " ".join(cmd), flush=True)
    final_rc = subprocess.call(cmd, cwd=str(sdk_root))

    return final_rc


if __name__ == "__main__":
    raise SystemExit(main())
