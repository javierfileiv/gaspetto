#!/usr/bin/env python3
"""
PlatformIO extra_script: stamp the firmware version into the build.

The version is the git revision the binary was built from, so it cannot drift
from the sources: the commit count gives a number that only grows, and the short
hash says exactly which tree was compiled. A dirty tree is marked as such,
because "v412-dirty" is a different firmware from "v412".

The values are passed as compiler definitions rather than written into a
header. A generated header is invisible to the build dependency scanner when it
is reached through __has_include, so a new revision would silently keep the old
version in the binary; a changed -D changes the compile command and forces the
rebuild. Host builds (the emulator, the unit tests) have no definitions and
FirmwareVersion.h falls back to a development marker.
"""

Import("env")  # noqa: F821 - PlatformIO SCons env

import subprocess


def _git(arguments, cwd):
    try:
        return subprocess.check_output(
            ["git"] + arguments, cwd=cwd, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _generate():
    project_dir = env.subst("$PROJECT_DIR")
    repo = _git(["rev-parse", "--show-toplevel"], project_dir) or project_dir

    count = _git(["rev-list", "--count", "HEAD"], repo) or "0"
    build = _git(["rev-parse", "--short", "HEAD"], repo) or "unknown"
    if _git(["status", "--porcelain", "--untracked-files=no"], repo):
        build += "-dirty"

    env.Append(
        CPPDEFINES=[
            ("FW_VERSION", count),
            ("FW_BUILD", env.StringifyMacro(build)),
        ]
    )


_generate()
