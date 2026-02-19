"""
PlatformIO pre-script: two workarounds for the pioarduino espressif32 platform.

1. toolchain-riscv32-esp >= 14.x moved binaries from <pkg>/bin/ to
   <pkg>/riscv32-esp-elf/bin/.  PlatformIO's core only prepends <pkg>/bin to
   PATH so the compiler is not found.  We prepend the nested bin dir here.

2. littlefs-python 0.17.x has a circular-import bug that breaks the platform
   builder.  PlatformIO's penv upgrade mechanism keeps re-installing 0.17.x.
   We downgrade it back to 0.12.0 automatically whenever it drifts up.
"""
import os
import subprocess
import sys

Import("env")  # noqa: F821 — SCons global

# ── 1. Toolchain PATH fix ─────────────────────────────────────────────────────
toolchain_dir = env.PioPlatform().get_package_dir("toolchain-riscv32-esp")
if toolchain_dir:
    nested_bin = os.path.join(toolchain_dir, "riscv32-esp-elf", "bin")
    if os.path.isdir(nested_bin):
        env.PrependENVPath("PATH", nested_bin)
        print(f"fix_toolchain_path: prepended {nested_bin}")

# ── 2. littlefs-python version pin ───────────────────────────────────────────
# Use pip-show via subprocess so we always get the live version on disk,
# not a stale importlib.metadata cache from the current process.
GOOD_VERSION = "0.12.0"
penv_pip = os.path.join(os.path.expanduser("~"), ".platformio", "penv", "Scripts", "pip.exe")

try:
    result = subprocess.run(
        [penv_pip, "show", "littlefs-python"],
        capture_output=True, text=True, check=True,
    )
    installed = "unknown"
    for line in result.stdout.splitlines():
        if line.startswith("Version:"):
            installed = line.split(":", 1)[1].strip()
            break
except Exception:
    installed = "unknown"

if installed != GOOD_VERSION:
    print(f"fix_toolchain_path: littlefs-python {installed} — pinning to {GOOD_VERSION}")
    subprocess.check_call([penv_pip, "install", f"littlefs-python=={GOOD_VERSION}", "-q"])
    print("fix_toolchain_path: littlefs-python pinned; continuing build")
