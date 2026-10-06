import os
import shutil
import subprocess
from pathlib import Path

Import("env")


project_dir = Path(env.subst("$PROJECT_DIR")).resolve()
repo_root = project_dir.parents[1]
cargo = Path.home() / ".cargo" / "bin" / "cargo.exe"
target_dir = project_dir / ".pio" / "rust"
target = "xtensa-esp32-none-elf"
archive_dir = target_dir / target / "release"

if not cargo.is_file():
    raise RuntimeError(f"Espressif Rust cargo was not found: {cargo}")

command = [
    str(cargo),
    "+esp",
    "build",
    "-Zbuild-std=core",
    "--manifest-path",
    str(repo_root / "rust" / "Cargo.toml"),
    "--package",
    "foc-rt-bridge",
    "--release",
    "--target",
    target,
    "--features",
    "motion-control,sensorless-foc",
]
rust_env = os.environ.copy()
rust_env["CARGO_TARGET_DIR"] = str(target_dir)
subprocess.run(command, cwd=repo_root, env=rust_env, check=True)

# `-Zbuild-std=core` links Rust's `compiler_builtins` memory functions into the
# static library.  Those weak `memcpy`/`memset` symbols can otherwise satisfy
# ESP-IDF startup references before the ROM/newlib implementation is selected.
# The Rust implementations live in flash, while ESP32 startup calls memset with
# the cache disabled, which causes an immediate boot panic.  Keep a transformed
# copy for the final link: Rust's own references follow the private names and
# ESP-IDF remains free to select its cache-safe runtime functions.
source_archive = archive_dir / "libfoc_rt_bridge.a"
linked_archive = archive_dir / "libfoc_rt_bridge_fluxrt.a"
if not source_archive.is_file():
    raise RuntimeError(f"Rust bridge archive was not produced: {source_archive}")

shutil.copy2(source_archive, linked_archive)
toolchain_dir = Path(
    env.PioPlatform().get_package_dir("toolchain-xtensa-esp-elf")
)
objcopy = toolchain_dir / "bin" / "xtensa-esp32-elf-objcopy.exe"
if not objcopy.is_file():
    raise RuntimeError(f"ESP32 objcopy was not found: {objcopy}")
subprocess.run(
    [
        str(objcopy),
        "--redefine-sym",
        "memcpy=__fluxrt_rust_memcpy",
        "--redefine-sym",
        "memmove=__fluxrt_rust_memmove",
        "--redefine-sym",
        "memset=__fluxrt_rust_memset",
        "--redefine-sym",
        "memcmp=__fluxrt_rust_memcmp",
        "--redefine-sym",
        "bcmp=__fluxrt_rust_bcmp",
        str(linked_archive),
    ],
    cwd=repo_root,
    check=True,
)

nm = toolchain_dir / "bin" / "xtensa-esp32-elf-nm.exe"
if not nm.is_file():
    raise RuntimeError(f"ESP32 nm was not found: {nm}")
symbols = subprocess.run(
    [str(nm), "-g", "--defined-only", str(linked_archive)],
    cwd=repo_root,
    check=True,
    capture_output=True,
    text=True,
).stdout.splitlines()
public_memory_symbols = {"memcpy", "memmove", "memset", "memcmp", "bcmp"}
leaked_symbols = sorted(
    line.split()[-1]
    for line in symbols
    if line.split() and line.split()[-1] in public_memory_symbols
)
if leaked_symbols:
    raise RuntimeError(
        "Rust archive still exports ESP-IDF memory runtime symbols: "
        + ", ".join(leaked_symbols)
    )

env.AppendUnique(LIBPATH=[str(archive_dir)])
env.AppendUnique(LIBS=["foc_rt_bridge_fluxrt"])
