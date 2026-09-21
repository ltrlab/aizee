#!/usr/bin/env python3
"""Inspect Jetson ELF files without running them (requires pyelftools)."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from elftools.elf.elffile import ELFFile


def inspect(path: Path, *, shared_library=False) -> dict:
    with path.open("rb") as stream:
        elf = ELFFile(stream)
        if (elf.elfclass, elf.little_endian, elf["e_machine"]) != (64, True, "EM_AARCH64"):
            raise ValueError(f"Not a little-endian AArch64 ELF64 executable: {path}")
        if shared_library and elf["e_type"] != "ET_DYN":
            raise ValueError("Shared library must be an ELF dynamic object")
        interpreter = next(
            (segment.get_interp_name() for segment in elf.iter_segments()
             if segment["p_type"] == "PT_INTERP"), None
        )
        dynamic = elf.get_section_by_name(".dynamic")
        needed = sorted(tag.needed for tag in dynamic.iter_tags("DT_NEEDED")) if dynamic else []
        search_paths = []
        if dynamic:
            for tag in dynamic.iter_tags():
                if tag.entry.d_tag == "DT_RUNPATH":
                    search_paths.append(tag.runpath)
                elif tag.entry.d_tag == "DT_RPATH":
                    search_paths.append(tag.rpath)
        versions = {}
        requirements = elf.get_section_by_name(".gnu.version_r")
        if requirements:
            for version, auxiliaries in requirements.iter_versions():
                versions[version.name] = sorted(auxiliary.name for auxiliary in auxiliaries)

    names = {name for library in versions.values() for name in library}

    def maximum(prefix: str) -> str | None:
        values = [name[len(prefix):] for name in names if name.startswith(prefix)]
        values = [value for value in values if all(part.isdecimal() for part in value.split("."))]
        return max(values, key=lambda value: tuple(map(int, value.split(".")))) if values else None

    glibc = maximum("GLIBC_")
    if glibc and tuple(map(int, glibc.split("."))) > (2, 35):
        raise ValueError(f"{path.name} requires GLIBC {glibc}, newer than the JetPack 6 baseline")
    if interpreter != (None if shared_library else "/lib/ld-linux-aarch64.so.1"):
        raise ValueError(f"Unexpected Linux AArch64 interpreter: {interpreter}")
    return {
        "file": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "format": "ELF64 little-endian",
        "machine": "AArch64",
        "interpreter": interpreter,
        "shared_library": shared_library,
        "needed_libraries": needed,
        "library_search_paths": search_paths,
        "max_glibc": glibc,
        "max_glibcxx": maximum("GLIBCXX_"),
        "symbol_version_requirements": versions,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binaries", nargs="*", type=Path)
    parser.add_argument("--output", type=Path, help="Also save the JSON inspection report")
    parser.add_argument("--shared-library", action="store_true", help="Validate shared objects without a program interpreter")
    args = parser.parse_args()
    release = Path(__file__).resolve().parents[1] / "rust/target/aarch64-unknown-linux-gnu/release"
    binaries = args.binaries or [release / name for name in ("motor_control", "lidar_control")]
    report = json.dumps([inspect(path,shared_library=args.shared_library) for path in binaries], indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
    print(report, end="")


if __name__ == "__main__":
    main()
