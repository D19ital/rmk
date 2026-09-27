#!/usr/bin/env bash

set -euo pipefail

if (( $# == 0 )); then
    echo "usage: $0 ELF [ELF ...]" >&2
    exit 2
fi

for elf in "$@"; do
    [[ -f "$elf" ]] || { echo "$elf: not a file" >&2; exit 1; }

    version_symbols="$(readelf --wide --syms "$elf" | grep -E '[[:space:]]_defmt_version_ = [^[:space:]]+$' || true)"
    exact_count="$(grep -Ec '[[:space:]]_defmt_version_ = 4$' <<<"$version_symbols" || true)"
    total_count="$(grep -c . <<<"$version_symbols" || true)"

    if [[ "$exact_count" != 1 || "$total_count" != 1 ]]; then
        echo "$elf: expected exactly one '_defmt_version_ = 4' symbol" >&2
        [[ -n "$version_symbols" ]] && printf '%s\n' "$version_symbols" >&2
        exit 1
    fi

    readelf --wide --sections "$elf" | grep -Eq '[[:space:]]\.defmt[[:space:]]+PROGBITS[[:space:]]' \
        || { echo "$elf: missing .defmt PROGBITS section" >&2; exit 1; }

    echo "$elf: defmt wire-format symbol and section OK"
done
