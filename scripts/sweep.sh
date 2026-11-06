#!/usr/bin/env bash
# Runs a built example on every session in `data/sessions/` but those in `scripts/sweep-skip.txt`,
# `$SWEEP_JOBS` at a time (default 4), largest first, each in its own `systemd-run` scope capped at
# `$SWEEP_CAP` (default 1200M) with no swap: the largest session peaks at 0.53 GB a replay (0.40 GB for `score`),
# so 4 caps (4.8 GB) stay under the 8 GB `claude.slice` with room for builds and Claude.
# Each session's output is kept apart and printed whole, in session order, at the end.
set -euo pipefail
example=$1
shift
jobs=${SWEEP_JOBS:-4}
cap=${SWEEP_CAP:-1200M}
out=data/work/sweep/$example
mkdir -p "$out"
bin=target/release/examples/$example
sessions=()
for s in data/sessions/*/; do
    s=${s%/}
    if rg -qx "$(basename "$s")" scripts/sweep-skip.txt; then continue; fi
    sessions+=("$s")
done
run_one() {
    local s=$1
    shift
    local name
    name=$(basename "$s")
    {
        echo "== $name"
        systemd-run --user --scope --quiet --collect -p MemoryMax="$cap" -p MemorySwapMax=0 \
            "$bin" "$s" "$@" 2>&1 \
            || echo "!! $name failed (exit $?; 137 means over the $cap cap)"
    } > "$out/$name.out"
}
export -f run_one
export bin out cap
# Largest first, so the longest replays don't start last.
du -s "${sessions[@]}" | sort -rn | cut -f2 \
    | xargs -P "$jobs" -I{} bash -c 'run_one "$@"' _ {} "$@"
for s in "${sessions[@]}"; do
    cat "$out/$(basename "$s").out"
done
