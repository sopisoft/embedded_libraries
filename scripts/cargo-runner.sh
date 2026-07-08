#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "cargo-runner.sh: missing executable path" >&2
  exit 1
fi

binary_path="$1"
shift
binary_name="$(basename -- "$binary_path")"
workspace_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

if [ "$binary_name" = "imu-viz" ]; then
  host_target="${HOST_TARGET:-$(rustc -vV | sed -n 's/^host: //p')}"
  cargo build \
    --manifest-path "$workspace_root/Cargo.toml" \
    --target "$host_target" \
    -p imu-viz
  exec "$workspace_root/target/$host_target/debug/imu-viz" "$@"
fi

export CARGO_TERM_COLOR=never
export CARGO_TERM_PROGRESS_WHEN=never
export CLICOLOR=0
export NO_COLOR=1

exec probe-rs run \
  --disable-progressbars \
  --chip RP235x \
  --rtt-scan-memory \
  "$binary_path" \
  "$@"
