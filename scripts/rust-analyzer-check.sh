#!/bin/sh

set -eu

if [ "$#" -lt 2 ]; then
  echo "usage: $0 <package-label> <saved-file>" >&2
  exit 1
fi

PACKAGE_ID=$1
SAVED_FILE=$2

PACKAGE_PATH=${PACKAGE_ID%%#*}
PACKAGE_NAME=$(basename "$PACKAGE_PATH")
SAVED_BASENAME=$(basename "$SAVED_FILE")
SAVED_STEM=${SAVED_BASENAME%.rs}
HOST_TARGET=$(rustc -vV | sed -n 's/^host: //p')
EMBEDDED_TARGET=thumbv8m.main-none-eabihf

if [ -z "$HOST_TARGET" ]; then
  echo "failed to detect rust host target" >&2
  exit 1
fi

if [ "$PACKAGE_NAME" = "imu-viz" ]; then
  exec cargo check -p "$PACKAGE_ID" --bin "$PACKAGE_NAME" --target "$HOST_TARGET" --message-format=json
fi

case "$SAVED_FILE" in
  */examples/*.rs)
    if [ "${SAVED_STEM#rp235x_}" != "$SAVED_STEM" ]; then
      exec cargo check -p "$PACKAGE_ID" --example "$SAVED_STEM" --target "$EMBEDDED_TARGET" --message-format=json
    fi

    exec cargo check -p "$PACKAGE_ID" --example "$SAVED_STEM" --target "$HOST_TARGET" --message-format=json
    ;;
  */build.rs)
    exec cargo check -p "$PACKAGE_ID" --target "$EMBEDDED_TARGET" --message-format=json
    ;;
  *)
    exec cargo check -p "$PACKAGE_ID" --lib --target "$EMBEDDED_TARGET" --message-format=json
    ;;
esac
