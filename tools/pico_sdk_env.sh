#!/usr/bin/env bash

# Source this file before configuring the Pico SDK firmware build. An existing
# environment setting wins; otherwise use the common sibling checkout layout.
if [ -z "${PICO_SDK_PATH:-}" ]; then
    _robot_pico_sdk_candidate="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../pico-sdk" 2>/dev/null && pwd)"
    if [ -n "${_robot_pico_sdk_candidate}" ] && [ -d "${_robot_pico_sdk_candidate}" ]; then
        export PICO_SDK_PATH="${_robot_pico_sdk_candidate}"
    else
        echo "PICO_SDK_PATH is not set and no sibling pico-sdk checkout was found." >&2
        return 1
    fi
    unset _robot_pico_sdk_candidate
fi
