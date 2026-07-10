#!/usr/bin/env bash
set -euo pipefail

CLI="${STM32_PROGRAMMER_CLI:-/opt/ST/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI}"

if [ "$#" -eq 0 ]; then
  CONNECT_ARGS=(port=SWD mode=UR reset=HWrst)
else
  CONNECT_ARGS=("$@")
fi

"$CLI" -c "${CONNECT_ARGS[@]}" -ob displ
