#!/usr/bin/env bash
set -euo pipefail

CLI="${STM32_PROGRAMMER_CLI:-/opt/ST/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI}"

if [ "$#" -eq 0 ]; then
  CONNECT_ARGS=(port=SWD mode=UR reset=HWrst)
else
  CONNECT_ARGS=("$@")
fi

echo "Programming Option Bytes:"
echo "  nSWBOOT0=0  -> BOOT0 is taken from option bit nBOOT0, not PB8/BOOT0 pin"
echo "  nBOOT0=1    -> software BOOT0 value is 1"
echo

"$CLI" -c "${CONNECT_ARGS[@]}" -ob nSWBOOT0=0 nBOOT0=1
"$CLI" -c "${CONNECT_ARGS[@]}" -ob displ
"$CLI" -c "${CONNECT_ARGS[@]}" -rst
