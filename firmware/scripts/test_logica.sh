#!/usr/bin/env bash
# Corre en la compu las pruebas de la logica del firmware que no dependen de ESP-IDF
# (hoy: el PID de centrado). No hace falta el robot ni el contenedor de ESP-IDF.
set -euo pipefail
cd "$(dirname "$0")/../micro_ros/logica/test"
salida="$(mktemp -d)"
trap 'rm -rf "$salida"' EXIT
gcc -std=c11 -Wall -Wextra -Werror -o "$salida/test_centrado_pid" test_centrado_pid.c -lm
"$salida/test_centrado_pid"
