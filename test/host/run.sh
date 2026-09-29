#!/usr/bin/env bash
#
# Compila e roda a verificacao da camada de controle no host, sob sanitizadores.
#
# Nao envolve a placa nem o PlatformIO: control/ e independente do Arduino, o que
# permite conferir a matematica do controle em segundos.
#
# Uso:  ./test/host/run.sh
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

# bounds-strict e o que detecta estouro em array membro de classe; o ASan sozinho
# nao pega esse caso, porque o acesso cai dentro do mesmo objeto.
g++ -std=c++11 -Wall -Wextra -Werror \
    -fsanitize=address,undefined -fsanitize=bounds-strict \
    -g -O1 \
    -I "$ROOT/include" \
    "$ROOT/test/host/test_control.cpp" \
    "$ROOT"/src/control/*.cpp \
    -o "$OUT/test_control"

"$OUT/test_control"
