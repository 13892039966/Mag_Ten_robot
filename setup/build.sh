#!/usr/bin/env bash
set -euo pipefail
[[ $(uname -s) == Linux && -n ${CONDA_PREFIX:-} ]] || { echo 'Activate the isolated Linux environment first'; exit 2; }
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cmake -S "$root/native" -B "$root/native/build" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build "$root/native/build" -j4
cmake -S "$root/native/libigl_adapter" -B "$root/native/libigl_adapter/build" -G Ninja \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DSOFA_BUILD_TESTS=OFF \
 -DPLUGIN_SOFADISTANCEGRID_CUDA=OFF -DCMAKE_INSTALL_RPATH="$CONDA_PREFIX/lib"
cmake --build "$root/native/libigl_adapter/build" -j4
test -f "$root/native/build/libTendonScheduler25.so"
test -f "$root/native/libigl_adapter/build/source/libSofaDistanceGrid.so"
echo 'Native build finished. Run python setup/doctor.py, then zero-input and short physics tests.'
