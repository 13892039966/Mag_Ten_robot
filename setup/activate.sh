#!/usr/bin/env bash
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then echo 'Use: source setup/activate.sh'; exit 2; fi
_tendon_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ -f "$_tendon_root/.install/env_path.txt" ]] || { echo 'Run bash setup/install.sh first'; return 2; }
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "$(cat "$_tendon_root/.install/env_path.txt")" || return 2
export SOFA_ROOT="$CONDA_PREFIX"
export PYTHONNOUSERSITE=1
unset PYTHONPATH SOFAPYTHON3_ROOT
export CMAKE_PREFIX_PATH="$CONDA_PREFIX"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
for _tendon_lib in "$CONDA_PREFIX"/plugins/*/lib "$CONDA_PREFIX"/lib/sofa/plugins/*/lib; do
 [[ -d "$_tendon_lib" ]] && export LD_LIBRARY_PATH="$_tendon_lib:$LD_LIBRARY_PATH"
done
export OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 OMP_NUM_THREADS=1
unset _tendon_root _tendon_lib
