#!/usr/bin/env bash
set -euo pipefail
[[ $(uname -s) == Linux && $(uname -m) == x86_64 ]] || { echo 'Linux x86_64 required'; exit 2; }
command -v conda >/dev/null || { echo 'Install Miniforge/Conda first. No system Python is modified.'; exit 2; }
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
target=${1:-"$HOME/.local/share/tendon-libigl-sofa2506"}
[[ ! -e "$target" ]] || { echo "Refusing existing environment: $target"; exit 2; }
# Exact183-package linux-64/noarch solution, including URL hashes. No re-solve
# against a newer SOFA or a different Python ABI during target installation.
conda create --yes --prefix "$target" --file "$here/setup/linux64-explicit.txt"
mkdir -p "$here/.install"
printf '%s\n' "$target" > "$here/.install/env_path.txt"
conda list --prefix "$target" --explicit > "$here/.install/conda-explicit-linux64.txt"
conda list --prefix "$target" --json > "$here/.install/conda-packages.json"
echo 'Environment resolved/installed; this is not native-build or physics validation.'
echo 'Next: source setup/activate.sh && bash setup/build.sh && python setup/doctor.py'
