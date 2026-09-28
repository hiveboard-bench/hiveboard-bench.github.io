#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON_BIN:-python3}"
cumotion_version="${CUMOTION_VERSION:-1.1.0}"
cuda_version="${CUMOTION_CUDA_VERSION:-13.0}"
machine="$(uname -m)"

if [[ "$machine" != "x86_64" ]]; then
  echo "This installer currently downloads NVIDIA's x86_64 Linux package; detected $machine." >&2
  exit 2
fi

case "$cuda_version" in
  12.6|13.0) ;;
  *) echo "Set CUMOTION_CUDA_VERSION to 12.6 or 13.0." >&2; exit 2 ;;
esac

python_tag="$($python_bin -c 'import sys; print(f"cp{sys.version_info.major}{sys.version_info.minor}")')"
cache_dir="$repo_root/.cache/cumotion"
archive_name="cumotion-${cumotion_version}-cuda${cuda_version}-x86_64.tar.gz"
archive_path="$cache_dir/$archive_name"
release_dir="$cache_dir/cumotion-${cumotion_version}-cuda${cuda_version}-x86_64"
download_url="https://github.com/nvidia-isaac/cumotion/releases/download/v${cumotion_version}/${archive_name}"

mkdir -p "$cache_dir"

if [[ ! -f "$archive_path" ]]; then
  echo "Downloading NVIDIA cuMotion ${cumotion_version} (${cuda_version}, x86_64)…"
  curl --fail --location --retry 3 "$download_url" --output "${archive_path}.part"
  mv "${archive_path}.part" "$archive_path"
fi

if [[ ! -d "$release_dir" ]]; then
  tar -xzf "$archive_path" -C "$cache_dir"
fi

wheel_path="$(find "$release_dir/python_wheels" -maxdepth 1 -type f \
  -name "cumotion-${cumotion_version}-${python_tag}-*.whl" -print -quit)"
if [[ -z "$wheel_path" ]]; then
  echo "The NVIDIA archive has no cuMotion wheel for Python ${python_tag}." >&2
  echo "Choose a Python version supported by this cuMotion release." >&2
  exit 2
fi

venv_python="$repo_root/.venv/bin/python"
if [[ ! -x "$venv_python" ]]; then
  "$python_bin" -m venv "$repo_root/.venv"
  venv_python="$repo_root/.venv/bin/python"
fi

"$venv_python" -m pip install -r "$repo_root/tools/requirements-simulation.txt"
"$venv_python" -m pip install "$wheel_path"
"$venv_python" -c 'from importlib.metadata import version; import cumotion, mujoco; print("cuMotion", version("cumotion")); print("MuJoCo", mujoco.__version__); print("cuMotion API ready:", hasattr(cumotion, "load_robot_from_file"))'

echo "Environment ready. Run: .venv/bin/python tools/test_cumotion_integration.py"
