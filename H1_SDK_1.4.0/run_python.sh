#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Probe without importing site packages or inheriting Python module paths.
is_python310() {
  command -v "$1" >/dev/null 2>&1 &&
    "$1" -I -S -c 'import sys; sys.exit(0 if sys.implementation.name == "cpython" and sys.version_info[:2] == (3, 10) else 1)' \
      >/dev/null 2>&1
}

select_python() {
  local candidate
  for candidate in "$@"; do
    if is_python310 "$candidate"; then
      python="$candidate"
      return 0
    fi
  done
  return 1
}

python=${PYTHON3:-}
if [[ -n "$python" ]]; then
  # An explicit override must be valid; do not silently select another Python.
  if ! is_python310 "$python"; then
    echo "PYTHON3 must name an executable CPython 3.10 interpreter: $python" >&2
    exit 2
  fi
else
  # sudo usually replaces HOME and PATH. Resolve the original login user's home.
  sdk_user=${SUDO_USER:-$(id -un)}
  sdk_user_home=$(getent passwd "$sdk_user" | cut -d: -f6) || sdk_user_home=""
  conda_envs=()
  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    conda_envs+=("${CONDA_PREFIX%/}")
  fi
  if [[ -n "$sdk_user_home" ]]; then
    # Conda records environments installed outside the usual directories here.
    if [[ -r "$sdk_user_home/.conda/environments.txt" ]]; then
      while IFS= read -r env_dir || [[ -n "$env_dir" ]]; do
        env_dir=${env_dir%$'\r'}
        env_dir=${env_dir%/}
        if [[ -n "$env_dir" ]]; then
          conda_envs+=("$env_dir")
        fi
      done < "$sdk_user_home/.conda/environments.txt"
    fi
    for conda_root in \
      "$sdk_user_home/miniconda3" "$sdk_user_home/anaconda3" \
      "$sdk_user_home/miniforge3" "$sdk_user_home/mambaforge" \
      "$sdk_user_home/.conda"
    do
      for env_dir in "$conda_root"/envs/*; do
        if [[ -d "$env_dir" ]]; then
          conda_envs+=("$env_dir")
        fi
      done
      if [[ -x "$conda_root/bin/python" ]]; then
        conda_envs+=("$conda_root")
      fi
    done
  else
    echo "Warning: cannot resolve home for $sdk_user; Conda directory discovery is limited." >&2
  fi

  # Check every zerith candidate before considering PATH or differently named envs.
  for env_dir in "${conda_envs[@]}"; do
    if [[ "${env_dir##*/}" == zerith ]] && select_python "$env_dir/bin/python"; then
      break
    fi
  done
  if [[ -z "$python" ]]; then
    select_python python3.10 python3 python || true
  fi
  if [[ -z "$python" ]]; then
    for env_dir in "${conda_envs[@]}"; do
      if select_python "$env_dir/bin/python"; then
        break
      fi
    done
  fi
  if [[ -z "$python" ]]; then
    echo "No compatible CPython 3.10 found in zerith, PATH, or the user's Conda environments." >&2
    echo "Install CPython 3.10, or specify PYTHON3=/absolute/path/to/python." >&2
    exit 2
  fi
fi

python_build_lib="$root/SDK_python/build/lib"
python_release_lib="$root/SDK_python/lib"
python_deps="$root/SDK_python/python_deps"
export PYTHONPATH="$python_build_lib:$python_release_lib:$python_deps${PYTHONPATH:+:$PYTHONPATH}"
export ROBOT_LOG_ROOT="${ROBOT_LOG_ROOT:-/tmp/h1-sdk-logs-$(id -u)}"

exec "$python" "$@"
