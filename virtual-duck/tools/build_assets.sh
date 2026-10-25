#!/usr/bin/env sh
# Builds the app's duck: the model, the motions and the voices, into its assets/duck.
#
# None of it is committed. It is rebuilt from pinned sources — microduck_rl's model, the official
# policy set Cargo.toml pins, the voices in tools/voices singing the robots' chorale score — and the
# meshes among it are CC BY-NC-SA rather than this repository's licence, so they travel as a recipe,
# not as files.
#
#   virtual-duck/tools/build_assets.sh
#
# Needs a microduck_rl checkout beside this repository, or wherever DUCK_SIM_RL points — the same
# convention as scripts/duck-sim — plus uv (or python3 -m venv) and ffmpeg.
#
# Runs at low priority with small thread pools: recording is a minute of CPU, and nothing about it
# is urgent enough to take a machine from whatever else is running on it.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
rl=${DUCK_SIM_RL:-$(cd "$repo/.." && pwd)/microduck_rl}
out="$here/../android/app/src/main/assets/duck"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2

if [ ! -f "$rl/scripts/infer_policy.py" ]; then
    echo "no microduck_rl at $rl — clone https://github.com/pollen-robotics/microduck_rl there," >&2
    echo "or point DUCK_SIM_RL at a checkout" >&2
    exit 1
fi

if [ ! -x "$here/.venv/bin/python" ]; then
    if command -v uv >/dev/null; then
        uv venv --python 3.12 "$here/.venv"
        uv pip install --python "$here/.venv/bin/python" -r "$here/requirements.txt"
    else
        python3 -m venv "$here/.venv"
        "$here/.venv/bin/pip" install -r "$here/requirements.txt"
    fi
fi
py="$here/.venv/bin/python"

mkdir -p "$out"
nice -n 19 "$py" "$here/export_model.py" --rl "$rl" --out "$out"
nice -n 19 "$py" "$here/record_motions.py" --rl "$rl" --out "$out"
nice -n 19 "$py" "$here/render_voices.py" --out "$out"
