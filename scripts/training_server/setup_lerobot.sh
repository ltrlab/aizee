#!/bin/bash
# LeRobot at openpi's pinned revision (dataset format v2.1), for the Minerva LeRobot export and ACT training.
set -euo pipefail
cd "${AIZEE_SIM:-$HOME/aizee-sim}"
python3.12 -m venv .venv-lerobot
. .venv-lerobot/bin/activate
pip install --upgrade pip wheel
pip install torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
GIT_LFS_SKIP_SMUDGE=1 pip install "lerobot @ git+https://github.com/huggingface/lerobot@0cf864870cf29f4738d3ade893e6fd13fbd7cdb5" "torchcodec==0.2.1" "torch==2.6.0" "torchvision==0.21.0"
# This revision predates PyAV 15 (AVRational base_rate), datasets 4 (Column) and huggingface-hub 1: pin what it was written against.
pip install --only-binary=:all: av==14.2.0 "datasets<4" "huggingface-hub<1" "diffusers<0.36"
python -c "import lerobot, torch, torchcodec; from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION; print(\"OK\", CODEBASE_VERSION, torch.__version__, torch.cuda.is_available())"
