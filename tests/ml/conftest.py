"""
conftest.py — adds the aizee repo root to sys.path so tests can import
python.training.* and python.nodes.* without installing as a package.
"""
import sys
from pathlib import Path

# Repo root = three levels up from tests/ml/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
