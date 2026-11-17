#!/usr/bin/env python3
"""Convert datasets recorded outside HiveBoard into ``datahive_episode_v1``.

Run with the repo venv:

    .venv/Scripts/python.exe tools/convert_dataset.py --help
    .venv/Scripts/python.exe tools/convert_dataset.py probe --source victorvanhalst/hive_botao_100
    .venv/Scripts/python.exe tools/convert_dataset.py profile --samples out/ --profile so101
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dataset_conversion.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
