'''
run all of the tests in this directory with:
from the main ViamGreeter directory, run:
pytest tests/*

--
Shared pytest setup: lets the tests import the project's modules (gestures,
tracking, detect_person) from the repository root, wherever pytest is run from.
'''

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
