"""Static repository checks; not a substitute for ROS launch or hardware tests."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


root = Path(__file__).resolve().parents[1]
counts = {}
for suffix, validate in (
    ("*.py", ast.parse),
    ("*.yaml", yaml.safe_load),
    ("*.xml", ET.fromstring),
    ("*.xacro", ET.fromstring),
):
    files = list((root / "src").rglob(suffix))
    for file in files:
        validate(file.read_text(encoding="utf-8"))
    counts[suffix] = len(files)
yaml.safe_load((root / ".github/workflows/ci.yml").read_text(encoding="utf-8"))
print("Static syntax checks passed:", counts)
