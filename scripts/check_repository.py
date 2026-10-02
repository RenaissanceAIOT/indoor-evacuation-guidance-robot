"""Static repository checks; not a substitute for ROS launch or hardware tests."""

import ast
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit
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

# Browser uploads can accidentally omit hidden files or nest the workspace.
for required in ("README.md", "LICENSE", ".gitignore", ".github/workflows/ci.yml", "src"):
    if not (root / required).exists():
        raise ValueError(f"missing required repository path: {required}")
packages = list((root / "src").glob("*/package.xml"))
if len(packages) != 5:
    raise ValueError(f"expected 5 ROS packages, found {len(packages)}")

markdown_files = [root / "README.md", root / "CONTRIBUTING.md", *sorted((root / "docs").glob("*.md"))]
local_links = 0
for document in markdown_files:
    for target in re.findall(r"!?\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)", document.read_text(encoding="utf-8")):
        link = urlsplit(target.strip("<>"))
        if link.scheme or link.netloc or not link.path:
            continue
        destination = (document.parent / unquote(link.path)).resolve()
        if not destination.is_relative_to(root) or not destination.exists():
            raise ValueError(f"broken local link in {document.relative_to(root)}: {target}")
        local_links += 1
print("Static syntax checks passed:", counts)
print(f"Repository layout passed: {len(packages)} ROS packages; {local_links} local Markdown links")
