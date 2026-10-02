.PHONY: test build lint
PYTHON ?= python3

test:
	PYTHONPATH=src/escape_robot_base:src/escape_robot_guidance $(PYTHON) -m unittest discover -s src/escape_robot_base/test -p 'test_*.py'
	PYTHONPATH=src/escape_robot_base:src/escape_robot_guidance $(PYTHON) -m unittest discover -s src/escape_robot_guidance/test -p 'test_*.py'
	PYTHONPATH=src/escape_robot_perception $(PYTHON) -m unittest discover -s src/escape_robot_perception/test -p 'test_*.py'

build:
	colcon build --symlink-install

lint:
	$(PYTHON) -m compileall -q src
