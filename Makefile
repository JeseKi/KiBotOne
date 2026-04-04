.PHONY: build dev lint test check

build:
	colcon build

check:
	mypy .
	ruff check --fix

test:
	python -m pytest . -q