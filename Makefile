.PHONY: test lint typecheck check

test:
	cd tools && env -u PYTHONPATH uv run pytest

lint:
	cd tools && uv run ruff check src tests

format-check:
	cd tools && uv run ruff format --check src tests

typecheck:
	cd tools && env -u PYTHONPATH uv run mypy src tests --python-version 3.10

check: lint format-check typecheck test
