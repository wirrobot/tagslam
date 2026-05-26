.PHONY: test lint typecheck check

PYSRC = tools/src/tagslam_tools tools/tests

test:
	env -u PYTHONPATH uv run pytest $(PYSRC)

lint:
	uv run ruff check $(PYSRC)

format-check:
	uv run ruff format --check $(PYSRC)

typecheck:
	env -u PYTHONPATH uv run mypy $(PYSRC)

check: lint format-check typecheck test
