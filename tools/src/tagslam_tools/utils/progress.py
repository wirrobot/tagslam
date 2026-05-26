"""Progress bar helper wrapping rich.progress."""

from __future__ import annotations

from collections.abc import Iterator
from typing import TypeVar

from rich.progress import (
    BarColumn,
    Progress,
    SpinnerColumn,
    TaskProgressColumn,
    TextColumn,
    TimeElapsedColumn,
)

T = TypeVar("T")


def track_progress(
    items: list[T],
    *,
    description: str = "Processing",
    total: int | None = None,
) -> Iterator[T]:
    """Yield items from *items* while showing a progress bar."""
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        TaskProgressColumn(),
        TimeElapsedColumn(),
    ) as progress:
        task_id = progress.add_task(description, total=total if total is not None else len(items))
        for item in items:
            yield item
            progress.advance(task_id)
