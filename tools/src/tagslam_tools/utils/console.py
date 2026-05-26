"""Rich Console, Theme, QStyle, and output helpers."""

from __future__ import annotations

from typing import Any

import questionary
from rich.console import Console
from rich.theme import Theme

PALETTE: dict[str, str] = {
    "primary": "#527590",
    "accent": "#feb298",
    "muted": "#cddcdf",
    "lightest": "#eeefea",
    "bg": "#ffffff",
}

_THEME = Theme(
    {
        "divider": PALETTE["muted"],
        "heading": f"{PALETTE['primary']} bold",
        "success": "bold green",
        "error": "bold red",
        "warn": PALETTE["accent"],
        "info": PALETTE["primary"],
        "category": f"{PALETTE['accent']} bold",
        "dim": PALETTE["muted"],
        "item": "white",
        "selected": f"{PALETTE['accent']} bold",
    }
)

console = Console(theme=_THEME, highlight=False)


def section() -> None:
    """Print a thin muted divider line."""
    console.print("─" * 60, style="divider")


def heading(text: str) -> None:
    """Print a bold primary-colour heading."""
    console.print(text, style="heading")


def make_choice(label: str, description: str, value: Any = None) -> questionary.Choice:
    """Create a menu option with left label and right muted description."""
    return questionary.Choice(
        title=[
            ("", f"  {label:<22}"),
            (f"fg:{PALETTE['muted']}", description),
        ],
        value=label if value is None else value,
    )


questionary_style = questionary.Style(
    [
        ("qmark", f"fg:{PALETTE['primary']} bold"),
        ("question", f"fg:{PALETTE['primary']} bold"),
        ("pointer", f"fg:{PALETTE['accent']} bold"),
        ("selected", f"fg:{PALETTE['accent']} bold"),
        ("separator", f"fg:{PALETTE['accent']} bold"),
        ("answer", f"fg:{PALETTE['primary']}"),
        ("instruction", f"fg:{PALETTE['muted']}"),
    ]
)
