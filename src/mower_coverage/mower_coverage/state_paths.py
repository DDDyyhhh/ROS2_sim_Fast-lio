"""Runtime state paths with read compatibility for historical home files."""

from pathlib import Path


STATE_DIR = Path.home() / '.local' / 'state' / 'mower_coverage'


def state_file(name):
    return str(STATE_DIR / name)


def ensure_parent(path):
    Path(path).expanduser().parent.mkdir(parents=True, exist_ok=True)


def readable_path(path, legacy_name):
    expanded = Path(path).expanduser()
    if expanded.exists():
        return str(expanded)
    legacy = Path.home() / legacy_name
    return str(legacy) if legacy.exists() else str(expanded)
