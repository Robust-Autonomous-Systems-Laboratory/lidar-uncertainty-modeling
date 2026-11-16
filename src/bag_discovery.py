import re
from dataclasses import dataclass
from pathlib import Path

# Bags follow 'test##_lidar_name.bag': ## is the repeated-trial counter and lidar_name
# identifies a distinct lidar unit / operating mode (e.g. 'vlp32_last', 'ouster_1024x20').
BAG_NAME_PATTERN = re.compile(r"^test(\d+)_(.+)\.bag$")


@dataclass(frozen=True)
class TrialBag:
    path: Path
    trial: int
    lidar_name: str


def discover_bags(directory: str | Path) -> tuple[dict[str, list[TrialBag]], list[Path], list[str]]:
    """Find trial bags under a directory (recursively) and group them by lidar name.

    :return: (groups, skipped, warnings)
        groups:   {lidar_name: [TrialBag, ...]} sorted by lidar name, trials ascending
        skipped:  .bag files that do not follow the naming convention
        warnings: human-readable problems, e.g. a trial number repeated for one lidar
    """
    groups: dict[str, list[TrialBag]] = {}
    skipped: list[Path] = []
    warnings: list[str] = []

    for path in sorted(Path(directory).rglob("*.bag")):
        match = BAG_NAME_PATTERN.match(path.name)
        if not match:
            skipped.append(path)
            continue
        bag = TrialBag(path=path, trial=int(match.group(1)), lidar_name=match.group(2))
        groups.setdefault(bag.lidar_name, []).append(bag)

    for lidar_name, bags in groups.items():
        bags.sort(key=lambda b: b.trial)
        trials = [b.trial for b in bags]
        duplicates = sorted({t for t in trials if trials.count(t) > 1})
        if duplicates:
            # Trial numbers name the output files, so a duplicate would overwrite results.
            warnings.append(f"{lidar_name}: trial number(s) {duplicates} appear more than once; "
                            f"only the first file for each is kept")
            unique: dict[int, TrialBag] = {}
            for b in bags:
                unique.setdefault(b.trial, b)
            groups[lidar_name] = list(unique.values())

    return dict(sorted(groups.items())), skipped, warnings
