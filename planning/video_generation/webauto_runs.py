"""Discover complete WebAuto recordings and download their filtered MCAP parts."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from datetime import timezone
import json
import logging
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

LOG = logging.getLogger("bag_video")
PART_NAME = re.compile(r"^(?P<stem>.+)_(?P<number>\d+)\.mcap$")
MCAP_MAGIC = b"\x89MCAP0\r\n"


@dataclass(frozen=True)
class LogPart:
    id: str
    name: str
    stem: str
    number: int
    start: datetime
    end: datetime

    @property
    def stream(self) -> str:
        # The final stem component identifies the recording stream (p0900 in
        # the current WebAuto filenames). Runs from other streams do not close it.
        return self.stem.rsplit("_", 1)[-1]


@dataclass(frozen=True)
class Run:
    stem: str
    parts: tuple[LogPart, ...]


def parse_datetime(value: str, option: str) -> datetime:
    try:
        normalized = value.replace("Z", "+00:00")
        # Python 3.10's fromisoformat accepts only some fractional-second
        # widths. WebAuto emits variable-width fractions (for example .99Z).
        fraction = re.fullmatch(r"(.+\.)(\d+)([+-]\d{2}:\d{2})", normalized)
        if fraction:
            normalized = fraction[1] + fraction[2][:6].ljust(6, "0") + fraction[3]
        result = datetime.fromisoformat(normalized)
    except (AttributeError, ValueError) as error:
        raise ValueError(f"{option} must be an ISO 8601 datetime with timezone") from error
    if result.tzinfo is None or result.utcoffset() is None:
        raise ValueError(f"{option} must include a timezone")
    return result


def datetime_ns(value: datetime) -> int:
    delta = value.astimezone(timezone.utc) - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return (delta.days * 86400 + delta.seconds) * 1_000_000_000 + delta.microseconds * 1000


def run_cli(command: list[str]) -> str:
    try:
        completed = subprocess.run(command, check=True, capture_output=True, text=True)
    except FileNotFoundError as error:
        raise RuntimeError("webauto CLI was not found in PATH") from error
    except subprocess.CalledProcessError as error:
        detail = (error.stderr or error.stdout or "").strip()
        raise RuntimeError(
            f"webauto command failed: {' '.join(command[:4])}: "
            f"{detail or f'exit status {error.returncode}'}"
        ) from error
    return completed.stdout


def search_parts(
    project_id: str, vehicle_id: str, start: datetime, search_end: datetime
) -> list[LogPart]:
    """Search through a later bound to find next-_0 markers."""
    if search_end <= start:
        raise ValueError("WebAuto search end must be after its start")
    parts: dict[str, LogPart] = {}
    page_token = ""
    seen_tokens: set[str] = set()
    while True:
        command = [
            "webauto",
            "data",
            "log-file",
            "search",
            "--project-id",
            project_id,
            "--vehicle-ids",
            vehicle_id,
            "--datetime-from",
            start.isoformat(),
            "--datetime-to",
            search_end.isoformat(),
            "--file-types",
            "rosbag",
            "--page-size",
            "100",
            "--output",
            "json",
            "--quiet",
        ]
        if page_token:
            command += ["--page-token", page_token]
        page = json.loads(run_cli(command))
        if isinstance(page, list):
            records, next_token = page, ""
        elif isinstance(page, dict):
            records = page.get("log_files", page.get("results", page.get("items")))
            next_token = page.get("next_page_token") or ""
        else:
            raise ValueError("Unexpected WebAuto log-file search response")
        if not isinstance(records, list):
            raise ValueError("WebAuto log-file search response has no log file list")
        for record in records:
            if not isinstance(record, dict):
                raise ValueError("WebAuto log-file search returned a non-object entry")
            name = record.get("log_file_name")
            if not isinstance(name, str):
                continue
            match = PART_NAME.fullmatch(Path(name).name)
            if not match:
                LOG.warning("Ignoring rosbag with unrecognized part name: %s", name)
                continue
            file_id = record.get("log_file_id") or record.get("id")
            if (
                not isinstance(file_id, str)
                or not file_id
                or Path(file_id).name != file_id
                or file_id in (".", "..")
            ):
                raise ValueError(f"WebAuto entry has no log file ID: {name}")
            part = LogPart(
                file_id,
                Path(name).name,
                match["stem"],
                int(match["number"]),
                parse_datetime(record["start_timestamp"], "start_timestamp"),
                parse_datetime(record["end_timestamp"], "end_timestamp"),
            )
            if part.end < part.start:
                raise ValueError(f"WebAuto entry ends before it starts: {name}")
            parts[file_id] = part
        if not next_token:
            return list(parts.values())
        if not isinstance(next_token, str) or next_token in seen_tokens:
            raise ValueError("WebAuto returned an invalid or repeated page token")
        seen_tokens.add(next_token)
        page_token = next_token


def select_runs(parts: list[LogPart], start: datetime, end: datetime) -> list[Run]:
    """Select fully contained runs closed by the next _0 on the same stream."""
    groups: dict[str, list[LogPart]] = {}
    for part in parts:
        groups.setdefault(part.stem, []).append(part)
    starts = sorted(
        (part for part in parts if part.number == 0),
        key=lambda part: (part.start, part.name),
    )
    selected: list[Run] = []
    for first in starts:
        if not start <= first.start < end:
            continue
        next_start = next(
            (part for part in starts if part.stream == first.stream and part.start > first.start),
            None,
        )
        if next_start is None:
            LOG.warning("Deferring %s: no later _0 confirms the recording ended", first.stem)
            continue
        ordered = sorted(groups[first.stem], key=lambda part: part.number)
        numbers = [part.number for part in ordered]
        if numbers != list(range(len(ordered))):
            LOG.warning(
                "Skipping %s: missing or duplicate numbered bag parts: %s", first.stem, numbers
            )
            continue
        if (
            any(part.start >= next_start.start for part in ordered)
            or ordered[-1].end > next_start.start
        ):
            LOG.warning("Skipping %s: bag part overlaps the next recording", first.stem)
            continue
        if any(right.start < left.end for left, right in zip(ordered, ordered[1:])):
            LOG.warning("Skipping %s: bag part times overlap", first.stem)
            continue
        if ordered[-1].end > end:
            LOG.info("Skipping %s: recording ends outside the requested range", first.stem)
            continue
        selected.append(Run(first.stem, tuple(ordered)))
    return selected


def find_mcap(root: Path) -> Path:
    candidates = []
    for path in root.rglob("*"):
        if path.is_file():
            with path.open("rb") as stream:
                if stream.read(len(MCAP_MAGIC)) == MCAP_MAGIC:
                    candidates.append(path)
    if len(candidates) != 1:
        raise ValueError(f"Expected one filtered MCAP in {root}; found {len(candidates)}")
    return candidates[0]


def pull_part(project_id: str, part: LogPart, topics: set[str], download_root: Path) -> Path:
    """Pull one source part; cache successful downloads by source log-file ID."""
    destination = download_root / part.id
    marker = destination / "download.json"
    if marker.is_file():
        record = json.loads(marker.read_text())
        cached = destination / record["mcap"]
        if record.get("topics") == sorted(topics) and cached.is_file():
            with cached.open("rb") as stream:
                if stream.read(len(MCAP_MAGIC)) == MCAP_MAGIC:
                    LOG.info("Reusing filtered bag: %s", cached)
                    return cached
    download_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f"pull_{part.number}_", dir=download_root))
    try:
        command = [
            "webauto",
            "data",
            "log-file",
            "pull-filtered-rosbag",
            "--project-id",
            project_id,
            "--log-file-ids",
            part.id,
            "--topics",
            ",".join(sorted(topics)),
            "--target-dir",
            str(stage),
        ]
        LOG.info("Downloading %s (%d topics)", part.name, len(topics))
        run_cli(command)
        source = find_mcap(stage)
        if destination.exists():
            shutil.rmtree(destination)
        # A successful pull may have arbitrary nested directory names. Preserve
        # them, then record the relative MCAP location for reuse.
        relative = source.relative_to(stage)
        stage.rename(destination)
        marker.write_text(
            json.dumps({"source_id": part.id, "mcap": str(relative), "topics": sorted(topics)})
        )
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return destination / relative
