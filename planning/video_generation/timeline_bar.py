"""Render a recording-time strip synchronized with the 1x video assets."""

from bisect import bisect_left
from bisect import bisect_right
from datetime import datetime
from zoneinfo import ZoneInfo

import cv2
import numpy as np

NSEC = 1_000_000_000
JST = ZoneInfo("Asia/Tokyo")
HEIGHT = 120
CONTIGUOUS_GAP_NS = NSEC
BACKGROUND = (24, 28, 31)  # BGR
UNKNOWN = (56, 60, 64)
HATCH = (105, 112, 118)
MANUAL = (160, 165, 170)
AUTOWARE = (218, 157, 75)
MPPI = (166, 213, 139)
WHITE = (244, 247, 249)
MUTED = (187, 197, 204)


def format_elapsed(seconds: int) -> str:
    hours, remainder = divmod(max(0, seconds), 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def mode_spans(
    bags, samples: list[tuple[int, bool]], timeline
) -> list[tuple[int, int, bool | None]]:
    """Carry sparse state across adjacent parts, but not recording gaps."""
    samples = sorted(samples, key=lambda item: item[0])
    stamps = [stamp for stamp, _ in samples]
    spans = []
    previous_end = None
    previous_state = None
    for bag in bags:
        state = (
            previous_state
            if previous_end is not None and bag.start - previous_end <= CONTIGUOUS_GAP_NS
            else None
        )
        start = max(bag.start, timeline.start)
        end = min(bag.end, timeline.end)
        if start >= end:
            if bag.end <= timeline.start:
                last_index = bisect_right(stamps, bag.end) - 1
                if last_index >= 0 and stamps[last_index] >= bag.start:
                    state = samples[last_index][1]
                previous_end = bag.end
                previous_state = state
            continue
        cursor = start
        index = bisect_left(stamps, bag.start)
        while index < len(samples) and stamps[index] <= bag.end:
            stamp, next_state = samples[index]
            if stamp < start:
                state = next_state
            elif stamp < end:
                if stamp > cursor:
                    spans.append((cursor, stamp, state))
                cursor = stamp
                state = next_state
            else:
                break
            index += 1
        if cursor < end:
            spans.append((cursor, end, state))
        previous_end = bag.end
        previous_state = state
    return spans


def track_x(stamp: int, timeline, left: int, right: int) -> int:
    duration = max(1, timeline.end - timeline.start)
    offset = max(0, min(duration, stamp - timeline.start))
    return left + (offset * (right - left) + duration // 2) // duration


def _draw_text(image, text: str, x: int, y: int, scale: float, color=WHITE) -> None:
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _static_bar(timeline, spans, activations, width: int) -> tuple[np.ndarray, int, int]:
    image = np.full((HEIGHT, width, 3), BACKGROUND, dtype=np.uint8)
    left, right = 190, width - 28
    cv2.line(image, (0, 0), (width - 1, 0), (64, 73, 80), 2)
    _draw_text(image, "MODE", 28, 68, 0.62, MUTED)
    _draw_text(image, "MPPI", 28, 99, 0.62, MUTED)
    cv2.rectangle(image, (left, 52), (right, 74), UNKNOWN, -1)
    for x in range(left, right, 12):
        cv2.line(image, (x, 74), (min(x + 12, right), 52), HATCH, 1, cv2.LINE_AA)
    cv2.rectangle(image, (left, 83), (right, 105), (44, 54, 51), -1)
    for start, end, state in spans:
        if state is None:
            continue
        x1 = track_x(start, timeline, left, right)
        x2 = track_x(end, timeline, left, right)
        cv2.rectangle(image, (x1, 52), (max(x1, x2 - 1), 74), AUTOWARE if state else MANUAL, -1)
    for start, end in activations:
        x1 = track_x(max(start, timeline.start), timeline, left, right)
        x2 = track_x(min(end, timeline.end), timeline, left, right)
        if x2 >= x1 and start < timeline.end and end > timeline.start:
            # Subpixel activations get an occurrence tick without implying duration.
            cv2.rectangle(image, (x1, 83), (max(x1, x2 - 1), 105), MPPI, -1)
    for y1, y2 in ((52, 74), (83, 105)):
        cv2.rectangle(image, (left, y1), (right, y2), (105, 117, 122), 1)
    for x, color, label in (
        (1690, AUTOWARE, "AUTO"),
        (1850, MANUAL, "MANUAL"),
        (2025, HATCH, "UNKNOWN"),
        (2230, MPPI, "MPPI"),
    ):
        cv2.rectangle(image, (x, 18), (x + 16, 31), color, -1)
        _draw_text(image, label, x + 23, 30, 0.46, MUTED)
    return image, left, right


def render_timeline_bar(
    bags, timeline, mode_samples, activations, path, writer_for, width: int
) -> None:
    spans = mode_spans(bags, mode_samples, timeline)
    base, left, right = _static_bar(timeline, spans, activations, width)
    starts = [start for start, _, _ in spans]
    writer = writer_for(path, (width, HEIGHT))
    try:
        for index in range(timeline.count):
            stamp = timeline.stamp(index)
            image = base.copy()
            wall_time = datetime.fromtimestamp(stamp // NSEC, JST)
            _draw_text(image, wall_time.strftime("%Y-%m-%d %H:%M:%S JST"), 28, 32, 0.68)
            elapsed = (stamp - timeline.start) // NSEC
            total = (timeline.end - timeline.start) // NSEC
            _draw_text(
                image, f"ELAPSED {format_elapsed(elapsed)} / {format_elapsed(total)}", 570, 32, 0.68
            )
            lookup_stamp = min(stamp, timeline.end - 1)
            span_index = bisect_right(starts, lookup_stamp) - 1
            state = None
            if span_index >= 0 and lookup_stamp < spans[span_index][1]:
                state = spans[span_index][2]
            label = (
                "AUTOWARE IN CONTROL"
                if state is True
                else "MANUAL / NO CONTROL" if state is False else "MODE UNKNOWN"
            )
            _draw_text(image, label, 1130, 32, 0.62)
            cursor = track_x(stamp, timeline, left, right)
            cv2.line(image, (cursor, 46), (cursor, 108), WHITE, 3, cv2.LINE_AA)
            cv2.fillConvexPoly(
                image,
                np.array([[cursor - 7, 43], [cursor + 7, 43], [cursor, 50]], dtype=np.int32),
                WHITE,
            )
            writer.write(image)
    finally:
        writer.release()
