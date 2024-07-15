import curses
from typing import List


def abbreviate_topic(topic: str) -> str:
    parts = topic.split("/")
    abbreviated_parts = [part[0] if len(part) > 1 else part for part in parts[:-1]]
    return "/".join(abbreviated_parts + [parts[-1]])


def wrap_topic_name(text: str, width: int) -> List[str]:
    lines = []
    while len(text) > width:
        split_point = text.rfind("/", 0, width)
        if split_point == -1:
            split_point = width
        lines.append(text[:split_point])
        text = text[split_point:]
    lines.append(text)
    return lines


def wrap_lines(lines, width, height):
    return [line[:width] for line in lines][:height]


def exit_curses():
    curses.echo()
    curses.nocbreak()
    curses.endwin()


def init_curses() -> curses.window:
    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    curses.mousemask(curses.ALL_MOUSE_EVENTS)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_WHITE)
    return stdscr
