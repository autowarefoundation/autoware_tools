import curses
from itertools import chain
from typing import List

from .tree import ProcessingTimeTree
from .utils import abbreviate_topic
from .utils import wrap_lines


def print_trees(
    prefix: str,
    topic_name: str,
    trees: List[ProcessingTimeTree],
    stdscr: curses.window,
    show_comment: bool = False,
    summarize: bool = False,
):
    stdscr.clear()
    height, width = stdscr.getmaxyx()
    stdscr.addstr(0, 0, prefix[: width - 2], curses.color_pair(2))
    topic_showing = (abbreviate_topic(topic_name) if len(topic_name) > width else topic_name)[
        : width - 2
    ]
    stdscr.addstr(1, 0, topic_showing, curses.color_pair(1))
    tree_lines = list(
        chain.from_iterable(tree.to_lines(show_comment, summarize) + [""] for tree in trees)
    )
    tree_lines = wrap_lines(tree_lines, width, height - 2)
    for i, line in enumerate(tree_lines):
        stdscr.addstr(i + 2, 1, line)
    stdscr.addstr(height - 1, 0, "'q' => quit. 'c' => show comment. 's' => summarize. 'y' => copy."[: width - 2])
    stdscr.refresh()

    return "".join([line + "\n" for line in tree_lines])
