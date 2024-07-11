import curses

from .tree import ProcessingTimeTree
from .utils import abbreviate_topic
from .utils import wrap_lines


def print_tree(prefix: str, topic_name: str, tree: ProcessingTimeTree, stdscr: curses.window):
    stdscr.clear()
    height, width = stdscr.getmaxyx()
    stdscr.addstr(0, 0, prefix[: width - 2], curses.color_pair(2))
    topic_showing = (abbreviate_topic(topic_name) if len(topic_name) > width else topic_name)[
        : width - 2
    ]
    stdscr.addstr(1, 0, topic_showing, curses.color_pair(1))
    tree_lines = wrap_lines(tree.to_lines(), width, height - 2)
    for i, line in enumerate(tree_lines):
        stdscr.addstr(i + 2, 1, line)
    stdscr.refresh()
