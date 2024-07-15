import curses
from typing import List
from typing import Optional

from .utils import abbreviate_topic
from .utils import wrap_topic_name


def select_topic(stdscr: curses.window, topics: List[str]) -> Optional[str]:
    curses.curs_set(0)  # Hide the cursor
    curses.start_color()  # Enable color support
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)  # Define color pair
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)  # Define red color pair

    current_topic = 0
    start_index = 0
    max_topics = 8

    while True:
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        # Check if the terminal window is too small
        if (
            width < max(len(abbreviate_topic(topic)) for topic in topics) + 2
            or height < max_topics + 2
        ):
            error_msg = "Terminal window too small. Please resize."
            stdscr.addstr(height // 2, width // 2 - len(error_msg) // 2, error_msg)
            stdscr.refresh()
            key = stdscr.getch()
            if key in [ord("q"), ord("Q")]:
                return None
            continue

        # Display the full selected topic in red at the top, with wrapping if necessary
        full_topic = topics[current_topic]
        lines = wrap_topic_name(full_topic, width - 2)

        for i, line in enumerate(lines):
            stdscr.attron(curses.color_pair(2))
            stdscr.addstr(i, 1, line)
            stdscr.attroff(curses.color_pair(2))

        # Display the topics
        for idx in range(start_index, min(start_index + max_topics, len(topics))):
            abbreviated_option = abbreviate_topic(topics[idx])[: width - 2]  # Truncate if necessary
            x = width // 2 - len(abbreviated_option) // 2
            y = height // 2 - max_topics // 2 + idx - start_index + len(lines)
            if idx == current_topic:
                stdscr.attron(curses.color_pair(1))
                stdscr.addstr(y, x, abbreviated_option)
                stdscr.attroff(curses.color_pair(1))
            else:
                stdscr.addstr(y, x, abbreviated_option)

        # Display navigation buttons if needed
        if start_index + max_topics < len(topics):
            string = "Next>"
            stdscr.addstr(height - 1, width - len(string) - 1, string)
        if start_index > 0:
            string = "<Prev"
            stdscr.addstr(height - 1, 0, string)

        stdscr.refresh()

        # Handle user input
        key = stdscr.getch()
        if key == curses.KEY_UP and current_topic > 0:
            current_topic -= 1
            if current_topic < start_index:
                start_index -= 1
        elif key == curses.KEY_DOWN and current_topic < len(topics) - 1:
            current_topic += 1
            if current_topic >= start_index + max_topics:
                start_index += 1
        elif key in [curses.KEY_ENTER, 10, 13]:
            return topics[current_topic]
        elif key == curses.KEY_RIGHT and start_index + max_topics < len(topics):
            start_index += max_topics
            current_topic = start_index
        elif key == curses.KEY_LEFT and start_index > 0:
            start_index -= max_topics
            current_topic = start_index
