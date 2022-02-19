"""
    scap.browser
    ~~~~~~~~
    Provides interactive browsing of scap related data such as history logs.
"""

import curses
from prettytable import PrettyTable


def browse(browseable):
    """
    Interactively browses a browseable object, i.e. an object that responds to
    `__prettytable__` and returns both a PrettyTable and a source collection
    with entries that correspond to table rows by index.
    """
    return curses.wrapper(lambda win: browse_in(browseable, win))


def browse_in(browseable, window):
    """
    Interactively browses a browseable object using an existing curses window.
    """
    return browser_for(browseable).browse_in(window)


def browser_for(browseable):
    if TableBrowser.isbrowseable(browseable):
        return TableBrowser.create(browseable)

    raise TypeError("argument is not browseable")


def isbrowseable(obj):
    return TableBrowser.isbrowseable(obj)


class TableBrowser:
    """
    Provides interactive browsing of a PrettyTable and selection of an object
    from the table's data source.
    """

    table = None
    data = []
    browser_cache = {}
    controls = {
        "prev": {curses.KEY_UP, ord('k')},
        "next": {curses.KEY_DOWN, ord('j')},
        "prev_page": {curses.KEY_PPAGE},
        "next_page": {curses.KEY_NPAGE},
        "select": {ord('\n'), ord(' ')},
        "inspect": {ord('i'), curses.KEY_RIGHT},
        "back": {ord('q'), curses.KEY_LEFT},
    }
    style = {
        "top_window_divisor": lambda: 3,
        "header": lambda: curses.A_DIM | curses.A_UNDERLINE,
        "selected": lambda: curses.A_STANDOUT,
        "footer": lambda: curses.A_STANDOUT,
    }
    help = "arrows or j/k: move, <enter>: select item, i: info, q: quit/back"

    def __init__(self, table, data, controls={}, style={}, help=None):
        self.table = table
        self.data = data
        self.browser_cache = {}
        self.controls = {**self.controls, **controls}
        self.style = {**self.style, **style}

        if help is not None:
            self.help = help

    @classmethod
    def create(cls, browseable):
        table, data = browseable.__prettytable__()

        if not isinstance(table, PrettyTable):
            raise NotImplementedError(
                "argument's __prettytable__() did not return a PrettyTable"
            )

        table.border = False

        return cls(table, data)

    @staticmethod
    def isbrowseable(obj):
        """
        Returns whether the object appears to be browseable by this class.
        """
        return hasattr(obj, '__prettytable__')

    def browse_in(self, win):
        """
        Interactively browse the table in an existing curses window.
        """

        if not self.table.rowcount:
            return None

        sel_row = 0

        def visible_rows():
            return win.getmaxyx()[0] - 1

        navigation = {
            "prev": lambda r: max(0, r - 1),
            "next": lambda r: min(self.table.rowcount - 1, r + 1),
            "prev_page": lambda r: max(0, r - visible_rows()),
            "next_page": lambda r: min(self.table.rowcount - 1, r + visible_rows()),
        }

        nav_dispatch = {}
        for nav in navigation:
            for key in self.controls[nav]:
                nav_dispatch[key] = navigation[nav]

        while True:
            self.redraw(win, sel_row)
            key = win.getch()

            if key in nav_dispatch:
                sel_row = nav_dispatch[key](sel_row)

            elif key in self.controls["select"]:
                return self.data[sel_row]

            elif key in self.controls["inspect"]:
                if isbrowseable(self.data[sel_row]):
                    self._subbrowser(sel_row).browse_in(win)

            elif key in self.controls["back"]:
                return None

    def redraw(self, win, sel_row, force_single=False, show_help=True,
               highlight_selected=True):
        """
        Draws the table to the given window with the given selected row. If
        the selected row also corresponds to a browseable object, the window
        is split into two panes and the selected object is drawn into the
        second.
        """
        height, width = win.getmaxyx()

        if not force_single and isbrowseable(self.data[sel_row]):
            top_height = sum(divmod(height, self.style["top_window_divisor"]()))
            top_win = win.derwin(top_height, width, 0, 0)
            bottom_win = win.derwin(height - top_height, width,
                                    top_height, 0)

            self.redraw(top_win, sel_row,
                        force_single=True,
                        show_help=False)
            self._subbrowser(sel_row).redraw(bottom_win, 0,
                                             force_single=True,
                                             highlight_selected=False)

            return

        inner_height = height
        inner_offset = 0

        if self.table.header:
            inner_height -= 1
            inner_offset += 1

        if show_help:
            inner_height -= 1

        # perform some nice midline oriented scrolling
        midline, _ = divmod(inner_height, 2)
        start = max(0, min(sel_row - midline, self.table.rowcount - inner_height))

        body = self.table.get_string(start=start, end=start + inner_height)
        rows = body.splitlines()

        if self.table.header:
            header = rows.pop(0)
            win.addstr(0, 0, header, self.style["header"]())
            win.clrtoeol()

        sel_y = sel_row - start + (1 if self.table.header else 0)
        nrows = len(rows)
        row = 0
        for y in range(inner_offset, inner_offset + inner_height):
            if row < nrows:
                if highlight_selected and y == sel_y:
                    win.addnstr(y, 0, rows[row], width,
                                self.style["selected"]())
                else:
                    win.addnstr(y, 0, rows[row], width)
            else:
                win.move(y, 0)

            win.clrtoeol()
            row += 1

        if show_help:
            win.addnstr(height - 1, 0, self.help, width,
                        self.style["footer"]())
            win.clrtoeol()

        win.refresh()

    def _subbrowser(self, row):
        if row not in self.browser_cache:
            self.browser_cache[row] = browser_for(self.data[row])
        return self.browser_cache[row]
