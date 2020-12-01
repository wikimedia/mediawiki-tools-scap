# -*- coding: utf-8 -*-
"""
    scap.terminal
    ~~~~~~~~~~~~~
    Text terminal output utilities

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import absolute_import

import collections

import fcntl
import io
import os
import struct
import sys
import termios

from contextlib import contextmanager

import curses
from curses import tigetstr, tparm


@contextmanager
def scroll_region_context(term, top=1, bottom=100):
    try:
        term.save().scroll_region(top, bottom)
        yield term
    finally:
        term.flush().scroll_region(1, term.height).flush().restore()


class TerminalIO(io.TextIOBase):
    """
TerminalIO represents a terminal (pty) and provides several convenience
methods for outputting terminal control sequences. Much of this code was
derived from the blessed library <https://github.com/jquast/blessed>.
The API for TerminalIO is different from the blessed Terminal API.

License:

Copyright (c) 2016 Mukunda Modell <mmodell@wikimedia.org>
Copyright (c) 2014 Jeff Quast
Copyright (c) 2011 Erik Rose

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

    _sugar = dict(
        save="sc",
        restore="rc",
        # 'clear' clears the whole screen.
        clear_eol="el",
        clear_bol="el1",
        clear_eos="ed",
        position="cup",  # deprecated
        enter_fullscreen="smcup",
        exit_fullscreen="rmcup",
        move="cup",
        move_x="hpa",
        move_y="vpa",
        move_left="cub1",
        move_right="cuf1",
        move_up="cuu1",
        move_down="cud1",
        hide_cursor="civis",
        normal_cursor="cnorm",
        reset_colors="op",  # oc doesn't work on my OS X terminal.
        normal="sgr0",
        reverse="rev",
        italic="sitm",
        no_italic="ritm",
        shadow="sshm",
        no_shadow="rshm",
        standout="smso",
        no_standout="rmso",
        subscript="ssubm",
        no_subscript="rsubm",
        superscript="ssupm",
        no_superscript="rsupm",
        underline="smul",
        no_underline="rmul",
        cursor_report="u6",
        cursor_request="u7",
        terminal_answerback="u8",
        terminal_enquire="u9",
    )

    def __init__(self, out=None, autoflush=True):
        super(TerminalIO, self).__init__()
        if out is None:
            out = sys.__stdout__

        self.cleanup_callbacks = []
        self.autoflush = autoflush

        try:
            stream_fd = (
                out.fileno()
                if hasattr(out, "fileno") and callable(out.fileno)
                else None
            )
        except io.UnsupportedOperation:
            stream_fd = None

        self._out = out
        self._kind = os.environ.get("TERM", "unknown")

        self._is_a_tty = stream_fd is not None and os.isatty(stream_fd)
        self._does_styling = self._is_a_tty

        if self._is_a_tty:
            try:
                self._init_descriptor = stream_fd
                curses.setupterm(self._kind, stream_fd)
            except curses.error:
                self._does_styling = False

        self.region_bottom = self.height
        self.region_top = 0

    def register_cleanup_callback(self, callback):
        self.cleanup_callbacks.append(callback)

    def write(self, *args):
        if not self._out:
            raise IOError("Cannot write to closed terminal stream.")

        for i in args:
            i = str(i)
            self._out.write(i)

        if self.autoflush:
            self._out.flush()
        return self

    def writeln(self, *args):
        self.write(*args)
        self._out.write("\r\n")
        self._out.flush()
        return self

    def close(self):
        if not self._out:
            raise IOError("Already closed")

        self.cleanup()
        self._out.flush()
        self._out = None

    def cleanup(self):
        """ call cleanup callbacks registered by other modules """
        for cb in self.cleanup_callbacks:
            cb(self)

        del self.cleanup_callbacks[:]

    def flush(self):
        self._out.flush()
        return self

    def nl(self):
        return self.write("\n")

    def cr(self):
        return self.move_y(0)

    def fg(self, color, text=None):
        self.setaf(color)
        if text:
            self.write(text).normal()
        return self

    @property
    def bottom(self):
        return min((self.region_bottom, self.height))

    @property
    def top(self):
        return max((self.region_top, 1))

    def scroll_region(self, top, bottom):
        self.region_top = max((top, 0))
        self.region_bottom = min((bottom, self.height))
        self.csr(top, bottom).move(bottom, 0)
        return self

    def scroll_forward(self, count=1):
        self.save().move(self.bottom, 0).write("\n" * count).restore()
        return self

    def scroll_reverse(self, count=1):
        self.save().move(self.top, 0).rin(count).restore()
        return self

    def __getattr__(self, name):
        capname = self._sugar.get(name, name)
        capstr = None
        if self._is_a_tty and self._does_styling:
            capstr = tigetstr(capname)

        def wrapper(*args):
            if capstr is None:
                return self
            val = tparm(capstr, *args)
            val = val.decode("latin1")
            self.write(val)
            return self

        return wrapper

    @property
    def kind(self):
        """
        Read-only property: Terminal kind determined on class initialization.
        :rtype: str
        """
        return self._kind

    @property
    def does_styling(self):
        """
        Read-only property: Whether this class instance may emit sequences.
        :rtype: bool
        """
        return self._does_styling

    @property
    def is_a_tty(self):
        """
        Read-only property: Whether :attr:`~.stream` is a terminal.
        :rtype: bool
        """
        return self._is_a_tty

    @property
    def height(self):
        """
        Read-only property: Height of the terminal (in number of lines).
        :rtype: int
        """
        return self._height_and_width().ws_row

    @property
    def width(self):
        """
        Read-only property: Width of the terminal (in number of columns).
        :rtype: int
        """
        return self._height_and_width().ws_col

    def _height_and_width(self):
        """
        Return a tuple of (terminal height, terminal width).
        If :attr:`stream` or :obj:`sys.__stdout__` is not a tty or does not
        support :func:`fcntl.ioctl` of :const:`termios.TIOCGWINSZ`, a window
        size of 80 columns by 25 rows is returned for any values not
        represented by environment variables ``LINES`` and ``COLUMNS``, which
        is the default text mode of IBM PC compatibles.
        :rtype: WINSZ
        WINSZ is a :class:`collections.namedtuple` instance, whose structure
        directly maps to the return value of the :const:`termios.TIOCGWINSZ`
        ioctl return value. The return parameters are:
        * ``ws_row``: width of terminal by its number of character cells.
        * ``ws_col``: height of terminal by its number of character cells.
        * ``ws_xpixel``: width of terminal by pixels (not accurate).
        * ``ws_ypixel``: height of terminal by pixels (not accurate).
        """

        if self._is_a_tty and self._init_descriptor is not None:
            try:
                return self._winsize(self._init_descriptor)
            except IOError:
                pass

        return WINSZ(
            ws_row=int(os.getenv("LINES", "25")),
            ws_col=int(os.getenv("COLUMNS", "80")),
            ws_xpixel=None,
            ws_ypixel=None,
        )

    @staticmethod
    def _winsize(fd):
        """
        Return named tuple describing size of the terminal by ``fd``.
        If the given platform does not have modules :mod:`termios`,
        :mod:`fcntl`, or :mod:`tty`, window size of 80 columns by 25
        rows is always returned.
        :arg int fd: file descriptor queries for its window size.
        :raises IOError: the file descriptor ``fd`` is not a terminal.
        :rtype: WINSZ
        WINSZ is a :class:`collections.namedtuple` instance, whose structure
        directly maps to the return value of the :const:`termios.TIOCGWINSZ`
        ioctl return value. The return parameters are:
        * ``ws_row``: width of terminal by its number of character cells.
        * ``ws_col``: height of terminal by its number of character cells.
        * ``ws_xpixel``: width of terminal by pixels (not accurate).
        * ``ws_ypixel``: height of terminal by pixels (not accurate).
        """
        data = fcntl.ioctl(fd, termios.TIOCGWINSZ, WINSZ._BUF)
        return WINSZ(*struct.unpack(WINSZ._FMT, data))


class WINSZ(
    collections.namedtuple("WINSZ", ("ws_row", "ws_col", "ws_xpixel", "ws_ypixel"))
):
    """
    Structure represents return value of :const:`termios.TIOCGWINSZ`.
    .. py:attribute:: ws_row rows, in characters
    .. py:attribute:: ws_col columns, in characters
    .. py:attribute:: ws_xpixel horizontal size, pixels
    .. py:attribute:: ws_ypixel vertical size, pixels
    """

    #: format of termios structure
    _FMT = "hhhh"
    #: buffer of termios structure appropriate for ioctl argument
    _BUF = "\x00" * struct.calcsize(_FMT)


class Region(object):
    def __init__(self, stream, top=0, height=None):
        self.top = top
        self.height = height
        self.stream = stream
        self.bottom = self.top + self.height
        self.cursorpos = stream.move(self.bottom, 0)

    def __enter__(self):
        self.stream.save().move(self.top, 0)
        return self.stream

    def __exit__(self, exc_type, value, traceback):
        self.stream.restore()

    def __getattr__(self, name):
        return getattr(self.stream, name, None)

    def clear(self):
        stream = self.stream
        stream.save()

        for i in range(self.top, self.bottom):
            stream.move(i, 0).clear_eol()

        stream.restore()
        return self


# We really only need a single global instance of TerminalIO. The class is
# (mostly) stateless and there should be exactly one instance per tty,
# of which there is usually only one.
TERM = TerminalIO(sys.stderr)
