# -*- coding: utf-8 -*-
"""
    scap.ansi
    ~~~~~~~~~
    ANSI escape codes

    ..seealso:: `https://en.wikipedia.org/wiki/ANSI_escape_code`

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

FG_BLACK = 30
FG_RED = 31
FG_GREEN = 32
FG_YELLOW = 33
FG_BLUE = 34
FG_MAGENTA = 35
FG_CYAN = 36
FG_WHITE = 37
FG_RESET = 39

BG_BLACK = 40
BG_RED = 41
BG_GREEN = 42
BG_YELLOW = 43
BG_BLUE = 44
BG_MAGENTA = 45
BG_CYAN = 46
BG_WHITE = 47
BG_RESET = 49

BRIGHT = 1
DIM = 2
UNDERLINE = 4
BLINK = 5
INVERSE = 7
RESET = 22
NOBLINK = 25
RESET_ALL = 0


def esc(*args):
    """
    Get an ANSI escape code.

    >>> esc(BG_WHITE, FG_RED, BLINK) == r'\x1b[5;31;47m'
    True

    :param args: ANSI attributes
    :returns: str
    """
    return "\x1b[%sm" % ";".join(str(arg) for arg in sorted(args))


def format_ansi(*args):
    """
    create an ansi color string from a list of color codes and plain strings.

    >>> format_ansi((FG_BLUE,BG_WHITE),'blue on white') \
                 == '\x1b[34;47mblue on white\x1b[0m'
    True

    :param args: ANSI color codes and strings of text
    :returns: str
    """
    result = ""
    for arg in args:
        argtype = type(arg)
        if argtype is int:
            result += esc(arg)
        elif argtype is tuple:
            result += esc(*arg)
        else:
            result += arg
    return result + esc(RESET_ALL)


def reset():
    """
    Get the ANSI reset code.

    >>> reset() == r'\x1b[0m'
    True

    :returns: str
    """
    return esc(RESET_ALL)


def logo(eyes=None, color=True, **colors):
    r"""
    Get the scap logo.

    Scappy the scap pig::

               ___ ____
             ⎛   ⎛ ,----
              \  //==--'
         _//| .·//==--'    ____________________________
        _OO≣=-  ︶ ᴹw ⎞_§ ______  ___\ ___\ ,\__ \/ __ \
       (∞)_, )  (     |  ______/__  \/ /__ / /_/ / /_/ /
         ¨--¨|| |- (  / _______\____/\___/ \__^_/  .__/
             ««_/  «_/ jgs/bd808               /_/

    Ascii art derived from original work by Joan Stark [#]_ and the `speed`
    figlet font [#]_.

    :param color: Color logo using ANSI escapes
    :param colors: Alternate colors
    :returns: str

    .. [#] http://www.oocities.org/spunk1111/farm.htm#pig
    .. [#] http://www.jave.de/figlet/fonts/details/speed.html
    """
    pallet = {
        "pig": reset() + esc(FG_MAGENTA, BRIGHT),
        "nose": reset() + esc(FG_MAGENTA, BRIGHT),
        "mouth": reset() + esc(FG_MAGENTA, BRIGHT),
        "goggles": reset() + esc(FG_YELLOW),
        "brand": reset(),
        "hoof": reset() + esc(FG_BLUE),
        "wing": reset() + esc(FG_CYAN),
        "speed": reset() + esc(FG_WHITE),
        "text": reset() + esc(FG_GREEN),
        "signature": reset() + esc(FG_BLUE),
        "reset": reset(),
    }
    pallet.update(colors)

    if not color:
        for key, _ in pallet.items():
            pallet[key] = ""

    if not eyes:
        eyes = "OO"

    eyes = eyes[:2]
    pallet["eyes"] = eyes.encode("utf-8")
    pallet["newline"] = "\n"

    return "".join(
        line % pallet
        for line in [
            r"""           %(wing)s___%(reset)s %(wing)s____%(reset)s""",
            r"""%(newline)s         %(wing)s⎛   ⎛ ,----%(reset)s%(newline)s""",
            r"""          %(wing)s\  //==--'%(reset)s%(newline)s""",
            r"""     %(pig)s_//|,.·%(wing)s//==--'%(reset)s    """,
            r"""%(speed)s______%(text)s____""",
            r"""%(speed)s_%(text)s____""",
            r"""%(speed)s___%(text)s____""",
            r"""%(speed)s__%(text)s____%(reset)s%(newline)s""",
            r"""    %(pig)s_%(goggles)s%(eyes)s≣=-%(pig)s """,
            r""" %(wing)s︶%(pig)s %(brand)sᴹw%(pig)s ⎞_§%(reset)s """,
            r"""%(speed)s______%(text)s  ___\ ___\ ,\__ \/ __ \%(reset)s""",
            r"""%(newline)s   %(pig)s(%(nose)s∞%(pig)s)%(mouth)s_,""",
            r"""%(pig)s )  (     |%(reset)s""",
            r"""  %(speed)s______%(text)s/__  \/ /__ / /_/ / /_/ /%(reset)s""",
            r"""%(newline)s     %(pig)s¨--¨|| |- (  /%(reset)s""",
            r""" %(speed)s______%(text)s\____/ \___/ \__^_/  .__/%(reset)s""",
            r"""%(newline)s         %(hoof)s««%(pig)s_/%(reset)s""",
            r"""  %(hoof)s«%(pig)s_/%(reset)s""",
            r""" %(signature)sjgs/bd808%(reset)s""",
            r"""                %(text)s/_/%(reset)s%(newline)s""",
        ]
    )
