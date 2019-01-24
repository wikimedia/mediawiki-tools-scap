# -*- coding: utf-8 -*-
"""
    scap.compat
    ~~~~~~~~
    Helper functions for compatibility between python2 and python3

    Copyright © 2014-2018 Wikimedia Foundation and Contributors.

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

import six

if six.PY2:
    def _unicode(obj, encoding='utf-8', errors='strict'):
        return obj.decode('utf-8', encoding, errors)
elif six.PY3:
    def _unicode(obj, *_):
        return obj


def to_text(obj, encoding='utf-8', errors='strict'):
    if not isinstance(obj, (six.text_type, six.binary_type)):
        raise TypeError("compat.to_text not expecting type '%s'" % type(obj))
    if isinstance(obj, six.text_type):
        return _unicode(obj, encoding, errors)
    if isinstance(obj, six.binary_type):
        return obj.decode(encoding, errors)
    return obj


def to_bytes(obj, encoding='utf-8', errors='strict'):
    if isinstance(obj, six.text_type):
        return obj.encode(encoding, errors)
    elif isinstance(obj, six.binary_type):
        return obj
    else:
        raise TypeError("not expecting type '%s'" % type(obj))


def to_bytes_or_none(obj, encoding='utf-8', errors='strict'):
    if obj is None:
        return obj
    return to_bytes(obj, encoding, errors)
