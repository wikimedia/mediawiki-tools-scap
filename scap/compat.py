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
    def _unicode(obj):
        return obj.decode('utf-8')
elif six.PY3:
    def _unicode(obj):
        return obj
