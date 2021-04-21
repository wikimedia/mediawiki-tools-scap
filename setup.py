#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Base setup stuff for packaging of scap. Version numbers, authors, all that
    jazz

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
import os.path
from distutils.core import setup

AUTHORS = [
    ("Antoine Musso", "hashar@free.fr"),
    ("Bryan Davis", "bd808@wikimedia.org"),
    ("Chad Horohoe", "chadh@wikimedia.org"),
    ("Dan Duvall", "dduvall@wikimedia.org"),
    ("Giuseppe Lavagetto", "joe@wikimedia.org"),
    ("Mukunda Modell", "mmodell@wikimedia.org"),
    ("Ori Livneh", "ori@wikimedia.org"),
    ("Tyler Cipriani", "tcipriani@wikimedia.org"),
]


# Read version from file shared with the module using technique from
# https://python-packaging-user-guide.readthedocs.io/en/latest/single_source_version/
VERSION = {}
FILENAME = os.path.join(os.path.dirname(__file__), "scap", "version.py")
# pylint complains about exec, but that's exactly what we want here
# pylint: disable=W0122
exec(compile(open(FILENAME, "rb").read(), FILENAME, "exec"), VERSION)

setup(
    name="Scap",
    version=VERSION["__version__"],
    description="Deployment toolchain for Wikimedia projects",
    long_description=open("README.rst", "rb").read().decode("UTF8"),
    author=", ".join([name for name, _ in AUTHORS]),
    author_email=", ".join([email for _, email in AUTHORS]),
    license="GNU GPLv3",
    maintainer="Wikimedia Foundation Release Engineering",
    maintainer_email="releng@wikimedia.org",
    url="https://gerrit.wikimedia.org/g/mediawiki/tools/scap",
    packages=["scap", "scap.plugins"],
    package_dir={"scap": "scap"},
    scripts=["bin/scap"],
    keywords=["deploy", "deployment", "scap", "scap2", "scap3"],
    classifiers=[
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python",
        "Programming Language :: Python :: 2",
        "Programming Language :: Python :: 2.7",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    ],
)
