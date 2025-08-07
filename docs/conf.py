# -*- coding: utf-8 -*-

import sys
import os
import subprocess
from datetime import date

sys.path.insert(0, os.path.abspath(".."))

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.graphviz",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinxcontrib.programoutput",
]

templates_path = ["_templates"]
source_suffix = ".rst"
master_doc = "index"
project = "scap"
year = date.today().year
copyright = "2014-%s, Wikimedia Foundation & Contributors." % year
copyright += (
    " Released under the terms of the GNU General Public License,"
    + " version 3 <https://www.gnu.org/licenses/gpl-3.0.txt>."
)
version = "3.0"
release = version
exclude_patterns = ["_build"]
pygments_style = "sphinx"
man_pages = [
    (
        "scap2/commands",
        "scap",
        "Wikimedia deployment tool - scap: scatter crap around production",
        [],
        "1",
    )
]

html_theme = "nature"

intersphinx_mapping = {"python": ("https://docs.python.org/2.7", None)}

htmlhelp_basename = "scapdoc"
html_favicon = "favicon.ico"
autodoc_default_options = {
    "members": None,
    "private-members": None,
    "special-members": None,
}
autodoc_member_order = "groupwise"
