# -*- coding: utf-8 -*-

import sys
import os
import subprocess
from datetime import date

sys.path.insert(0, os.path.abspath(".."))
# import scap

extensions = ["sphinx.ext.autodoc", "sphinx.ext.viewcode"]
# extensions += ['sphinxarg.ext']
extensions += ["sphinxcontrib.programoutput"]
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

extensions += ["sphinx.ext.intersphinx"]
intersphinx_mapping = {"python": ("https://docs.python.org/2.7", None)}

# html_static_path = ['_static']
htmlhelp_basename = "scapdoc"
html_favicon = "favicon.ico"
autodoc_default_options = {
    "members": None,
    "private-members": None,
    "special-members": None,
}
autodoc_memeber_order = "groupwise"

# diagram support
extensions += ["sphinxcontrib.actdiag", "sphinxcontrib.blockdiag"]
actdiag_html_image_format = blockdiag_html_image_format = "svg"

try:
    # attempt to find a decent font using font-config
    actdiag_fontpath = blockdiag_fontpath = subprocess.check_output(
        ["fc-match", "-f", "%{file}", "arial"]
    )
except OSError:
    pass
