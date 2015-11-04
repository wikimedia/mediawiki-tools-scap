# -*- coding: utf-8 -*-

import sys
import os
from datetime import date

sys.path.insert(0, os.path.abspath('..'))
# import scap

extensions = ['sphinx.ext.autodoc', 'sphinx.ext.viewcode']
# extensions += ['sphinxarg.ext']
extensions += ['sphinxcontrib.programoutput']
templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'index'
project = u'scap'
copyright = u'%s, Wikimedia Foundation & contributors' % date.today().year
version = '3.0'
release = version
exclude_patterns = ['_build']
pygments_style = 'sphinx'

html_theme = 'nature'
# html_static_path = ['_static']
htmlhelp_basename = 'scapdoc'
html_favicon = 'favicon.ico'
autodoc_default_flags = ['members', 'private-members', 'special-members']
autodoc_memeber_order = 'groupwise'
