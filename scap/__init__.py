# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
from .main import scap
from .main import sync_common
from .main import mwversionsinuse
from . import log

__all__ = (
    'scap',
    'sync_common',
    'mwversionsinuse',
)

any((scap, sync_common, mwversionsinuse))  # Ignore unused import warning

log.setup_loggers()
