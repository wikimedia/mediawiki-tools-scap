# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
from .main import scap          # NOQA  -- ignore unused import warning
from .main import sync_common   # NOQA  -- ignore unused import warning
from . import log

__all__ = (
    'scap',
    'sync_common',
)

log.setup_loggers()
