# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
from .main import MWVersionsInUse
from .main import Scap
from .main import SyncCommon

from . import log

__all__ = (
    'MWVersionsInUse',
    'Scap',
    'SyncCommon',
)

any((MWVersionsInUse, Scap, SyncCommon))  # Ignore unused import warning

log.setup_loggers()
