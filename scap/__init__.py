# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
from .main import CompileWikiversions
from .main import MWVersionsInUse
from .main import PurgeL10nCache
from .main import Scap
from .main import SyncCommon
from .main import SyncWikiversions

__all__ = (
    'CompileWikiversions',
    'MWVersionsInUse',
    'PurgeL10nCache',
    'Scap',
    'SyncCommon',
    'SyncWikiversions',
)

any((
    CompileWikiversions,
    MWVersionsInUse,
    PurgeL10nCache,
    Scap,
    SyncCommon,
    SyncWikiversions))  # Ignore unused import warning
