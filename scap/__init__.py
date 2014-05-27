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
from .main import RebuildCdbs
from .main import Scap
from .main import SyncCommon
from .main import SyncDblist
from .main import SyncDir
from .main import SyncDocroot
from .main import SyncFile
from .main import SyncWikiversions
from .main import UpdateL10n

__all__ = (
    'CompileWikiversions',
    'MWVersionsInUse',
    'PurgeL10nCache',
    'RebuildCdbs',
    'Scap',
    'SyncCommon',
    'SyncDblist',
    'SyncDir',
    'SyncDocroot',
    'SyncFile',
    'SyncWikiversions',
    'UpdateL10n',
)

any((
    CompileWikiversions,
    MWVersionsInUse,
    PurgeL10nCache,
    RebuildCdbs,
    Scap,
    SyncCommon,
    SyncDblist,
    SyncDir,
    SyncDocroot,
    SyncFile,
    SyncWikiversions,
    UpdateL10n))  # Ignore unused import warning
