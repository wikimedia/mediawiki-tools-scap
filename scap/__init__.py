# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
from .main import (
    CompileWikiversions,
    HHVMGracefulAll,
    MWVersionsInUse,
    PurgeL10nCache,
    RebuildCdbs,
    RestartHHVM,
    Scap,
    SyncCommon,
    SyncDblist,
    SyncDir,
    SyncDocroot,
    SyncFile,
    SyncMaster,
    SyncWikiversions,
    UpdateL10n,
)

from .deploy import (
    Deploy,
    DeployLocal,
    DeployLog,
)

__all__ = (
    'CompileWikiversions',
    'Deploy',
    'DeployLocal',
    'DeployLog',
    'HHVMGracefulAll',
    'MWVersionsInUse',
    'PurgeL10nCache',
    'RebuildCdbs',
    'RestartHHVM',
    'Scap',
    'SyncCommon',
    'SyncDblist',
    'SyncDir',
    'SyncDocroot',
    'SyncFile',
    'SyncMaster',
    'SyncWikiversions',
    'UpdateL10n',
)

any((
    CompileWikiversions,
    Deploy,
    DeployLocal,
    DeployLog,
    HHVMGracefulAll,
    MWVersionsInUse,
    PurgeL10nCache,
    RebuildCdbs,
    RestartHHVM,
    Scap,
    SyncCommon,
    SyncDblist,
    SyncDir,
    SyncDocroot,
    SyncFile,
    SyncMaster,
    SyncWikiversions,
    UpdateL10n))  # Ignore unused import warning
