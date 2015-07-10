# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
from .main import (
    CompileWikiversions,
    DeployLocal,
    Deploy,
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
    SyncWikiversions,
    UpdateL10n,
)

__all__ = (
    'CompileWikiversions',
    'DeployLocal',
    'Deploy',
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
    'SyncWikiversions',
    'UpdateL10n',
)

any((
    CompileWikiversions,
    DeployLocal,
    Deploy,
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
    SyncWikiversions,
    UpdateL10n))  # Ignore unused import warning
