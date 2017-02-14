# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

"""
import scap.plugins

from .main import (
    CompileWikiversions,
    HHVMGracefulAll,
    MWVersionsInUse,
    RebuildCdbs,
    RefreshCdbJsonFiles,
    RestartHHVM,
    ServerAdminLog,
    Scap,
    SecurityPatchCheck,
    SyncCommon,
    SyncFile,
    SyncL10n,
    SyncMaster,
    SyncWikiversions,
    UpdateL10n,
)

from .version import __version__

from .deploy import (
    Deploy,
    DeployLocal,
    DeployLog,
)

__all__ = [
    'CompileWikiversions',
    'Deploy',
    'DeployLocal',
    'DeployLog',
    'HHVMGracefulAll',
    'MWVersionsInUse',
    'RebuildCdbs',
    'RefreshCdbJsonFiles',
    'RestartHHVM',
    'ServerAdminLog',
    'Scap',
    'SecurityPatchCheck',
    'SyncCommon',
    'SyncFile',
    'SyncL10n',
    'SyncMaster',
    'SyncWikiversions',
    'UpdateL10n',
    '__version__',
]


def all_applications():
    scap.plugins.load_plugins()
    apps = []
    apps.extend(__all__)
    apps.extend(scap.plugins.__all__)
    return apps


any((
    CompileWikiversions,
    Deploy,
    DeployLocal,
    DeployLog,
    HHVMGracefulAll,
    MWVersionsInUse,
    RebuildCdbs,
    RefreshCdbJsonFiles,
    ServerAdminLog,
    RestartHHVM,
    Scap,
    SecurityPatchCheck,
    SyncCommon,
    SyncFile,
    SyncMaster,
    SyncL10n,
    SyncWikiversions,
    UpdateL10n,
    __version__))  # Ignore unused import warning
