# -*- coding: utf-8 -*-
"""
    scap
    ~~~~
    Wikimedia's MediaWiki deployment script. Deploys MediaWiki code and
    configuration to a group of servers via SSH and rsync.

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import absolute_import

import scap.plugins

from scap.main import (
    CompileWikiversions,
    LockManager,
    MWVersionsInUse,
    RebuildCdbs,
    RefreshCdbJsonFiles,
    Scap,
    SecurityPatchCheck,
    SyncCommon,
    SyncFile,
    SyncL10n,
    SyncMaster,
    SyncWikiversions,
    Version,
)

from scap.version import __version__

from scap.deploy import (
    Deploy,
    DeployLocal,
    DeployLog,
)

__all__ = [
    'CompileWikiversions',
    'Deploy',
    'DeployLocal',
    'DeployLog',
    'LockManager',
    'MWVersionsInUse',
    'RebuildCdbs',
    'RefreshCdbJsonFiles',
    'Scap',
    'SecurityPatchCheck',
    'SyncCommon',
    'SyncFile',
    'SyncL10n',
    'SyncMaster',
    'SyncWikiversions',
    'Version',
    '__version__',
]


def all_applications():
    """Load all the plugins and add them to the list of applications"""
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
    LockManager,
    MWVersionsInUse,
    RebuildCdbs,
    RefreshCdbJsonFiles,
    Scap,
    SecurityPatchCheck,
    SyncCommon,
    SyncFile,
    SyncMaster,
    SyncL10n,
    SyncWikiversions,
    Version,
    __version__))  # Ignore unused import warning
