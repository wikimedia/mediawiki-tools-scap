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

import sys

import scap.plugins

from scap.main import (
    CompileWikiversions,
    LockManager,
    MWVersionsInUse,
    RebuildCdbs,
    RefreshCdbJsonFiles,
    SecurityPatchCheck,
    SyncPull,
    SyncFile,
    SyncMaster,
    SyncWikiversions,
    Version,
)

from scap.version import __version__

from scap.deploy import Deploy, DeployLocal, DeployLog

import scap.runcmd

from scap.deploy_promote import DeployPromote
from scap.stage_train import StageTrain
from scap.train import Train
from scap.install_world import InstallWorld

__all__ = [
    "CompileWikiversions",
    "Deploy",
    "DeployLocal",
    "DeployLog",
    "DeployPromote",
    "InstallWorld",
    "LockManager",
    "MWVersionsInUse",
    "RebuildCdbs",
    "RefreshCdbJsonFiles",
    "SecurityPatchCheck",
    "StageTrain",
    "SyncPull",
    "SyncFile",
    "SyncMaster",
    "SyncWikiversions",
    "Train",
    "Version",
    "__version__",
    "runcmd",
]


def all_applications():
    """Load all the plugins and add them to the list of applications"""
    scap.plugins.load_plugins()
    apps = []
    apps.extend(__all__)
    apps.extend(scap.plugins.__all__)
    return apps


any(
    (
        CompileWikiversions,
        Deploy,
        DeployLocal,
        DeployLog,
        LockManager,
        MWVersionsInUse,
        RebuildCdbs,
        RefreshCdbJsonFiles,
        SecurityPatchCheck,
        SyncPull,
        SyncFile,
        SyncMaster,
        SyncWikiversions,
        Version,
        __version__,
    )
)  # Ignore unused import warning

assert sys.version_info > (3,)
