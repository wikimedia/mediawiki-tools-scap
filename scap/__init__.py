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

from scap.version import __version__

# Import here classes decorated with `@cli.command`. They will be registered and made available as scap commands

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

from scap.backport import Backport
from scap.clean import Clean
from scap.deploy import (
    Deploy,
    DeployLocal,
    DeployLog,
)  # `scap.deploy.DeployMediaWiki` not imported. It looks like it never saw the light of day
from scap.deploy_promote import DeployPromote
from scap.install_world import InstallWorld
from scap.kubernetes_cli import BuildImages
from scap.mwscript import Mwscript, Mwshell
from scap.patch import SecurityPatchManager
from scap.patches import ApplyPatches, UpdatePatch, RemovePatch
from scap.php_fpm_restart import PhpRpmRestartCmd
from scap.prep import CheckoutMediaWiki
from scap.rollback import Rollback
from scap.say import Fortune, Say
from scap.spiderpig.jobrunner import JobRunner
from scap.stage_train import StageTrain
from scap.test import TestBackport, TestProgress, TestTrain
from scap.train import Train
from scap.updateinterwikicache import UpdateInterwikiCache
from scap.updatewikiversions import UpdateWikiversions

assert sys.version_info > (3,)
