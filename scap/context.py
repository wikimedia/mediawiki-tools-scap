# -*- coding: utf-8 -*-
"""
    scap.context
    ~~~~~~~~~~~~
    Management of deployment host/target directories and execution context.

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

from datetime import datetime
import glob
import os

import scap.utils as utils

REVS_TO_KEEP = 5


class Context(object):
    """Base context for either the deployment host or target."""

    def __init__(self, root, environment=None):
        """
        Instantiate a new context at the given root path.

        :param root: Root directory
        :param environment: Environment name used when resolving config files.
        """

        self.root = root
        self.environment = environment

    def path(self, *relpaths):
        """Qualify path relative to the root path."""

        return os.path.join(self.root, *relpaths)

    def setup(self):
        """Create the root directory, use it as the root context."""
        utils.mkdir_p(self.root)

        # Needed for deploy-local.
        #
        # If deploy-local is run like:
        #
        #     sudo -u [deploy-user] -- deploy-local --repo [repo]
        #
        # it will fail unless the deploy-user is able to cd into the current
        # directory. This is because the utils.cd context manager tries to
        # move back to the directory in which it started after the
        # cd context is closed.
        os.chdir(self.root)


class HostContext(Context):
    """Manage deployment host paths and execution context."""

    def env_specific_path(self, *relpaths):
        """
        Return path to default or environment specific file/directory.

        Both the environment specific path at ``scap/environments/{name}`` and
        the default path at ``scap`` is searched in respective order. The
        first path that exists will be returned.
        """

        paths = self.env_specific_paths(*relpaths)

        if paths:
            return paths[0]
        else:
            return None

    def env_specific_paths(self, *relpaths):
        """
        Return paths to default and environment specific files/directories.

        Both the environment specific path at ``scap/environments/{name}`` and
        the default path at ``scap`` is searched. Paths are included in the
        returned list for each one that exists.
        """

        paths = []

        if self.environment:
            paths.append(
                self.scap_path('environments', self.environment, *relpaths))

        paths.append(self.scap_path(*relpaths))

        return [real_path for p in paths for real_path in glob.glob(p)]

    def log_path(self, *relpaths):
        """Qualify the given log path."""

        return self.scap_path('log', *relpaths)

    def setup(self):
        """
        Create the scap and log directories as necessary.

        See :class:``Context.setup`` for its additional operations.
        """

        super(HostContext, self).setup()

        for d in [self.scap_path(), self.log_path(), self.temp_config_path()]:
            if not os.path.exists(d):
                os.mkdir(d)

    def scap_path(self, *relpaths):
        """Qualify path relative to the ``scap`` directory."""

        return self.path('scap', *relpaths)

    def temp_config_path(self, *relpaths):
        return self.path('.git', 'config-files', *relpaths)


class TargetContext(Context):
    """Manage target host paths and execution context."""

    @property
    def cache_dir(self):
        """Path to the cached repo clone."""

        return self.path('cache')

    @property
    def current_link(self):
        """Symlink that points to the currently deployed revision."""

        return self.path('current')

    @property
    def current_rev_dir(self):
        """Real path to the currently deployed revision."""

        if os.path.exists(self.current_link):
            return os.path.realpath(self.current_link)
        else:
            return None

    @property
    def done_rev_dir(self):
        """Real path to the revision previously marked as done."""

        if os.path.exists(self._done_link):
            return os.path.realpath(self._done_link)
        else:
            return None

    @property
    def local_config(self):
        """Local target file that has a copy of the last-deployed config."""
        return self.path('.config')

    def find_old_rev_dirs(self):
        """
        Generate revision directories that are candidates for deletion.

        The :py:const:`REVS_TO_KEEP` most recent revision directories and any
        revision directory that is current or in progress is not considered.
        """

        rev_dirs = map(
            lambda d: os.path.join(self.revs_dir, d),
            os.walk(self.revs_dir).next()[1])
        rev_dirs_by_ctime = sorted(
            rev_dirs, key=os.path.getctime, reverse=True)

        off_limits = [self.current_rev_dir, self.done_rev_dir]

        for rev_dir in rev_dirs_by_ctime[REVS_TO_KEEP::]:
            full_rev_dir = os.path.join(self.revs_dir, rev_dir)

            if full_rev_dir not in off_limits:
                yield full_rev_dir

    def link_path_to_rev(self, path, rev, backup=False):
        """Create link to the given rev's directory at the given path."""

        # Backup any file/directory that exists at the given path
        if backup and os.path.exists(path) and not os.path.islink(path):
            date = datetime.utcnow().isoformat()
            os.rename(path, '{}.{}'.format(path, date))

        utils.move_symlink(self.rev_path(rev), path)

    def mark_rev_current(self, rev):
        """
        Change the current rev to the given one.

        This state is maintained as a ``current`` symlink in the directory
        root that points to the relevant ``revs/{rev}`` directory.
        """

        self.link_path_to_rev(self.current_link, rev)

    def mark_rev_done(self, rev):
        """
        Change the state to done for the given rev.

        This state is maintained as a ``.done`` symlink in the directory root
        that points to the relevant ``revs/{rev}`` directory.
        """

        self.link_path_to_rev(self._done_link, rev)

    def rm_in_progress(self):
        if os.path.exists(self._progress_link):
            os.unlink(self._progress_link)

    def mark_rev_in_progress(self, rev):
        """
        Change the state to in-progress for the given rev.

        This state is maintained as a ``.in-progress`` symlink in the
        directory root that points to the relevant ``revs/{rev}`` directory.
        """

        self.link_path_to_rev(self._progress_link, rev)

    @property
    def rev_done(self):
        """The rev that is currently marked as done."""

        return self._rev_from_path(self._done_link)

    @property
    def rev_in_progress(self):
        """The rev that is currently marked as in-progress."""

        return self._rev_from_path(self._progress_link)

    def rev_path(self, rev, *paths):
        """Return the path to the given repo revision."""

        return os.path.join(self.revs_dir, rev, *paths)

    @property
    def revs_dir(self):
        """Context directory that stores revisions."""

        return self.path('revs')

    def setup(self):
        """
        Create the cache and revs directory.

        See :class:``Context.setup`` for its additional operations.
        """

        super(TargetContext, self).setup()

        for d in [self.cache_dir, self.revs_dir]:
            utils.mkdir_p(d)

    @property
    def _done_link(self):
        return self.path('.done')

    @property
    def _progress_link(self):
        return self.path('.in-progress')

    def _rev_from_path(self, path):
        realpath = os.path.realpath(path)

        if os.path.exists(realpath):
            return os.path.basename(realpath)
        else:
            return None
