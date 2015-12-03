# -*- coding: utf-8 -*-
"""
    scap.context
    ~~~~~~~~~~~
    Management of deployment host/target directories and execution context.

"""
from datetime import datetime
import os
import shutil

from . import utils

REV_DELIMITER = '_'
REVS_TO_KEEP = 5


class Context(object):
    """Base context for either the deployment host or target."""

    def __init__(self, root, environment=None, user=utils.get_real_username()):
        """Instantiates a new context at the given root path.

        :param root: Root directory
        :param environment: Environment name used when resolving config files.
        :param user: User the context will execute commands as.
        """

        self.root = root
        self.environment = environment
        self.user = user

    def cleanup(self):
        """Deletes all files in the temporary directory."""

        _, dirs, files = os.walk(self.temp_path()).next()

        for d in dirs:
            shutil.rmtree(self.temp_path(d))

        for f in files:
            os.unlink(self.temp_path(f))

    def path(self, *relpaths):
        """Qualifies path relative to the root path."""

        return os.path.join(self.root, *relpaths)

    def setup(self):
        """Creates the root and temporary directories as needed."""

        for d in [self.root, self.temp_path()]:
            utils.mkdir_p(d, user=self.user)

    def temp_path(self, *relpaths):
        """Qualifies a path to be used for temporary files/directories."""

        return self.path('tmp', *relpaths)


class HostContext(Context):
    """Manages deployment host paths and execution context."""

    def env_specific_path(self, *relpaths):
        """Returns path to default or environment specific file/directory.

        Both the environment specific path at ``scap/environments/{name}`` and
        the default path at ``scap`` is searched in respective order. The
        first path that exists will be returned.
        """

        paths = self.env_specific_paths(*relpaths)

        if paths:
            return paths[-1]
        else:
            return None

    def env_specific_paths(self, *relpaths):
        """Returns paths to default and environment specific files/directories.

        Both the environment specific path at ``scap/environments/{name}`` and
        the default path at ``scap`` is searched. Paths are included in the
        returned list for each one that exists.
        """

        paths = [self.scap_path(*relpaths)]

        if self.environment:
            paths.append(self.scap_path('environments', self.environment,
                                        *relpaths))

        return filter(os.path.exists, paths)

    def log_path(self, *relpaths):
        """Qualifies the given log path."""

        return self.scap_path('log', *relpaths)

    def setup(self):
        """Creates the scap and log directories as necessary.

        See :class:``Context.setup`` for its additional operations.
        """

        super(HostContext, self).setup()

        for d in [self.scap_path(), self.log_path()]:
            if not os.path.exists(d):
                os.mkdir(d)

    def scap_path(self, *relpaths):
        """Qualifies path relative to the ``scap`` directory."""

        return self.path('scap', *relpaths)

    def temp_config_path(self, *relpaths):
        return self.path('.git', 'config-files', *relpaths)


class TargetContext(Context):
    """Manages target host paths and execution context."""

    @property
    def cache_dir(self):
        """Path to the cached repo clone."""

        return self.path('cache')

    @property
    def current_config_rev(self):
        """Currently deployed config revision."""

        if self.current_rev_dir is None:
            return None

        rev_dir = os.path.basename(self.current_rev_dir)

        if rev_dir and REV_DELIMITER in rev_dir:
            return rev_dir.split(REV_DELIMITER)[0]
        else:
            return None

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

    def find_old_rev_dirs(self):
        """Generates revision directories that are candidates for deletion.

        The :py:const:`REVS_TO_KEEP` most recent revision directories and any
        revision directory that is current or in progress is not considered.
        """

        rev_dirs = map(lambda d: os.path.join(self.revs_dir, d),
                       os.walk(self.revs_dir).next()[1])
        rev_dirs_by_ctime = sorted(rev_dirs,
                                   key=os.path.getctime,
                                   reverse=True)

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

        utils.move_symlink(self.rev_path(rev), path, user=self.user)

    def mark_rev_current(self, rev):
        """Change the current rev to the given one.

        This state is maintained as a ``current`` symlink in the directory
        root that points to the relevant ``revs/{rev}`` directory.
        """

        self.link_path_to_rev(self.current_link, rev)

    def mark_rev_done(self, rev):
        """Change the state to done for the given rev.

        This state is maintained as a ``.done`` symlink in the directory root
        that points to the relevant ``revs/{rev}`` directory.
        """

        self.link_path_to_rev(self._done_link, rev)

        if os.path.exists(self._progress_link):
            utils.remove_symlink(self._progress_link, user=self.user)

    def mark_rev_in_progress(self, rev):
        """Change the state to in-progress for the given rev.

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
        """Returns the path to the given repo revision."""

        config_rev = self._saved_config_rev

        if config_rev:
            rpath = REV_DELIMITER.join([config_rev, rev])
        else:
            rpath = rev

        return os.path.join(self.revs_dir, rpath, *paths)

    @property
    def revs_dir(self):
        """Context directory that stores revisions."""

        return self.path('revs')

    def setup(self):
        """Creates the cache and revs directory.

        See :class:``Context.setup`` for its additional operations.
        """

        super(TargetContext, self).setup()

        for d in [self.cache_dir, self.revs_dir]:
            utils.mkdir_p(d, user=self.user)

    def use_config_rev(self, config_rev):
        """Use the given config revision in namespacing the rev directory."""

        with open(self._config_rev_temp_path, 'w') as f:
            f.write(config_rev)

    @property
    def _config_rev_temp_path(self):
        return self.temp_path('.config-digest')

    @property
    def _done_link(self):
        return self.path('.done')

    @property
    def _progress_link(self):
        return self.path('.in-progress')

    def _rev_from_path(self, path):
        realpath = os.path.realpath(path)

        if os.path.exists(realpath):
            return os.path.basename(realpath).split(REV_DELIMITER)[-1]
        else:
            return None

    @property
    def _saved_config_rev(self):
        if os.path.exists(self._config_rev_temp_path):
            with open(self._config_rev_temp_path) as f:
                return f.read()
        else:
            return None
