"""
    scap.history
    ~~~~~~~~
    Interfaces for recording the git checkouts of a number of repo working
    trees that occur during prep.
"""

from datetime import datetime
import getpass
import json
import os
from prettytable import PrettyTable

from scap import browser
from scap import utils
from scap.runcmd import gitcmd, FailedCommand


TIMESTAMP_FORMAT = '%Y-%m-%d %H:%M:%S'

# To avoid having to line-wise truncate the history log file on every run,
# file size is used as a heuristic measure by which to detect the need to
# GC. A typical log entry in production will be a little under 400 bytes
# long, so this heuristic uses a file size well above the number of entries
# we'd definitely like to keep around (say, double that number) multiplied
# by the typical log entry size. See log() and log_gc()
LOG_GC_TRUNCATE_TO_LINES = 40
LOG_GC_BYTE_LIMIT = 400 * (LOG_GC_TRUNCATE_TO_LINES * 2)


def load(path, **kwargs):
    """Loads history from a given log file."""
    try:
        with utils.open_exclusively(path, 'r') as f:
            return History.load(f, **kwargs)
    except FileNotFoundError:
        return History(**kwargs)
    except ValueError as e:
        raise ValueError(
            "failed to parse history file %s" % path
        ) from e


def log(entry, path):
    """
    Appends the log with a new history entry and performs garbage collection
    when the log surpasses LOG_GC_BYTE_LIMIT in size.
    """
    dirname = os.path.dirname(path)
    if dirname:
        os.makedirs(dirname, exist_ok=True)

    with utils.open_exclusively(path, 'a+') as f:
        f.write(entry.dumps() + "\n")

        # perform garbage collection once the log size limit is reached
        if f.tell() >= LOG_GC_BYTE_LIMIT:
            lines = f.readlines()[-LOG_GC_TRUNCATE_TO_LINES:]
            f.seek(0)
            f.writelines(lines)
            f.truncate(f.tell())


def strip_common_dirname(paths):
    """
    Removes from the given paths a parent directory that all paths have in
    common.

    >>> strip_common(['/srv/foo', '/srv/foo/bar', '/srv/bar'])
    ['foo', 'foo/bar', 'bar']
    """

    if not paths:
        return paths

    def match_or_find_upward(common, dirname):
        if dirname == common or dirname.startswith(common + "/"):
            return common

        i = common.rfind("/")
        if i >= 0:
            return match_or_find_upward(common[0:i], dirname)

        return ""

    common_dir = os.path.dirname(paths[0])

    for path in paths:
        if common_dir == "":
            break

        common_dir = match_or_find_upward(common_dir, os.path.dirname(path))

    if common_dir == "":
        return paths

    def strip(path):
        if path == common_dir:
            return path
        else:
            return path[len(common_dir):].lstrip('/')

    return list(map(strip, paths))


class History:
    """
    Represents scap prep history.
    """

    entries = None

    @classmethod
    def load(cls, lines, **kwargs):
        """
        Loads history from the given iterable, typically the lines of an open
        file object.
        """

        entries = []

        for i, line in enumerate(lines):
            try:
                entries.append(Entry.loads(line))
            except ValueError as e:
                raise ValueError(
                    "invalid history on line %d: %s" % ((i + 1), line)
                ) from e

        return cls(entries, **kwargs)

    def __init__(self, entries=[], display_repos=[]):
        self.entries = entries
        self.display_repos = display_repos

    def browse(self, completed_only=True):
        """
        Interactively browses the history and returns the user-selected entry.
        """

        return browser.browse(self.completed() if completed_only else self)

    def completed(self):
        """
        Returns a history object with only completed entries.
        """

        return self.__class__(
            filter(lambda e: e.completed, self.entries),
            display_repos=self.display_repos
        )

    def __prettytable__(self):
        table = PrettyTable()
        table.field_names = ["#", "timestamp", "deployer", "branches"]
        entries = sorted(self.entries, key=lambda e: e.timestamp, reverse=True)
        for i, entry in enumerate(entries):
            branch_heads = entry.branch_heads(self.display_repos)
            branches = ', '.join([
                "%s (%s)" % (branch, head[:8])
                for branch, head in branch_heads.items()
            ])
            table.add_row([i, entry.timestamp, entry.username, branches])
        table.align = "l"
        return (table, entries)


class Entry:
    """
    Represents a single entry in the history, a set of checkouts performed
    during the scap prep session, and metadata for username, timestamp, and
    whether the prep session completed successfully.
    """

    completed = False
    timestamp = None
    checkouts = None
    username = None

    def __init__(self, **kwargs):
        self.checkouts = {}
        for attr in kwargs:
            setattr(self, attr, kwargs[attr])

    @classmethod
    def loads(cls, serialized):
        """Loads a history entry from a JSON string."""
        attrs = json.loads(serialized)

        ts = attrs.get('timestamp')
        if ts is not None:
            attrs['timestamp'] = datetime.strptime(ts, TIMESTAMP_FORMAT)

        return cls(**attrs)

    @classmethod
    def now(cls, **attrs):
        """
        Creates a new entry with the current timestamp and meta data from the
        environment.
        """

        return cls(
            username=getpass.getuser(),
            timestamp=datetime.utcnow(),
            completed=False,
            **attrs
        )

    def branch_heads(self, repos):
        """
        Returns a short head summary from the checkout log for the given
        short repo names.
        """
        heads = {}
        repos = set(repos)
        for co in self.checkout_list():
            if co.short_repo in repos:
                heads[co.branch] = co.commit

        return heads

    def checkout_list(self):
        """
        Returns the checkouts as a list of Checkout objects.

        Used to present various human readable output. Note that common
        directory and repo paths are removed.
        """

        repos = []
        branches = []
        directories = []
        commits = []

        for repo in self.checkouts:
            for branch in self.checkouts[repo]:
                for directory in self.checkouts[repo][branch]:
                    repos.append(repo)
                    branches.append(branch)
                    directories.append(directory)
                    commits.append(self.checkouts[repo][branch][directory])

        short_repos = strip_common_dirname(repos)
        short_directories = strip_common_dirname(directories)

        return [
            Checkout(
                repos[i], branches[i], directories[i], commits[i],
                short_repos[i], short_directories[i]
            )
            for i in range(len(repos))
        ]

    def dumps(self, **kwargs):
        """
        Returns a JSON representation of the history entry, formatted for
        storage as a single file line.
        """

        attrs = self.__dict__

        ts = attrs.get('timestamp')
        if ts is not None:
            attrs['timestamp'] = ts.strftime(TIMESTAMP_FORMAT)

        return json.dumps(
            attrs,
            **{"sort_keys": True, "separators": (',', ':'), **kwargs},
        )

    def lookup(self, repo, branch, directory):
        """
        Returns the commit/ref from the entry's checkouts for the given
        repo, branch and directory.
        """
        return self.checkouts.get(repo, {}).get(branch, {}).get(directory)

    def summary(self, repos):
        """
        Returns a terse summary of the entry, including branch head
        information for only the given repos.
        """
        return "%s: (%s): %s" % (self.timestamp, self.username,
                                 self.branch_heads(repos))

    def update(self, repo, branch, directory, commit):
        """
        Updates checkout record (repo/branch/commit) information for this
        entry.
        """
        if repo not in self.checkouts:
            self.checkouts[repo] = {}

        if branch not in self.checkouts[repo]:
            self.checkouts[repo][branch] = {}

        if directory not in self.checkouts[repo][branch]:
            self.checkouts[repo][branch][directory] = {}

        self.checkouts[repo][branch][directory] = commit

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def __prettytable__(self):
        checkouts = self.checkout_list()

        table = PrettyTable()
        table.field_names = ["directory", "repo", "branch", "commit"]
        for co in checkouts:
            table.add_row([
                co.short_directory, co.short_repo, co.branch, co.commit
            ])
        table.align = "l"
        table.align["branch"] = "r"
        table.align["commit"] = "r"

        return (table, checkouts)

    def __repr__(self):
        return self.__dict__.__repr__()


class Checkout:
    """
    Used to display checkout details to the user.
    """

    def __init__(self, repo, branch, directory, commit, short_repo,
                 short_directory):
        self.repo = repo
        self.branch = branch
        self.directory = directory
        self.commit = commit
        self.short_repo = short_repo
        self.short_directory = short_directory

    def __prettytable__(self):
        table = PrettyTable()
        table.header = False

        commit_info = []
        try:
            commit_info = gitcmd("show", self.commit,
                                 cwd=self.directory).splitlines()
        except (FailedCommand, FileNotFoundError):
            commit_info = ["(checkout information is unavailable)"]

        for line in commit_info:
            table.add_row([line])

        table.align = "l"

        return (table, commit_info)

    def __repr__(self):
        return self.__dict__.__repr__()
