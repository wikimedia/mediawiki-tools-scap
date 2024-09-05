"""
    scap.history
    ~~~~~~~~
    Interfaces for recording the git checkouts of a number of repo working
    trees that are synced during scap sync-world.
"""

import dataclasses
from datetime import datetime, timedelta, timezone
import os
from prettytable import PrettyTable, SINGLE_BORDER

from scap import browser
from scap.runcmd import gitcmd, FailedCommand

from typing import List, Optional
from sqlalchemy import Engine, create_engine, ForeignKey, select, delete
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship, Session
from sqlalchemy.event import listen
from sqlalchemy.pool import Pool

# Maximum age of a history entry before it is gc'd
MAX_HISTORY_LENGTH = timedelta(days=365)


class ModelBase(DeclarativeBase):
    pass


class Deployment(ModelBase):
    """
    Represents a scap sync-world deployment in history, including
    information about the relevant git repo checkouts that were
    synced, and metadata for username, start and end times, and
    whether the sync completed successfully.
    """

    __tablename__ = "deployment"

    id: Mapped[int] = mapped_column(primary_key=True)
    starttime: Mapped[datetime] = mapped_column(index=True)
    endtime: Mapped[Optional[datetime]]
    username: Mapped[str]
    completed: Mapped[bool] = mapped_column(default=False)
    errors: Mapped[bool] = mapped_column(default=False)

    checkouts: Mapped[List["Checkout"]] = relationship(
        back_populates="deployment", cascade="all, delete-orphan"
    )

    def summary(self, repos) -> str:
        """
        Returns a terse summary of the deployment, including branch head
        information for only the given repos.
        """
        return "%s: (%s): %s" % (
            self.starttime,
            self.username,
            self._branch_heads(repos),
        )

    def lookup(self, repo, branch, directory) -> str:
        """
        Returns the commit/ref from the deployment's checkouts for the given
        repo, branch and directory.
        """
        for checkout in self.checkouts:
            if (
                checkout.repo == repo
                and checkout.branch == branch
                and checkout.directory == directory
            ):
                return checkout.commit_ref

    def _branch_heads(self, repos):
        """
        Returns a short head summary from the checkout log for the given
        short repo names.
        """
        heads = {}
        repos = set(repos)
        for co in self._checkout_list():
            if co.short_repo in repos:
                heads[co.branch] = co.commit

        return heads

    def _checkout_list(self) -> List["DisplayCheckout"]:
        """
        Returns the checkouts as a list of DisplayCheckout objects.

        Used to present various human readable output. Note that common
        directory and repo paths are removed.
        """

        repos = []
        branches = []
        directories = []
        commits = []

        for checkout in sorted(self.checkouts, key=lambda checkout: checkout.repo):
            repos.append(checkout.repo)
            branches.append(checkout.branch)
            directories.append(checkout.directory)
            commits.append(checkout.commit_ref)

        short_repos = strip_common_dirname(repos)
        short_directories = strip_common_dirname(directories)

        return [
            DisplayCheckout(
                repos[i],
                branches[i],
                directories[i],
                commits[i],
                short_repos[i],
                short_directories[i],
            )
            for i in range(len(repos))
        ]

    def __prettytable__(self):
        checkouts = self._checkout_list()

        table = PrettyTable()
        table.set_style(SINGLE_BORDER)
        table.field_names = ["directory", "repo", "branch", "commit"]
        for co in checkouts:
            table.add_row([co.short_directory, co.short_repo, co.branch, co.commit])
        table.align = "l"
        table.align["branch"] = "r"
        table.align["commit"] = "r"

        return (table, checkouts)


class Checkout(ModelBase):
    """
    Information about a git repo that was included in a scap sync-world operation.
    """

    __tablename__ = "checkout"

    id: Mapped[int] = mapped_column(primary_key=True)
    deployment_id: Mapped[int] = mapped_column(ForeignKey("deployment.id"))
    repo: Mapped[str]
    branch: Mapped[str]
    directory: Mapped[str]
    commit_ref: Mapped[str]

    deployment: Mapped["Deployment"] = relationship(back_populates="checkouts")


@dataclasses.dataclass
class HistoryResults:
    entries: List[Deployment] = dataclasses.field(default_factory=list)
    display_repos: List[str] = dataclasses.field(default_factory=list)

    def __prettytable__(self):
        table = PrettyTable()
        table.set_style(SINGLE_BORDER)
        table.field_names = ["#", "start time", "deployer", "branches"]
        for entry in self.entries:
            branch_heads = entry._branch_heads(self.display_repos)
            branches = ", ".join(
                [
                    "%s (%s)" % (branch, head[:8])
                    for branch, head in branch_heads.items()
                ]
            )
            table.add_row([entry.id, entry.starttime, entry.username, branches])
        table.align = "l"
        return (table, self.entries)


class History:
    db_filename: str
    engine: Engine

    def __init__(self, db_filename):
        self.db_filename = db_filename
        self.engine = self._get_db_engine()

    def _get_db_engine(self):
        def set_sqlite_pragma(dbapi_connection, connection_record):
            cursor = dbapi_connection.cursor()
            cursor.execute("PRAGMA journal_mode=WAL;")
            cursor.close()

        engine = create_engine(f"sqlite:///{self.db_filename}")
        listen(Pool, "connect", set_sqlite_pragma)
        return engine

    def _setup_db(self):
        # Set up tables if needed
        ModelBase.metadata.create_all(self.engine)

        # Ensure that the database is group writable
        if os.geteuid() == os.stat(self.db_filename).st_uid:
            os.chmod(self.db_filename, 0o664)

    def log(self, deployment: Deployment):
        """
        Record a deployment in the history log.
        """

        self._setup_db()

        with Session(self.engine) as session:
            session.add(deployment)
            session.add_all(deployment.checkouts)
            session.commit()
        self._gc()

    def browse(self, display_repos=[]) -> Deployment:
        """
        Interactively browses the history and returns the user-selected deployment.
        """
        with Session(self.engine) as session:
            deployments = list(
                session.scalars(
                    select(Deployment)
                    .where(Deployment.completed)
                    .order_by(Deployment.starttime.desc())
                )
            )
            return browser.browse(HistoryResults(deployments, display_repos))

    def _gc(self):
        cutoff = datetime.now(timezone.utc) - MAX_HISTORY_LENGTH

        with Session(self.engine) as session:
            session.execute(delete(Deployment).where(Deployment.starttime < cutoff))
            session.commit()


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
            return path[len(common_dir) :].lstrip("/")

    return list(map(strip, paths))


class DisplayCheckout:
    """
    Used to display checkout details to the user.
    """

    def __init__(self, repo, branch, directory, commit, short_repo, short_directory):
        self.repo = repo
        self.branch = branch
        self.directory = directory
        self.commit = commit
        self.short_repo = short_repo
        self.short_directory = short_directory

    def __prettytable__(self):
        table = PrettyTable()
        table.set_style(SINGLE_BORDER)
        table.header = False

        commit_info = []
        try:
            commit_info = gitcmd("show", self.commit, cwd=self.directory).splitlines()
        except (FailedCommand, FileNotFoundError):
            commit_info = ["(checkout information is unavailable)"]

        for line in commit_info:
            table.add_row([line])

        table.align = "l"

        return (table, commit_info)

    def __repr__(self):
        return self.__dict__.__repr__()
