import enum
import json
import logging
import os
import time

from typing import Dict, List, Optional
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
)
from pydantic.alias_generators import to_camel
from sqlalchemy import ForeignKey, select, delete, text, null, Enum
from sqlalchemy.ext.hybrid import hybrid_property
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, Session
from alembic.autogenerate import produce_migrations
from alembic.migration import MigrationContext
from alembic.operations import Operations
from alembic.operations.ops import ModifyTableOps

from scap import train


class BasePydantic(BaseModel):
    """
    BasePydantic should be used for models that are not database backed
    (non-ORM).
    """

    model_config = ConfigDict(
        alias_generator=to_camel,
        populate_by_name=True,
        from_attributes=True,
    )


class Base(DeclarativeBase):
    """
    Base should be used for database backed models (ORMs).
    """

    pass


class User(Base):
    __tablename__ = "user"

    name: Mapped[str] = mapped_column(primary_key=True)
    otp_seed: Mapped[str]
    last_2fa_time: Mapped[Optional[int]]
    last_2fa_code: Mapped[Optional[str]]

    @classmethod
    def get(self, session: Session, name: str) -> Optional["User"]:
        return session.scalar(select(User).where(User.name == name))

    @classmethod
    def add(self, session: Session, name: str, otp_seed: str) -> "User":
        """
        The caller is assumed to have already verified that a user
        with the given name does not already exist in the database.

        This method starts and ends a transaction.
        """
        user = User(name=name, otp_seed=otp_seed)
        session.execute(text("BEGIN IMMEDIATE"))
        session.add(user)
        session.commit()
        return user

    def update_last_2fa_code(self, session: Session, last_2fa_code: str):
        """
        This method commits.
        """
        self.last_2fa_time = time.time()
        self.last_2fa_code = last_2fa_code
        session.commit()


class JobrunnerStatus(Base):
    __tablename__ = "jobrunner_status"

    id: Mapped[int] = mapped_column(primary_key=True)
    pid: Mapped[Optional[int]]
    status: Mapped[str]
    job_id: Mapped[Optional[int]]

    @classmethod
    def get(self, session: Session) -> "JobrunnerStatus":
        return session.scalar(select(JobrunnerStatus))

    @classmethod
    def set(
        self,
        session: Session,
        status: str,
        job_id: Optional[int] = None,
        clear_pid: bool = False,
    ):
        session.execute(delete(JobrunnerStatus))
        pid = None if clear_pid else os.getpid()
        status = JobrunnerStatus(pid=pid, status=status, job_id=job_id)
        session.add(status)
        session.commit()


class AlreadyFinished(Exception):
    pass


class JobType(enum.Enum):
    BACKPORT = "backport"
    TRAIN = "train"


class Job(Base):
    __tablename__ = "job"

    id: Mapped[int] = mapped_column(primary_key=True)
    type: Mapped[JobType] = mapped_column(
        Enum(JobType, length=50),
        server_default="BACKPORT",
        nullable=False,
    )
    user: Mapped[str]
    command: Mapped[str]  # Expected to be a JSON-encoded list.
    queued_at: Mapped[float] = mapped_column(default=time.time)
    started_at: Mapped[Optional[float]]
    finished_at: Mapped[Optional[float]]
    exit_status: Mapped[Optional[int]]
    status: Mapped[Optional[str]]
    data: Mapped[Optional[str]]  # Optional auxiliary information in JSON format

    @classmethod
    def add(
        cls, type: JobType, session, user: str, command: List[str], data=None
    ) -> int:
        """Create a new Job record and add it to the database.

        :returns: The job id
        """
        job = Job(
            type=type, user=user, command=json.dumps(command), data=json.dumps(data)
        )
        session.execute(text("BEGIN IMMEDIATE"))
        session.add(job)
        session.commit()
        return job.id

    @classmethod
    def get(cls, session, job_id) -> Optional["Job"]:
        return session.scalar(select(Job).where(Job.id == job_id))

    @classmethod
    def get_jobs(cls, session, limit: int, skip: int) -> List["Job"]:
        return [
            job
            for job in session.scalars(
                select(Job).order_by(Job.id.desc()).limit(limit).offset(skip)
            )
        ]

    @classmethod
    def pop(cls, session) -> Optional["Job"]:
        """
        Return the next eligible job to run, if any.

        This method starts and ends a transaction.
        """
        # This must executed as an atomic read-modify-write operation.
        # We use BEGIN IMMEDIATE to serialize concurrent processes.
        #   xref: https://www.sqlite.org/lang_transaction.html
        session.execute(text("BEGIN IMMEDIATE"))
        stmt = select(Job).where(Job.started_at == null()).order_by(Job.id).limit(1)
        job = session.scalar(stmt)
        if not job:
            session.rollback()
            return

        # Claim the job by setting started_at
        job.started_at = time.time()
        session.commit()

        return job

    def signal(self, session: Session, user: str, type: str):
        """
        This method starts and ends a transaction.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        if self.finished_at:
            session.rollback()
            raise AlreadyFinished(
                f"Job {self.id} cannot be signalled because it has already finished"
            )

        Interruption.add(session, self.id, user, type)
        session.commit()

    def extract_status(self) -> dict:
        if not self.status:
            return {"status": None, "progress": None}
        try:
            return json.loads(self.status)
        except json.decoder.JSONDecodeError:
            # Assume old style status which was a plain string
            return {"status": self.status, "progress": None}

    def _set_status(self, session: Session, status: dict):
        """
        This method starts and ends a transaction.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        self.status = json.dumps(status)
        session.commit()

    def set_status(self, session: Session, status: Optional[str]):
        """
        Note: set_status clears out progress information.

        This method starts and ends a transaction.
        """
        rec = self.extract_status()
        rec["status"] = status
        rec["progress"] = None
        self._set_status(session, rec)

    def set_progress(self, session: Session, data: Optional[dict]):
        """
        This method starts and ends a transaction.
        """
        rec = self.extract_status()
        rec["progress"] = data
        self._set_status(session, rec)

    def finish(self, session: Session, exit_status: Optional[int]):
        """
        This method starts and ends a transaction.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        self.exit_status = exit_status
        self.finished_at = time.time()

        # Clean up any unprocessed interactions and interruptions
        session.execute(delete(Interaction).where(Interaction.job_id == self.id))
        session.execute(delete(Interruption).where(Interruption.job_id == self.id))
        session.commit()


class Interruption(Base):
    __tablename__ = "interruption"

    id: Mapped[int] = mapped_column(primary_key=True)
    job_id: Mapped[int] = mapped_column(ForeignKey("job.id"))
    created_at: Mapped[float] = mapped_column(default=time.time)
    user: Mapped[str]
    type: Mapped[str]  # "kill" or "interrupt"

    SIGNAL_TYPES = ["kill", "interrupt"]

    @classmethod
    def add(cls, session: Session, job_id: int, user: str, type: str):
        """
        Adds a new Interruption to the session.

        Does NOT commit.
        """
        assert type in cls.SIGNAL_TYPES
        session.add(Interruption(job_id=job_id, user=user, type=type))

    @classmethod
    def peek(cls, session: Session, job_id: int) -> Optional["Interruption"]:
        return session.scalar(
            select(Interruption)
            .where(Interruption.job_id == job_id)
            .order_by(Interruption.id)
            .limit(1)
        )

    @classmethod
    def pop(cls, session: Session, job_id: int) -> Optional["Interruption"]:
        """
        Removes and returns the next unprocessed interuption, if any.

        This method starts and ends a transaction.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        i = Interruption.peek(session, job_id)
        if not i:
            session.rollback()
            return
        session.delete(i)
        session.commit()
        return i


class AlreadyResponded(Exception):
    pass


class Interaction(Base):
    __tablename__ = "interaction"

    id: Mapped[int] = mapped_column(primary_key=True)
    job_id: Mapped[int] = mapped_column(ForeignKey("job.id"))

    type: Mapped[str]  # "input_line" or "choices"
    prompt: Mapped[str]

    choices: Mapped[Optional[str]]  # JSON-encoded list, only set when type == "choices"
    default: Mapped[Optional[str]]

    responded_by: Mapped[Optional[str]]
    response: Mapped[Optional[str]]

    @hybrid_property
    def choices_parsed(self) -> dict:
        if self.type == "choices":
            return json.loads(self.choices)
        return {}

    @classmethod
    def register(
        self,
        session: Session,
        job_id: int,
        type: str,
        prompt: str,
        choices: Optional[List[str]] = None,
        default: Optional[str] = None,
    ):
        i = Interaction(
            job_id=job_id,
            type=type,
            prompt=prompt,
            choices=json.dumps(choices) if choices else None,
            default=default,
        )
        session.add(i)
        session.commit()

    @classmethod
    def lookup_pending(self, session: Session, job_id: int) -> Optional["Interaction"]:
        """
        Locate a non-responded-to interaction for the specified job id.
        """
        stmt = (
            select(Interaction)
            .where(Interaction.job_id == job_id)
            .where(Interaction.response == null())
            .limit(1)
        )
        return session.scalar(stmt)

    @classmethod
    def pop_responded(self, session: Session, job_id) -> Optional["Interaction"]:
        """
        If there is a responded-to interaction, delete it from the database and
        return it.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        stmt = (
            select(Interaction)
            .where(Interaction.job_id == job_id)
            .where(Interaction.response != null())
            .limit(1)
        )
        i = session.scalar(stmt)
        if i is None:
            session.rollback()
            return

        session.delete(i)
        session.commit()
        return i

    def respond(self, session: Session, responded_by: str, response: str):
        session.execute(text("BEGIN IMMEDIATE"))
        if self.response is not None:
            session.rollback()
            raise AlreadyResponded(
                f"{self.responded_by} already responded to interaction id {self.id}"
            )
        self.responded_by = responded_by
        self.response = response
        session.commit()


class TrainGroup(BasePydantic):
    name: str
    versions: List[str]
    wikis: List[str]
    selected_wikis: List[str] = Field(
        default_factory=lambda data: data["wikis"],
    )
    has_rolled: bool = False


class TrainStatus(BasePydantic):
    groups: List[TrainGroup]
    version: str
    old_version: Optional[str]
    at_group: Optional[str]
    wiki_versions: Dict[str, str]
    wiki_count: int = 0
    task: Optional[str]
    task_url: Optional[str]
    task_status: Optional[str]
    warnings: List[str]

    @classmethod
    def from_info(cls, info: train.TrainInfo):
        rolled_group_i = -1
        if info.train_is_at is not None:
            try:
                rolled_group_i = train.GROUPS.index(info.train_is_at)
            except ValueError:
                pass

        return cls(
            groups=[
                TrainGroup(
                    name=name,
                    versions=[version for version in info.groups[name]],
                    wikis=[wiki for wiki in info.group_wikis[name]],
                    has_rolled=rolled_group_i >= i,
                )
                for i, name in enumerate(train.GROUPS)
            ],
            version=info.train_version,
            old_version=info.old_version,
            at_group=info.train_is_at,
            wiki_versions=info.wiki_versions,
            wiki_count=len(info.wiki_versions),
            task=info.task,
            task_url=info.task_url,
            task_status=info.task_status,
            warnings=info.warnings,
        )


class TrainPromotion(BasePydantic):
    group: str
    version: str
    old_version: str
    excluded_wikis: List[str] = []
    original_train_status: TrainStatus

    def add_job(self, **kwargs) -> int:
        return Job.add(
            JobType.TRAIN,
            command=self.command,
            data=self.data,
            **kwargs,
        )

    @property
    def command(self):
        cmd = [
            "scap",
            "deploy-promote",
            "--yes",
            "--train",
            "--old-version",
            self.old_version,
        ]

        if self.excluded_wikis:
            cmd += ["--exclude-wikis", ",".join(self.excluded_wikis)]

        return cmd + [self.group, self.version]

    @property
    def data(self):
        return self.dict(by_alias=True)


def setup_db(engine, db_filename):
    Base.metadata.create_all(engine)

    # Ensure that the database is group writable
    if os.geteuid() == os.stat(db_filename).st_uid:
        os.chmod(db_filename, 0o660)

    migrate_db(engine)


def migrate_db(engine):
    logger = logging.getLogger()
    logger.info("Running db migration (if needed)")

    # xref https://alembic.sqlalchemy.org/en/latest/cookbook.html#run-alembic-operation-objects-directly-as-in-from-autogenerate
    with engine.connect() as conn:
        mc = MigrationContext.configure(conn)

        migrations = produce_migrations(mc, Base.metadata)

        operations = Operations(mc)

        use_batch = engine.name == "sqlite"

        stack = [migrations.upgrade_ops]
        while stack:
            elem = stack.pop(0)

            if use_batch and isinstance(elem, ModifyTableOps):
                with operations.batch_alter_table(
                    elem.table_name, schema=elem.schema
                ) as batch_ops:
                    for table_elem in elem.ops:
                        batch_ops.invoke(table_elem)

            elif hasattr(elem, "ops"):
                stack.extend(elem.ops)
            else:
                operations.invoke(elem)
        conn.commit()

    logger.info("db migration finished")
