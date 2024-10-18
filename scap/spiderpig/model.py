import datetime
import json
import os

from typing import List, Optional
from sqlalchemy import ForeignKey, select, delete, text, null
from sqlalchemy.ext.hybrid import hybrid_property
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, Session
from sqlalchemy.sql import func


class Base(DeclarativeBase):
    pass


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
        # FIXME: Change to None once I figure out db migration.
        pid = 0 if clear_pid else os.getpid()
        status = JobrunnerStatus(pid=pid, status=status, job_id=job_id)
        session.add(status)
        session.commit()


class AlreadyFinished(Exception):
    pass


class Job(Base):
    __tablename__ = "job"

    id: Mapped[int] = mapped_column(primary_key=True)
    user: Mapped[str]
    command: Mapped[str]  # Expected to be a JSON-encoded list.
    queued_at: Mapped[datetime.datetime] = mapped_column(server_default=func.now())
    started_at: Mapped[Optional[datetime.datetime]]
    finished_at: Mapped[Optional[datetime.datetime]]
    exit_status: Mapped[Optional[int]]
    status: Mapped[Optional[str]]

    @classmethod
    def add(cls, session, user: str, command: List[str]) -> int:
        """Create a new Job record and add it to the database.

        :returns: The job id
        """
        job = Job(user=user, command=json.dumps(command))
        session.add(job)
        session.commit()
        return job.id

    @classmethod
    def get(cls, session, job_id) -> Optional["Job"]:
        return session.scalar(select(Job).where(Job.id == job_id))

    @classmethod
    def get_last_n(cls, session, last: int) -> List["Job"]:
        return [
            job
            for job in session.scalars(select(Job).order_by(Job.id.desc()).limit(last))
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
        job.started_at = func.now()
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

    def set_status(self, session: Session, status: Optional[str]):
        """
        This method starts and ends a transaction.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        self.status = status
        session.commit()

    def finish(self, session: Session, exit_status: Optional[int]):
        """
        This method starts and ends a transaction.
        """
        session.execute(text("BEGIN IMMEDIATE"))
        self.exit_status = exit_status
        self.finished_at = func.now()

        # Clean up any unprocessed interactions and interruptions
        session.execute(delete(Interaction).where(Interaction.job_id == self.id))
        session.execute(delete(Interruption).where(Interruption.job_id == self.id))
        session.commit()


class Interruption(Base):
    __tablename__ = "interruption"

    id: Mapped[int] = mapped_column(primary_key=True)
    job_id: Mapped[int] = mapped_column(ForeignKey("job.id"))
    created_at: Mapped[Optional[datetime.datetime]] = mapped_column(
        server_default=func.now()
    )
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
        return json.loads(self.choices)

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


def setup_db(engine, db_filename):
    Base.metadata.create_all(engine)

    # Ensure that the database is group writable
    if os.geteuid() == os.stat(db_filename).st_uid:
        os.chmod(db_filename, 0o664)
