import asyncio
from datetime import datetime, timezone, timedelta
import functools
import jwt
import os
import pyotp
import subprocess

import sys

if sys.version_info < (3, 9):
    from typing_extensions import Annotated
else:
    from typing import Annotated

from typing import List, Optional

from fastapi import Depends, FastAPI, HTTPException, Query, Body, status
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm

from sqlalchemy.orm import Session

from scap import cli, utils, gerrit
import scap.spiderpig
from scap.spiderpig.model import JobrunnerStatus, Job, Interaction, AlreadyResponded


def get_pyotp() -> pyotp.TOTP:
    seedfile = os.path.expanduser("~/.spiderpig-seed")

    if os.path.exists(seedfile):
        with open(seedfile) as f:
            seed = f.read()
    else:
        seed = pyotp.random_base32()
        with open(seedfile, "w") as f:
            os.chmod(seedfile, 0o600)
            f.write(seed)

    return pyotp.TOTP(seed)


def validate_otp(username, supplied_otp) -> bool:
    expected_user = utils.get_username()
    if username != expected_user:
        return False
    return get_pyotp().verify(supplied_otp)


@cli.command(
    "spiderpig-otp",
    help="Generate a one-time password for logging into spiderpig",
    primary_deploy_server_only=True,
)
class SpiderpigOTP(cli.Application):
    def main(self, *extra_args):
        otp = get_pyotp().now()
        print(f"Login: {utils.get_username()}")
        print(f"Password: {otp}")


@cli.command(
    "spiderpig-apiserver",
    help="Run the SpiderPig API server",
    primary_deploy_server_only=True,
)
class SpiderpigAPIServer(cli.Application):
    @cli.argument(
        "--host",
        default="0.0.0.0",
        help="The IP address to listen on.  Default is 0.0.0.0",
    )
    @cli.argument(
        "--port",
        type=int,
        default=8000,
        help="The port to listen on",
    )
    @cli.argument(
        "--dev",
        action="store_true",
        help=f"Enable development mode (Automatic reload of {__file__} (or anything it imports) when changed, open CORS)",
    )
    def main(self, *extra_args):
        if self.arguments.dev:
            # Change to the directory containing this file.  This seems
            # to be required to make fastapi dev monitor it for changes.
            os.chdir(os.path.dirname(__file__))

        env = os.environ.copy()
        env.update(
            {
                "SPIDERPIG_GERRIT_URL": self.config["gerrit_url"],
                "SPIDERPIG_JOB_LOG_DIR": self.spiderpig_joblogdir(),
                "SPIDERPIG_DBFILE": self.spiderpig_dbfile(),
                "SPIDERPIG_JWT_KEY_FILE": self.spiderpig_jwt_secret_file(),
            }
        )
        if self.arguments.dev:
            env["SPIDERPIG_OPEN_CORS"] = "1"

        cmd = [
            "fastapi",
            "dev" if self.arguments.dev else "run",
            "--host",
            self.arguments.host,
            "--port",
            str(self.arguments.port),
            __file__,
        ]
        subprocess.run(cmd, env=env)


app = FastAPI()
if os.getenv("SPIDERPIG_OPEN_CORS"):
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_headers=["authorization"],
        allow_methods=["GET", "POST"],
    )


# From the tutorial:
# engine = create_engine(
#     SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
# )
def _get_session():
    return Session(scap.spiderpig.engine(os.environ["SPIDERPIG_DBFILE"]))


def get_session():
    session = _get_session()
    try:
        yield session
    finally:
        session.close()


ALGORITHM = "HS256"
ISSUER = "spiderpig-auth"


@functools.lru_cache()
def get_jwt_secret_key() -> str:
    filename = os.getenv("SPIDERPIG_JWT_KEY_FILE")

    if os.path.exists(filename):
        with open(filename) as f:
            key = f.read()
    else:
        key = os.urandom(32).hex()  # 256 random bits
        with open(filename, "w") as f:
            os.chmod(filename, 0o660)
            f.write(key)

    return key


def generate_token() -> str:
    now = datetime.now(tz=timezone.utc)
    claims = {
        "sub": utils.get_username(),
        "exp": now + timedelta(hours=1),
        "iat": now,
        "iss": ISSUER,
    }
    return jwt.encode(claims, get_jwt_secret_key(), algorithm=ALGORITHM)


def validate_token(token: str) -> Optional[str]:
    """
    Returns the subject (the user) of the token if it is valid.
    """
    try:
        claims = jwt.decode(
            token,
            get_jwt_secret_key(),
            algorithms=[ALGORITHM],
            options={
                "verify_iss": True,
                "verify_exp": True,
                "verify_iat": True,
            },
            issuer=ISSUER,
        )
    except jwt.InvalidTokenError:
        return None

    return claims.get("sub")


oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/login")


async def get_current_user(token: Annotated[str, Depends(oauth2_scheme)]):
    user = validate_token(token)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user


async def get_job_by_id(job_id: int, session: Session = Depends(get_session)):
    job = Job.get(session, job_id)
    if job is None:
        raise HTTPException(
            status_code=404,
            detail={
                "message": "No such job",
                "job_id": job_id,
            },
        )
    return job


def get_parsed_interaction(session, job) -> Optional[dict]:
    i = Interaction.lookup_pending(session, job.id)
    if not i:
        return None
    choices_parsed = i.choices_parsed
    i = jsonable_encoder(i)
    i["choices"] = choices_parsed
    return i


@app.get("/api/jobrunner/status")
async def jobrunner_status(
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
):
    def pid_exists(pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            # No such process
            return False

    status = JobrunnerStatus.get(session)

    if not status.pid or not pid_exists(status.pid):
        return {
            "status": "Jobrunner not running",
            "job": None,
            "pending_interaction": None,
        }

    job = None
    i = None

    if status.job_id:
        job = Job.get(session, status.job_id)
        if job:
            i = get_parsed_interaction(session, job)

    return {
        "status": status.status,
        "job": job,
        "pending_interaction": i,
    }


@app.post("/api/jobs/train")
async def start_train(
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
):
    job_id = Job.add(
        session,
        user=user,
        command=[
            "scap",
            "train",
        ],
    )
    return {
        "message": "Job created",
        "id": job_id,
    }


@app.post("/api/jobs/backport")
async def start_backport(
    change_url: Annotated[
        List[str],
        Query(
            title="A change to backport",
            description="The URL or change number of the change to backport",
        ),
    ],
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
):
    gerritsession = gerrit.GerritSession(url=os.getenv("SPIDERPIG_GERRIT_URL"))

    # NOTE: 'change_url' is a list
    baddies = [
        url for url in change_url if gerritsession.change_number_from_url(url) is None
    ]
    if baddies:
        raise HTTPException(
            status_code=400,
            detail={
                "message": "Invalid change url or number",
                "baddies": baddies,
            },
        )

    job_id = Job.add(session, user=user, command=["scap", "backport"] + change_url)
    return {
        "message": "Job created",
        "id": job_id,
    }


@app.get("/api/jobs")
async def get_jobs(
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
    last: int = 10,
):
    return {
        "jobs": Job.get_last_n(session, last),
    }


@app.get("/api/jobs/{job_id}")
async def get_job(
    job: Annotated[Job, Depends(get_job_by_id)],
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
):
    i = get_parsed_interaction(session, job)

    return {
        "job": job,
        "pending_interaction": i,
    }


@app.post("/api/jobs/{job_id}/interact/{interaction_id}")
async def interact(
    job: Annotated[Job, Depends(get_job_by_id)],
    interaction_id: int,
    response: Annotated[str, Body()],
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
):
    i = Interaction.lookup_pending(session, job.id)
    if i is None or i.id != interaction_id:
        raise HTTPException(
            status_code=404,
            detail={
                "message": "No pending interactions",
                "job_id": job.id,
                "interaction_id": interaction_id,
            },
        )

    try:
        i.respond(session, user, response)
    except AlreadyResponded:
        raise HTTPException(
            status_code=409,
            detail={
                "message": "Interaction already responded to",
                "job_id": job.id,
                "interaction_id": interaction_id,
                "responded_by": i.responded_by,
                "response": i.response,
            },
        )

    return {
        "message": "Response stored",
        "job_id": job.id,
        "interaction_id": interaction_id,
    }


@app.get("/api/jobs/{job_id}/log")
async def get_log(
    job_id: int,
    user: Annotated[str, Depends(get_current_user)],
):
    with _get_session() as session:
        # Validate the job_id
        await get_job_by_id(job_id, session)

    async def log_streamer():
        polling_interval = 1  # seconds

        logfile = os.path.join(os.environ["SPIDERPIG_JOB_LOG_DIR"], f"{job_id}.log")

        # Wait for the logfile to appear
        while True:
            if os.path.exists(logfile):
                break
            await asyncio.sleep(polling_interval)
        # Logfile has appeared.

        with open(logfile) as f:
            while True:
                got = f.read(65536)
                if got:
                    yield got
                    continue
                # EOF

                with _get_session() as session:
                    job = await get_job_by_id(job_id, session)
                    if job.finished_at:
                        return

                await asyncio.sleep(polling_interval)

    return StreamingResponse(log_streamer())


@app.post("/api/jobs/{job_id}/signal/{type}")
async def signal_job(
    type: str,
    job: Annotated[Job, Depends(get_job_by_id)],
    user: Annotated[str, Depends(get_current_user)],
    session: Session = Depends(get_session),
):
    job.signal(session, user, type)
    return {
        "message": "Interrupted",
        "job_id": job.id,
    }


# FIXME: Rate limiting
@app.post("/api/login")
async def login(form_data: Annotated[OAuth2PasswordRequestForm, Depends()]):
    if not validate_otp(form_data.username, form_data.password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = generate_token()
    return {"message": "Login successful", "token": token}
