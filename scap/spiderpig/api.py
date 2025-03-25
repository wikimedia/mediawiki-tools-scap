import asyncio
import base64
import cas
from datetime import datetime
import fcntl
import functools
import html
import json
import os
import pyotp
import re
import subprocess
import sys
import time
import urllib.parse

if sys.version_info < (3, 9):
    from typing_extensions import Annotated
else:
    from typing import Annotated

from typing import List, Optional

from pydantic import BaseModel

from fastapi import (
    Depends,
    FastAPI,
    HTTPException,
    Request,
    Query,
    Body,
)
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.sessions import SessionMiddleware
from fastapi.responses import (
    FileResponse,
    StreamingResponse,
    RedirectResponse,
    JSONResponse,
)
from fastapi.staticfiles import StaticFiles

from sqlalchemy.orm import Session

from scap import cli, utils, gerrit

import scap.spiderpig
from scap.spiderpig.model import (
    JobrunnerStatus,
    Job,
    Interaction,
    AlreadyResponded,
    User,
)

OTP_TIMEOUT = 60


def get_or_init_dbuser(session: Session, username: str) -> User:
    user = User.get(session, username)
    if not user:
        user = User.add(session, username, pyotp.random_base32())
    return user


def get_pyotp(dbuser: User) -> pyotp.TOTP:
    return pyotp.TOTP(dbuser.otp_seed, interval=OTP_TIMEOUT)


def validate_otp(dbuser: User, supplied_otp: str) -> bool:
    if (
        supplied_otp == dbuser.last_2fa_code
        and time.time() < dbuser.last_2fa_time + OTP_TIMEOUT
    ):
        # Don't allow reuse of the last code if it is still within its validity window.
        return False

    return get_pyotp(dbuser).verify(supplied_otp)


@cli.command(
    "spiderpig-otp",
    help="Generate a one-time password for logging into spiderpig",
    primary_deploy_server_only=True,
)
class SpiderpigOTP(cli.Application):
    def main(self, *extra_args):
        username = utils.get_username()

        # Set SPIDERPIG_DBFILE in the environment so that get_db_session()
        # will work.
        os.environ["SPIDERPIG_DBFILE"] = self.spiderpig_dbfile()

        with _get_db_session() as session:
            user = get_or_init_dbuser(session, username)
            totp = get_pyotp(user)
            now = datetime.now().timestamp()
            otp = totp.at(now)
            time_remaining = totp.interval - now % totp.interval
            print(f"Login: {utils.get_username()}")
            print(f"Password: {otp}  (Expires in {round(time_remaining)} seconds)")


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
        env["SPIDERPIG_SCAP_CONFIG"] = utils.string_to_base64_string(
            json.dumps(self.config)
        )
        env["SPIDERPIG_SCAP_FLAGS"] = utils.string_to_base64_string(
            json.dumps(self.format_passthrough_args())
        )

        env.update(
            {
                "SPIDERPIG_DIR": self.spiderpig_dir(),
                "SPIDERPIG_JOB_LOG_DIR": self.spiderpig_joblogdir(),
                "SPIDERPIG_DBFILE": self.spiderpig_dbfile(),
                "SPIDERPIG_SESSION_KEY_FILE": self.spiderpig_session_secret_file(),
            }
        )
        if self.arguments.dev:
            env["SPIDERPIG_OPEN_CORS"] = "1"

        scap_bin_dir = os.path.dirname(os.path.realpath(sys.argv[0]))
        fastapi_exe = os.path.join(scap_bin_dir, "fastapi")

        cmd = [
            fastapi_exe,
            "dev" if self.arguments.dev else "run",
            "--host",
            self.arguments.host,
            "--port",
            str(self.arguments.port),
            __file__,
        ]
        subprocess.run(cmd, env=env)


app = FastAPI()

#####################
# SUPPORT FUNCTIONS #
#####################


def read_or_initialize_file(filename, initial_value_func) -> str:
    """
    initial_value_func must return a string.
    """
    # This results in openat(AT_FDCWD, filename, O_RDWR|O_CREAT|O_APPEND|O_CLOEXEC, 0666).
    with open(filename, "a+") as f:
        fd = f.fileno()
        # Acquire read lock
        fcntl.lockf(fd, fcntl.LOCK_SH)

        try:
            f.seek(0)
            got = f.read()
            if len(got):
                return got
        finally:
            fcntl.lockf(fd, fcntl.LOCK_UN)
        # Read lock has been released.

        # Acquire exclusive lock to maybe initialize the file.
        fcntl.lockf(fd, fcntl.LOCK_EX)

        try:
            # It is possible that another process has finished
            # initializing the file between the time we released the read lock and acquired
            # the exclusive lock, so check again.
            f.seek(0)
            got = f.read()
            if len(got):
                return got

            # The file is still empty.  Initialize it.
            os.chmod(filename, 0o660)

            got = initial_value_func()
            f.write(got)

            return got

        finally:
            fcntl.lockf(fd, fcntl.LOCK_UN)


def reset_file(filename):
    """
    Truncate the specified file while holding an exclusive lock on it.
    """
    with open(filename, "a+") as f:
        fd = f.fileno()
        fcntl.lockf(fd, fcntl.LOCK_EX)
        try:
            os.ftruncate(fd, 0)
        finally:
            fcntl.lockf(fd, fcntl.LOCK_UN)


def get_session_key() -> str:
    filename = os.getenv("SPIDERPIG_SESSION_KEY_FILE")

    def generate_session_key():
        return os.urandom(32).hex()  # 256 random bits

    return read_or_initialize_file(filename, generate_session_key)


def session_epoch_filename():
    return os.path.join(os.getenv("SPIDERPIG_DIR"), "session-epoch")


def get_current_epoch() -> str:
    def generate_epoch():
        return os.urandom(32).hex()  # 256 random bits

    return read_or_initialize_file(session_epoch_filename(), generate_epoch)


def reset_current_epoch():
    reset_file(session_epoch_filename())


@functools.lru_cache()
def get_scap_config():
    return json.loads(base64.b64decode(os.getenv("SPIDERPIG_SCAP_CONFIG")))


@functools.lru_cache()
def get_scap_flags():
    return json.loads(base64.b64decode(os.getenv("SPIDERPIG_SCAP_FLAGS")))


def _get_db_session():
    # Using check_same_thread=False allows FastAPI to use the same SQLite database
    # in different threads. This is necessary as one single request could use more
    # than one thread (for example in dependencies).
    # xref: https://fastapi.tiangolo.com/tutorial/sql-databases/#create-an-engine
    return Session(
        scap.spiderpig.engine(
            os.environ["SPIDERPIG_DBFILE"], connect_args={"check_same_thread": False}
        )
    )


def get_db_session():
    session = _get_db_session()
    try:
        yield session
    finally:
        session.close()


def guess_login_url(request):
    host_header = request.headers.get("Host")
    if not host_header:
        raise HTTPException(
            status_code=400, detail="HTTP request did not include the Host header"
        )
    host = host_header.split(":")[0]
    if host == "localhost":
        scheme = "http"
    else:
        scheme = "https"
    return f"{scheme}://{host_header}/api/login"


def cas_client(request: Request, next=None) -> cas.CASClientV3:
    service_url = guess_login_url(request)

    if next:
        next = urllib.parse.urlencode({"next": next})
        service_url += f"?{next}"

    server_url = get_scap_config()["spiderpig_auth_server"]
    if not server_url:
        raise Exception(
            "spiderpig_auth_server must be set up scap.cfg for SpiderPig authentication to work"
        )

    # server_url must end with a trailing slash for proper operation.
    if not server_url.endswith("/"):
        server_url += "/"

    return cas.CASClientV3(
        server_url=server_url,
        service_url=service_url,
    )


@functools.lru_cache()
def get_config_groups(setting: str) -> set:
    return set([group.strip() for group in get_scap_config()[setting].splitlines()])


def get_spiderpig_user_groups() -> set:
    return get_config_groups("spiderpig_user_groups")


def get_spiderpig_admin_groups() -> set:
    return get_config_groups("spiderpig_admin_groups")


def user_is_authorized(user_groups) -> bool:
    # Admin users are automatically authorized.
    if set(user_groups).intersection(
        get_spiderpig_user_groups()
    ) or user_in_admin_group(user_groups):
        return True
    return False


def user_in_admin_group(user_groups) -> bool:
    return (
        True if set(user_groups).intersection(get_spiderpig_admin_groups()) else False
    )


# This object is stored in a cookie in the client browser, so don't
# include any secret information in here.
class SessionUser(BaseModel):
    name: str  # The shell username
    groups: List[str]
    fully_authenticated: bool = False

    @property
    def isAdmin(self) -> bool:
        return user_in_admin_group(self.groups)


class NotAuthenticatedException(Exception):
    def __init__(self, need2fa=False, user: SessionUser = None):
        self.need2fa = need2fa
        self.user = user


ROUTE_2FA = "/api/2fa"


@app.exception_handler(NotAuthenticatedException)
async def authn_exception_handler(request: Request, exc: NotAuthenticatedException):
    if exc.need2fa:
        return JSONResponse(
            status_code=401,
            content={
                "code": "need2fa",
                "message": "Please complete 2FA first",
                "url": ROUTE_2FA,
                "user": exc.user.name,
            },
        )

    return JSONResponse(
        status_code=401,
        content={
            "code": "needauth",
            "message": "Please log in first",
            "url": cas_client(request, request.headers.get("Referer")).get_login_url(),
        },
    )


def maybe_get_current_user(request: Request) -> Optional[SessionUser]:
    """
    Returns a SessionUser object which may or may not have completed
    authentication.  Returns None if there is no current user.
    """
    user = request.session.get("user")
    if not user:
        return None

    # Discard user information from other epochs.
    if request.session.get("epoch") != get_current_epoch():
        return None

    return SessionUser(**user)


def get_current_user(request: Request) -> SessionUser:
    """
    Returns a fully authenticated SessionUser object, or
    raises NotAuthenticatedException if there is no current user
    or if the current user it not fully authenticated.
    """
    user = maybe_get_current_user(request)

    if not user:
        raise NotAuthenticatedException()
    if not user.fully_authenticated:
        raise NotAuthenticatedException(need2fa=True, user=user)

    return user


def get_admin_user(request: Request) -> SessionUser:
    user = get_current_user(request)
    if user.isAdmin:
        return user
    raise HTTPException(
        status_code=403,
        detail={
            "message": "Admin privileges are required for this operation",
        },
    )


async def get_job_by_id(job_id: int, session: Session = Depends(get_db_session)):
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


def get_gerrit_session(config: Annotated[dict, Depends(get_scap_config)]):
    return gerrit.GerritSession(url=config["gerrit_url"])


def get_parsed_interaction(session, job) -> Optional[dict]:
    i = Interaction.lookup_pending(session, job.id)
    if not i:
        return None
    choices_parsed = i.choices_parsed
    i = jsonable_encoder(i)
    i["choices"] = choices_parsed
    return i


def linkify_commit_mesage(commit_message) -> List:
    scap_config = get_scap_config()
    phorge_url = scap_config["phorge_url"]
    gerrit_url = scap_config["gerrit_url"]

    # Extracted and converted from Gerrit's commentlink config.
    LINKDEFS = {
        "phabricator": {
            "match": re.compile('\\bT(\\d+)(#\\d+)?\\b(?![#"]|</a>)'),
            "link": f"{phorge_url}/T\\1\\2",
        },
        "changeid": {
            "match": re.compile("\\b(I[0-9a-f]{7,40})\\b"),
            "link": f"{gerrit_url}q/\\1",
        },
    }

    res = [html.escape(commit_message)]

    index = 0
    while index < len(res):
        current = res[index]
        if not isinstance(current, str):
            index += 1
            continue

        linkAdded = False
        for linksettings in LINKDEFS.values():
            m = linksettings["match"].search(current)
            if m:
                link = {
                    "type": "link",
                    "href": m.expand(linksettings["link"]),
                    "text": m.group(),
                }

                before = current[: m.start()]
                after = current[m.end() :]

                res = res[0:index] + [before, link, after] + res[index + 1 :]

                linkAdded = True
                break

        if not linkAdded:
            index += 1

    return res


##########
# ROUTES #
##########


@app.get("/api/jobrunner/status")
async def jobrunner_status(
    session: Session = Depends(get_db_session),
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
    user: Annotated[SessionUser, Depends(get_current_user)],
    session: Session = Depends(get_db_session),
):
    job_id = Job.add(
        session,
        user=user.name,
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
    user: Annotated[SessionUser, Depends(get_current_user)],
    gerritsession: Annotated[gerrit.GerritSession, Depends(get_gerrit_session)],
    session: Session = Depends(get_db_session),
):
    baddies = []
    change_infos = []

    # NOTE: 'change_url' is a list
    for url in change_url:
        change_num = gerritsession.change_number_from_url(url)
        if change_num is None:
            baddies.append(url)
            continue

        change = gerritsession.change_detail(change_num).get()
        change_infos.append(
            {
                "number": change._number,
                "project": change.project,
                "branch": change.branch,
                "subject": change.subject,
                "commit_msg": change.revisions[change.current_revision]["commit"][
                    "message"
                ],
                "url": os.path.join(gerritsession.url, f"c/{change._number}"),
            }
        )

    if baddies:
        raise HTTPException(
            status_code=400,
            detail={
                "message": "Invalid change url or number",
                "baddies": baddies,
            },
        )

    operation = "fake-backport" if get_scap_config()["local_dev_mode"] else "backport"

    job_id = Job.add(
        session,
        user=user.name,
        command=["scap", operation] + get_scap_flags() + change_url,
        data={"change_infos": change_infos},
    )
    return {
        "message": "Job created",
        "id": job_id,
    }


@app.get("/api/jobs")
async def get_jobs(
    session: Session = Depends(get_db_session),
    limit: int = 10,
    skip: Optional[int] = 0,
):
    jobs = Job.get_jobs(session, limit, skip)

    for job in jobs:
        # Detach the job object from the SQLAlchemy session so that we can
        # safely modify it without the changes being committed back to the
        # database.
        session.expunge(job)
        job.interaction = get_parsed_interaction(session, job)
        job.data = json.loads(job.data)

    return {
        "jobs": jobs,
    }


@app.get("/api/jobs/{job_id}")
async def get_job(
    job: Annotated[Job, Depends(get_job_by_id)],
    session: Session = Depends(get_db_session),
):
    scap_config = get_scap_config()
    gerrit_url = scap_config["gerrit_url"]

    i = get_parsed_interaction(session, job)

    data = json.loads(job.data)
    for change_info in data["change_infos"]:
        change_info["linkifiedCommitMsg"] = linkify_commit_mesage(
            change_info["commit_msg"]
        )
        project = change_info["project"]
        branch = change_info["branch"]
        change_info["repoQueryUrl"] = os.path.join(
            gerrit_url,
            "q/" + urllib.parse.quote_plus(f"project:{project}"),
        )
        change_info["branchQueryUrl"] = os.path.join(
            gerrit_url,
            "q/" + urllib.parse.quote_plus(f"project:{project} branch:{branch}"),
        )

    job.data = data

    return {
        "job": job,
        "pending_interaction": i,
    }


@app.post("/api/jobs/{job_id}/interact/{interaction_id}")
async def interact(
    job: Annotated[Job, Depends(get_job_by_id)],
    interaction_id: int,
    response: Annotated[str, Body()],
    user: Annotated[SessionUser, Depends(get_current_user)],
    session: Session = Depends(get_db_session),
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
        i.respond(session, user.name, response)
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
    include_sensitive: bool = False,
):
    with _get_db_session() as session:
        # Validate the job_id
        await get_job_by_id(job_id, session)

    def toRecord(record: dict) -> str:
        return json.dumps(record) + "\n"

    async def log_streamer():
        polling_interval = 1  # seconds

        logfile = os.path.join(
            os.environ["SPIDERPIG_JOB_LOG_DIR"], f"{job_id}.log.jsonl"
        )

        yield toRecord({"type": "connected"})

        # Wait for the logfile to appear
        notified_waiting = False
        while True:
            if os.path.exists(logfile):
                break
            if not notified_waiting:
                yield toRecord({"type": "waitingForLogfile"})
                notified_waiting = True
            await asyncio.sleep(polling_interval)
        # Logfile has appeared.

        last_yield_was_hidden = False
        with open(logfile) as f:
            while True:
                got = f.readline()
                if got:
                    payload = json.loads(got)["payload"]
                    type = payload["type"]
                    if type == "line":
                        if payload.get("sensitive") and not include_sensitive:
                            if not last_yield_was_hidden:
                                yield toRecord(
                                    {
                                        "type": "line",
                                        "line": "<sensitive information hidden>",
                                        "sensitive": True,
                                    }
                                )
                            last_yield_was_hidden = True
                        else:
                            yield toRecord({"type": "line", "line": payload["line"]})
                            last_yield_was_hidden = False
                    continue
                # EOF

                with _get_db_session() as session:
                    job = await get_job_by_id(job_id, session)
                    if job.finished_at:
                        yield toRecord({"type": "EOF"})
                        return

                await asyncio.sleep(polling_interval)

    return StreamingResponse(log_streamer())


@app.post("/api/jobs/{job_id}/signal/{type}")
async def signal_job(
    type: str,
    job: Annotated[Job, Depends(get_job_by_id)],
    user: Annotated[SessionUser, Depends(get_current_user)],
    session: Session = Depends(get_db_session),
):
    job.signal(session, user.name, type)
    return {
        "message": "Interrupted",
        "job_id": job.id,
    }


# This function is intentionally not marked "async" due to the blocking
# io that happens when querying Gerrit.
@app.get("/api/searchPatch")
def searchPatch(
    gerritsession: Annotated[gerrit.GerritSession, Depends(get_gerrit_session)],
    q: str,
    n: int,
):
    try:
        return gerritsession.changes().query(q, n)
    except Exception:
        return []


#####################
# AUTH RELATED CODE #
#####################
# Also see require_user in the Middleware section near the end of this file


# This function is intentionally not marked "async" due to the blocking
# io that happens in `verify_ticket()`.
@app.get("/api/login")
def login(
    request: Request,
    next: Optional[str] = None,
    ticket: Optional[str] = None,
):
    """
    A CAS server will redirect the browser to this route when the user
    has completed login into the auth server. The redirect will include a
    single-use ticket that we validate here.

    The reference to this route comes from guess_login_url.
    """
    if not ticket:
        raise NotAuthenticatedException()

    # The call to verify_ticket() will take the ticket and make a request
    # to the auth server to validate it.  If validated, the username and
    # attributes are returned.
    username, attributes, _ = cas_client(request, next).verify_ticket(ticket)

    if not username:
        raise NotAuthenticatedException()

    uid = attributes.get("uid")
    if not uid:
        raise NotAuthenticatedException()

    # From here on, the username is the shell username (uid)
    username = uid
    groups = attributes.get("memberOf")
    if groups is None:
        groups = []
    if isinstance(groups, str):
        groups = [groups]

    user = SessionUser(name=username, groups=groups)
    # Store the logged-in-user information in a signed cookie
    # (prepared by SessionMiddleware, which is set up near the bottom of
    # this file).
    request.session["user"] = user.model_dump()
    request.session["epoch"] = get_current_epoch()
    return RedirectResponse(next or "/")


@app.post(ROUTE_2FA)
async def login2(
    otp: Annotated[str, Body()],
    request: Request,
    user: Annotated[Optional[SessionUser], Depends(maybe_get_current_user)],
    session: Session = Depends(get_db_session),
):
    """
    This route is for performing 2FA.
    """
    if not user:
        raise NotAuthenticatedException()

    if user.fully_authenticated:
        return {"code": "ok", "message": "2FA already completed", "user": user.name}

    dbuser = get_or_init_dbuser(session, user.name)
    if not validate_otp(dbuser, otp):
        return JSONResponse(
            status_code=400,
            content={"code": "invalid", "message": "Invalid OTP", "user": user.name},
        )
    # Success
    dbuser.update_last_2fa_code(session, otp)
    user.fully_authenticated = True
    request.session["user"] = user.model_dump()
    return {"code": "ok", "message": "2FA completed", "user": user.name}


def getSSOLogoutUrl(request: Request) -> str:
    cli = cas_client(request, request.headers.get("Referer"))

    return cli.get_logout_url(cli.service_url)


@app.post("/api/logout")
async def logout(
    request: Request,
    user: Annotated[Optional[SessionUser], Depends(maybe_get_current_user)],
):
    SSOLogoutUrl = getSSOLogoutUrl(request)

    if not user:
        return {
            "message": "Already logged out",
            "SSOLogoutUrl": SSOLogoutUrl,
        }
    # Discard knowledge about the current user.
    request.session["user"] = None

    return {
        "message": "Logged out",
        "SSOLogoutUrl": SSOLogoutUrl,
    }


@app.post("/api/logoutAll")
async def logoutAll(
    request: Request, user: Annotated[SessionUser, Depends(get_admin_user)]
):
    # Mass logout is achieved by terminating the current session epoch.
    # The epoch information will be freshly created the next time it is
    # accessed.
    reset_current_epoch()

    SSOLogoutUrl = getSSOLogoutUrl(request)

    return {
        "message": "All login sessions have been invalidated",
        "SSOLogoutUrl": SSOLogoutUrl,
    }


@app.get("/api/whoami")
async def whoami(
    request: Request,
    user: Annotated[Optional[SessionUser], Depends(maybe_get_current_user)],
):
    if user:
        return {
            "user": user.name,
            "fully_authenticated": user.fully_authenticated,
            "groups": user.groups,
            "isAuthorized": user_is_authorized(user.groups),
            "isAdmin": user.isAdmin,
        }

    return {
        "user": None,
        "loginUrl": cas_client(request).get_login_url(),
    }


################
# Web UI stuff #
################

web_dir = os.path.join(os.path.dirname(__file__), "../../web")
dist_dir = os.path.join(web_dir, "dist")
assets_dir = os.path.join(dist_dir, "assets")
index_html = os.path.join(dist_dir, "index.html")

# This conditional is here to allow this module to be imported
# during tests.
if os.path.isdir(assets_dir):
    app.mount("/assets", StaticFiles(directory=assets_dir))


# The following routes correspond to Vue routes established in router.js
@app.get("/")
@app.get("/admin")
@app.get("/login")
@app.get("/logout")
@app.get("/jobs/{job_id}")
@app.get("/notauthorized")
async def index_page():
    return FileResponse(index_html)


##############
# Middleware #
##############

# When processing an HTTP request, middlewares are processed in the reverse
# order of their definitions. xref: https://github.com/fastapi/fastapi/issues/4746
# Therefore all middleware stuff is together in this section of the file to avoid
# confusion


# This middleware is processed last
@app.middleware("http")
async def require_user(request: Request, call_next):
    # Routes outside of the /api/ space are presumed to be user interface
    # routes are therefore not protected.
    if not request.url.path.startswith("/api/") or request.url.path in (
        # These routes do not _require_ a current user.
        "/api/login",
        "/api/logout",
        "/api/whoami",
    ):
        return await call_next(request)

    # Beyond here, at least a partially authenticated user is required.

    user = maybe_get_current_user(request)
    if not user:
        return await authn_exception_handler(request, NotAuthenticatedException())

    if request.url.path == ROUTE_2FA:
        return await call_next(request)

    # All remainining api routes require a fully authenticated user.
    if not user.fully_authenticated:
        return await authn_exception_handler(
            request, NotAuthenticatedException(need2fa=True, user=user)
        )

    # Beyond here we have a fully authenticated user.
    # Though fully authenticated, the user may not be authorized to use SpiderPig.
    # We validate that here.
    if not user_is_authorized(user.groups):
        return JSONResponse(
            status_code=403,
            content={
                "code": "unauthorized",
                "message": "You are not authorized to perform this action",
            },
        )

    return await call_next(request)


# SessionMiddleware must be processed before require_user() (which uses `request.session`)
# is defined, therefore it must be added after require_user() is defined, since middlewares are
# processed in reverse order of their definitions.
if os.getenv("SPIDERPIG_SESSION_KEY_FILE"):
    app.add_middleware(SessionMiddleware, secret_key=get_session_key())


@app.middleware("http")
async def add_cache_control_header(request: Request, call_next):
    response = await call_next(request)
    # Ensure that responses to API requests are not cached by proxies.
    if request.url.path.startswith("/api/"):
        response.headers["Cache-Control"] = "no-cache"
    return response


# CORSMiddleware must be processed before require_user(), therefore it must be
# added after require_user() is defined, since middlewares are processed in reverse order
# of their definitions.
if os.getenv("SPIDERPIG_OPEN_CORS"):
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173"],
        allow_headers=["authorization"],
        allow_methods=["GET", "POST"],
        allow_credentials=True,
    )
