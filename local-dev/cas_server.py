from typing import Annotated, Optional
from fastapi import FastAPI, Form, Request
from fastapi.responses import HTMLResponse, RedirectResponse
import json
import os
import time
import urllib.parse

from starlette.middleware.sessions import SessionMiddleware


# This is a fake CAS server which handles authentication for a few
# hard-coded test accounts used during SpiderPig development.

USER_DB = {
    "deployer01": {
        "username": "deployer01",
        "password": "scap100",
        "attributes": {
            "uid": "deployer01",
            "memberOf": [
                "cn=deployers,ou=groups,dc=example",
                "cn=admins,ou=groups,dc=example",
            ],
        },
    },
    "deployer02": {
        "username": "deployer02",
        "password": "scap102",
        "attributes": {
            "uid": "deployer02",
            "memberOf": [
                "cn=deployers,ou=groups,dc=example",
            ],
        },
    },
}

SERVICE_DB = {"http://localhost:8000/": {"name": "SpiderPig local development"}}


def get_service_info(service: str) -> Optional[dict]:
    for url_prefix, info in SERVICE_DB.items():
        if service.startswith(url_prefix):
            return info

    return None


TICKETS_FILENAME = "/workspace/cas-tickets"
TICKET_TTL = 60  # Seconds


def write_ticket_db(db: dict):
    tmpname = f"{TICKETS_FILENAME}.tmp"

    with open(tmpname, "w") as f:
        json.dump(db, f)

    os.rename(tmpname, TICKETS_FILENAME)


def read_ticket_db() -> dict:
    db = {}

    if os.path.exists(TICKETS_FILENAME):
        with open(TICKETS_FILENAME) as f:
            db = json.load(f)

    # Discard any expired tickets
    now = time.time()
    for ticket, info in db.items():
        if now >= info["expires_at"]:
            del db[ticket]

    return db


def generate_cas_ticket(username) -> str:
    ticket = os.urandom(32).hex()

    db = read_ticket_db()

    now = time.time()
    db[ticket] = {
        "username": username,
        "issued_at": now,
        "expires_at": now + TICKET_TTL,
    }

    write_ticket_db(db)

    return ticket


def validate_ticket(ticket: str):
    db = read_ticket_db()
    info = db.get(ticket)
    if not info:
        return None, None

    username = info["username"]
    attributes = USER_DB[username]["attributes"]

    # Remove the ticket
    del db[ticket]
    write_ticket_db(db)

    return username, attributes


def cas_successful_auth_response(service: Optional[str], username: str):
    if service:
        parsed = urllib.parse.urlsplit(service)
        d = urllib.parse.parse_qs(parsed.query)
        d["ticket"] = generate_cas_ticket(username)
        parsed = parsed._replace(query=urllib.parse.urlencode(d, doseq=True))
        # a 302 redirect is used to ensure that the client
        # uses a GET request on the request URL even though they just
        # POSTed to /cas/login.
        return RedirectResponse(parsed.geturl(), status_code=302)

    return HTMLResponse("Single sign-on completed.")


def render_cas_login_form(service=None, complaint=None):
    resp = ""

    if service:
        info = get_service_info(service)
        name = info["name"]
        resp += f"{name} login"
    else:
        resp += "General SSO requested"
    resp += "<br>"

    if complaint:
        resp += f'<p style="color: red;">{complaint}</p>'

    resp += """
<form method="POST">
Username: <input name="username"/><br>
Password: <input name="password" type="password"/><br>
<input type="submit"/>
</form>

"""

    return HTMLResponse(resp)


app = FastAPI()


@app.get("/cas/login")
async def cas_login(request: Request, service: Optional[str] = None):
    if service and not get_service_info(service):
        return HTMLResponse(
            f"Unwilling to handle authentication for service {service}", status_code=403
        )

    username = request.session.get("SSO")
    if username:
        # The user has already completed SSO.
        return cas_successful_auth_response(service, username)

    return render_cas_login_form(service=service)


@app.post("/cas/login")
async def cas_login_post(
    request: Request,
    username: Annotated[str, Form()],
    password: Annotated[str, Form()],
    service: Optional[str] = None,
):
    if service and not get_service_info(service):
        return HTMLResponse(
            f"Unwilling to handle authentication for service {service}", status_code=403
        )

    user_info = USER_DB.get(username)
    if not user_info or password != user_info["password"]:
        return render_cas_login_form(
            complaint="Invalid username/password", service=service
        )

    # Successful authentication
    request.session["SSO"] = username

    return cas_successful_auth_response(service, username)


@app.get("/cas/logout")
async def cas_logout(request: Request, service: Optional[str] = None):
    request.session["SSO"] = None

    login_url = "/cas/login"
    if service:
        login_url += "?" + urllib.parse.urlencode({"service": service})

    return HTMLResponse(
        f'You have successfully logged out of SSO.  Click <a href="{login_url}">here</a> to sign back in'
    )


@app.get("/cas/p3/serviceValidate")
async def cas_validate(request: Request, ticket: str, service: str):
    if not get_service_info(service):
        return HTMLResponse(
            f"Unwilling to handle authentication for service {service}", status_code=403
        )

    username, attributes = validate_ticket(ticket)
    if not username:
        return HTMLResponse(
            f"""<cas:serviceResponse xmlns:cas="http://www.yale.edu/tp/cas">
 <cas:authenticationFailure code="INVALID_TICKET">
    Ticket {ticket} not recognized
  </cas:authenticationFailure>
</cas:serviceResponse>"""
        )

    resp = f"""<cas:serviceResponse xmlns:cas="http://www.yale.edu/tp/cas">
 <cas:authenticationSuccess>
  <cas:user>{username}</cas:user>
  <cas:attributes>
"""

    for key, values in attributes.items():
        if not isinstance(values, list):
            values = [values]
        for value in values:
            resp += f"<cas:{key}>{value}</cas:{key}>\n"

    resp += """
  </cas:attributes>
 </cas:authenticationSuccess>
</cas:serviceResponse>
"""

    return HTMLResponse(resp)


def get_session_key():
    filename = "/workspace/cas-session-key"

    if os.path.exists(filename):
        with open(filename) as f:
            return f.read()

    key = os.urandom(32).hex()

    with open(filename, "w") as f:
        f.write(key)

    return key


app.add_middleware(SessionMiddleware, secret_key=get_session_key())
