# Test spiderpig apiserver auth-related functions

import json
import re
from fastapi.datastructures import State
import pyotp
import os
import sys

from scap.spiderpig.api import (
    get_pyotp,
    maybe_get_current_user,
    get_current_user,
    NotAuthenticatedException,
    require_user,
    login,
    login2,
    logout,
    logoutAll,
    SessionUser,
    get_current_epoch,
    get_admin_user,
    get_session_key,
    ROUTE_2FA,
)
from scap.spiderpig.model import User
from scap.spiderpig.session import SessionCookie
import scap.utils

from fastapi import HTTPException
from fastapi.responses import JSONResponse, RedirectResponse

import pytest
import unittest.mock
from unittest.mock import Mock

if sys.version_info >= (3, 8):
    from unittest.mock import AsyncMock


@pytest.fixture(autouse=True)
def spiderpigdir(tmpdir):
    with unittest.mock.patch.dict(
        os.environ,
        {
            "SPIDERPIG_DIR": str(tmpdir),
            "SPIDERPIG_SESSION_KEY_FILE": f"{tmpdir}/spiderpig_session_key",
        },
    ):
        get_current_epoch()
        yield tmpdir


@pytest.fixture(autouse=True)
def mock_app(spiderpigdir):
    with unittest.mock.patch.object(scap.spiderpig.api, "app") as app:
        app.state.session_cookie = SessionCookie(get_session_key())
        yield app


@pytest.fixture(autouse=True)
def scap_config():
    scap_config = {
        "spiderpig_auth_server": "https://cas.example.org/cas",
        "spiderpig_admin_groups": "cn=admins,ou=groups,dc=example,dc=org",
        "spiderpig_user_groups": "cn=deployers,ou=groups,dc=example,dc=org",
    }

    env = {
        "SPIDERPIG_SCAP_CONFIG": scap.utils.string_to_base64_string(
            json.dumps(scap_config)
        )
    }

    with unittest.mock.patch.dict(os.environ, env):
        yield


@pytest.fixture
def bad_epoch_user_session(mock_app):
    return mock_app.state.session_cookie.encode(
        {
            "user": {
                "name": "bruce",
                "groups": ["cn=deployers,ou=groups,dc=example,dc=org"],
                "fully_authenticated": False,
            },
            "epoch": "bogus",
        }
    )


@pytest.fixture
def partially_authenticated_user_session(mock_app):
    return mock_app.state.session_cookie.encode(
        {
            "user": {
                "name": "bruce",
                "groups": ["cn=deployers,ou=groups,dc=example,dc=org"],
                "fully_authenticated": False,
            },
            "epoch": get_current_epoch(),
        }
    )


@pytest.fixture
def fully_authenticated_user_session(mock_app):
    return mock_app.state.session_cookie.encode(
        {
            "user": {
                "name": "bruce",
                "groups": ["cn=deployers,ou=groups,dc=example,dc=org"],
                "fully_authenticated": True,
            },
            "epoch": get_current_epoch(),
        }
    )


@pytest.fixture
def fully_authenticated_rando_session(mock_app):
    return mock_app.state.session_cookie.encode(
        {
            "user": {
                "name": "rando",
                "groups": ["cn=randos,ou=groups,dc=example,dc=org"],
                "fully_authenticated": True,
            },
            "epoch": get_current_epoch(),
        }
    )


@pytest.fixture
def fully_authenticated_admin_user_session(mock_app):
    return mock_app.state.session_cookie.encode(
        {
            "user": {
                "name": "bruce",
                "groups": [
                    "cn=admins,ou=groups,dc=example,dc=org",
                ],
                "fully_authenticated": True,
            },
            "epoch": get_current_epoch(),
        }
    )


def test_maybe_get_current_user(
    bad_epoch_user_session, partially_authenticated_user_session
):
    request = Mock()
    request.cookies = {}
    request.state = State()
    assert maybe_get_current_user(request) is None

    # Verify that user info from a non-current epoch is ignored.
    request.cookies["session"] = bad_epoch_user_session
    request.state = State()
    assert maybe_get_current_user(request) is None

    # Reset to a proper partially authenthenticated user
    request.cookies["session"] = partially_authenticated_user_session

    request.state = State()
    user = maybe_get_current_user(request)
    assert user
    assert user.name == "bruce"
    assert user.groups == ["cn=deployers,ou=groups,dc=example,dc=org"]
    assert user.fully_authenticated is False
    assert user.isAdmin is False


def test_get_current_user(
    partially_authenticated_user_session, fully_authenticated_user_session
):
    request = Mock()
    request.cookies = {}
    request.state = State()
    with pytest.raises(NotAuthenticatedException) as excinfo:
        get_current_user(request)

    request.cookies["session"] = partially_authenticated_user_session

    with pytest.raises(NotAuthenticatedException) as excinfo:
        request.state = State()
        get_current_user(request)
    assert excinfo.value.need2fa is True
    assert excinfo.value.user.name == "bruce"

    request.cookies["session"] = fully_authenticated_user_session
    request.state = State()
    user = get_current_user(request)
    assert user.name == "bruce"
    assert user.groups == ["cn=deployers,ou=groups,dc=example,dc=org"]


def test_get_admin_user(
    mockrequest,
    partially_authenticated_user_session,
    fully_authenticated_user_session,
    fully_authenticated_admin_user_session,
):
    mockrequest.cookies["session"] = partially_authenticated_user_session
    with pytest.raises(NotAuthenticatedException):
        mockrequest.state = State()
        get_admin_user(mockrequest)

    mockrequest.cookies["session"] = fully_authenticated_user_session
    with pytest.raises(HTTPException):
        mockrequest.state = State()
        get_admin_user(mockrequest)

    mockrequest.cookies["session"] = fully_authenticated_admin_user_session
    mockrequest.state = State()
    user = get_admin_user(mockrequest)
    assert user.isAdmin is True


@pytest.fixture
def mockrequest():
    request = Mock()
    request.cookies = {}
    request.state = State()
    request.headers = {
        "Host": "spiderpig-apiserver.example.org",
        "Referer": "https://spiderpig-webserver.example.org/somepage",
    }

    yield request


@pytest.mark.skipif(
    sys.version_info < (3, 8), reason="requires python3.8 or higher for AsyncMock"
)
@pytest.mark.anyio
async def test_require_user(
    mockrequest,
    partially_authenticated_user_session,
    fully_authenticated_user_session,
    fully_authenticated_rando_session,
    fully_authenticated_admin_user_session,
):
    # Test anonymous access to unprotected routes
    for route in ["/non-api-path", "/api/login", "/api/logout", "/api/whoami"]:
        mockrequest.url.path = route
        call_next = AsyncMock()
        await require_user(mockrequest, call_next)
        call_next.assert_called_once_with(mockrequest)

    # Test anonymous access to protected route
    mockrequest.url.path = "/api/anything"
    call_next = AsyncMock()
    res = await require_user(mockrequest, call_next)
    assert isinstance(res, JSONResponse)
    assert res.status_code == 401
    body = json.loads(res.body)
    assert body["code"] == "needauth"
    assert (
        body["url"]
        == "https://cas.example.org/cas/login?service=https%3A%2F%2Fspiderpig-apiserver.example.org%2Fapi%2Flogin%3Fnext%3Dhttps%253A%252F%252Fspiderpig-webserver.example.org%252Fsomepage"
    )
    call_next.assert_not_awaited()

    # Test half-authenticated access to a protected route
    mockrequest.cookies["session"] = partially_authenticated_user_session
    mockrequest.url.path = "/api/anything"
    mockrequest.state = State()
    call_next = AsyncMock()
    res = await require_user(mockrequest, call_next)
    assert isinstance(res, JSONResponse)
    assert res.status_code == 401
    body = json.loads(res.body)
    assert body["code"] == "need2fa"
    assert body["url"] == ROUTE_2FA
    assert body["user"] == "bruce"
    call_next.assert_not_awaited()

    # Test half-authenticated access to 2FA route
    mockrequest.url.path = ROUTE_2FA
    mockrequest.state = State()
    call_next = AsyncMock()
    await require_user(mockrequest, call_next)
    call_next.assert_called_once_with(mockrequest)

    # Test fully authenticated access to a protected route
    mockrequest.url.path = "/api/anything"
    mockrequest.cookies["session"] = fully_authenticated_user_session
    mockrequest.state = State()
    call_next = AsyncMock()
    await require_user(mockrequest, call_next)
    call_next.assert_called_once_with(mockrequest)

    # Test unauthorized user group
    mockrequest.cookies["session"] = fully_authenticated_rando_session
    mockrequest.state = State()
    call_next = AsyncMock()
    res = await require_user(mockrequest, call_next)
    assert isinstance(res, JSONResponse)
    assert res.status_code == 403
    call_next.assert_not_awaited()

    # Test admin user is always authorized
    mockrequest.cookies["session"] = fully_authenticated_admin_user_session
    mockrequest.state = State()
    call_next = AsyncMock()
    await require_user(mockrequest, call_next)
    call_next.assert_called_once_with(mockrequest)


def test_login(mock_app, mockrequest):
    with unittest.mock.patch("cas.CASClientV3.verify_ticket") as verify_ticket:
        # Test no ticket provided.
        with pytest.raises(NotAuthenticatedException):
            login(mockrequest, None, None)

        # Test bad ticket
        verify_ticket.return_value = (None, None, None)
        with pytest.raises(NotAuthenticatedException):
            login(mockrequest, None, "123")

        # Test good ticket, good group
        verify_ticket.return_value = (
            # username
            "Bruce",
            # LDAP attributes
            {
                "uid": "bruce",
                "memberOf": ["cn=deployers,ou=groups,dc=example,dc=org"],
            },
            # proxy-granting ticket IOU
            None,
        )

        res = login(mockrequest, None, "123")
        assert isinstance(res, RedirectResponse)
        assert res.headers["location"] == "/"

        # Test 'next' handling
        res = login(mockrequest, "/somepage", "123")
        assert isinstance(res, RedirectResponse)
        assert res.headers["location"] == "/somepage"

        # Test case where "memberOf" is a single string.
        verify_ticket.return_value = (
            # username
            "Bruce",
            # LDAP attributes
            {
                "uid": "bruce",
                "memberOf": "cn=deployers,ou=groups,dc=example,dc=org",
            },
            # proxy-granting ticket IOU
            None,
        )
        res = login(mockrequest, None, "123")
        assert isinstance(res, RedirectResponse)
        cookie_header = res.headers["set-cookie"]
        m = re.match(r'^session="([^;]+)";', cookie_header)
        assert m
        data = mock_app.state.session_cookie.decode(m[1])
        assert data["user"]["groups"] == ["cn=deployers,ou=groups,dc=example,dc=org"]


@pytest.mark.anyio
async def test_login2(mockrequest):
    dbsession = Mock()
    with pytest.raises(NotAuthenticatedException):
        await login2("XXXXXX", mockrequest, None, dbsession)
    user = SessionUser(
        name="bruce", groups=["cn=deployers,ou=groups,dc=example,dc=org"]
    )
    dbuser = User(name="Bruce", otp_seed=pyotp.random_base32())

    with unittest.mock.patch.object(User, "get") as g:
        g.return_value = dbuser

        # Test bad OTP
        res = await login2("XXXXXX", mockrequest, user, dbsession)
        assert isinstance(res, JSONResponse)
        assert res.status_code == 400
        body = json.loads(res.body)
        assert body["code"] == "invalid"
        assert body["user"] == "bruce"

        # Test good OTP
        otp = get_pyotp(dbuser).now()
        res = await login2(otp, mockrequest, user, dbsession)
        assert (
            res.body
            == JSONResponse(
                {"code": "ok", "message": "2FA completed", "user": "bruce"}
            ).body
        )
        assert user.fully_authenticated is True

        # Test already 2fa'd
        res = await login2(otp, mockrequest, user, dbsession)
        assert res == {
            "code": "ok",
            "message": "2FA already completed",
            "user": "bruce",
        }

        # Test reuse of good OTP
        user.fully_authenticated = False
        res = await login2(otp, mockrequest, user, dbsession)
        assert isinstance(res, JSONResponse)
        assert res.status_code == 400
        body = json.loads(res.body)
        assert body["code"] == "invalid"
        assert body["user"] == "bruce"


@pytest.mark.anyio
async def test_logout(mockrequest, partially_authenticated_user_session):
    SSOLogoutUrl = "https://cas.example.org/cas/logout?service=https%3A%2F%2Fspiderpig-apiserver.example.org%2Fapi%2Flogin%3Fnext%3Dhttps%253A%252F%252Fspiderpig-webserver.example.org%252Fsomepage"

    assert await logout(mockrequest, None) == {
        "message": "Already logged out",
        "SSOLogoutUrl": SSOLogoutUrl,
    }
    res = await logout(mockrequest, Mock())
    assert (
        res.body
        == JSONResponse(
            {
                "message": "Logged out",
                "SSOLogoutUrl": SSOLogoutUrl,
            }
        ).body
    )
    cookie_header = res.headers["set-cookie"]
    assert "Max-Age=0;" in cookie_header


@pytest.mark.anyio
async def test_logoutAll(mockrequest):
    user = SessionUser(
        name="bruce", groups=["cn=deployers,ou=groups,dc=example,dc=org"]
    )
    assert await logoutAll(mockrequest, user) == {
        "message": "All login sessions have been invalidated",
        "SSOLogoutUrl": "https://cas.example.org/cas/logout?service=https%3A%2F%2Fspiderpig-apiserver.example.org%2Fapi%2Flogin%3Fnext%3Dhttps%253A%252F%252Fspiderpig-webserver.example.org%252Fsomepage",
    }
