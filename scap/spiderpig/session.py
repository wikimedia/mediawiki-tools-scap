import base64
import json
import logging
from typing import Optional
from fastapi import Response
import itsdangerous


DEFAULT_SESSION_COOKIE_MAX_AGE = 14 * 24 * 60 * 60  # 14 days, in seconds


class SessionCookie:
    def __init__(
        self, key, max_age=DEFAULT_SESSION_COOKIE_MAX_AGE, cookie_name="session"
    ):
        self.max_age = max_age
        self.cookie_name = cookie_name
        self.signer = itsdangerous.TimestampSigner(key)

    def decode(self, cookie: Optional[str]) -> Optional[dict]:
        """
        Decode the session cookie and return the data as a dictionary.
        Returns None if the cookie is invalid or does not exist.
        """
        if not cookie:
            return None

        try:
            data = self.signer.unsign(cookie, max_age=self.max_age)
            data = json.loads(base64.b64decode(data))
            return data
        except Exception as e:
            logging.warning(f"Invalid session cookie '{cookie}': {e}")
            return None

    def encode(self, data: dict) -> str:
        """
        Encode the session data into a string suitable for storing in a cookie.
        """
        data = base64.b64encode(json.dumps(data).encode("utf-8"))
        data = self.signer.sign(data)
        return data.decode("utf-8")

    def set_session_cookie_on_response(self, response: Response, data: Optional[dict]):
        if data is None:
            response.delete_cookie("session")
            return

        response.set_cookie(
            self.cookie_name,
            self.encode(data),
            httponly=True,
            secure=True,
            max_age=self.max_age,
        )
