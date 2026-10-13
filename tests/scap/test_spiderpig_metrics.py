# Verify the HTTP metrics of the apiserver.

from fastapi.testclient import TestClient

import scap.spiderpig.api as api
from scap.spiderpig.session import SessionCookie


def test_metrics_measure_requests(monkeypatch):
    monkeypatch.setattr(
        api.app.state, "session_cookie", SessionCookie("0" * 64), raising=False
    )
    client = TestClient(api.app)

    assert client.get("/doesnotexist/123").status_code == 404
    body = client.get("/metrics").text

    assert 'http_requests_total{handler="none",method="GET",status="4xx"}' in body
    assert "/doesnotexist" not in body
    assert 'handler="/metrics"' not in body
