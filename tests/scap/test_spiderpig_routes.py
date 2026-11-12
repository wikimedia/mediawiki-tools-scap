# Verify that the page routes of the apiserver match the routes of the Vue app.

import re
from pathlib import Path

import pytest

import scap.spiderpig.api as api

ROUTER_JS = Path(__file__).parents[2] / "web" / "src" / "router.js"

# Page routes that the apiserver serves although router.js declares no route
# for them.
SERVER_ONLY_PATHS = {"/logout"}

PARAMETER_RE = re.compile(r":[A-Za-z_][A-Za-z0-9_]*|\{[A-Za-z_][A-Za-z0-9_]*\}")

needs_web_sources = pytest.mark.skipif(
    not ROUTER_JS.exists(), reason="the web sources are not present"
)


def normalize(path: str) -> str:
    """The path with each route parameter replaced by a placeholder.

    router.js writes a parameter as ":jobId" where FastAPI writes "{job_id}",
    so the comparison is of the shape of the path, not of parameter names.
    """
    return PARAMETER_RE.sub("{}", path)


def vue_paths() -> set:
    paths = {
        normalize(path)
        for path in re.findall(r"path:\s*'([^']*)'", ROUTER_JS.read_text())
    }
    assert paths, f"found no route paths in {ROUTER_JS}"
    return paths


def served_paths() -> set:
    return {
        normalize(route.path)
        for route in api.app.routes
        if getattr(route, "endpoint", None) is api.index_page
    }


@needs_web_sources
def test_index_page_serves_every_vue_route():
    missing = vue_paths() - served_paths()
    assert not missing, (
        "index_page() in scap/spiderpig/api.py does not serve these routes of"
        f" web/src/router.js, so a reload of one answers 404: {sorted(missing)}"
    )


@needs_web_sources
def test_index_page_serves_no_route_that_the_vue_app_lacks():
    extra = served_paths() - vue_paths() - {normalize(p) for p in SERVER_ONLY_PATHS}
    assert not extra, (
        "index_page() in scap/spiderpig/api.py serves these routes, which"
        f" web/src/router.js does not declare: {sorted(extra)}"
    )
