"""
    scap.gerrit
    ~~~~~~~~~~~~~~~~~~~
    Provides Gerrit REST api support for commands
"""

from json import JSONEncoder
from requests import Session
from requests.utils import get_netrc_auth
from string import Template
from pygments import highlight
from pygments.lexers import JsonLexer
from pygments.formatters import TerminalFormatter

import re
import requests
import json
import logging

from urllib.parse import quote
import urllib3


def debug_log(msg, *args):
    logger = logging.getLogger()
    logger.debug(msg, *args)


def error_log(msg, *args):
    logger = logging.getLogger()
    logger.error(msg, *args)


class urlencode_map(object):
    def __init__(self, *base_maps):
        """wrap one or more mappings and urlencode the values"""
        self.base_maps = base_maps

    def __getitem__(self, key):
        for map in self.base_maps:
            if key in map:
                return quote(map[key], safe="")

        raise KeyError('Key "%s" not found' % key)


class GerritSession(object):
    """Opens and tracks a session with a Gerrit instance and provides methods
    for making API calls."""

    def __init__(self, url="https://gerrit.wikimedia.org/r/", use_auth=False):
        self.url = url
        self.useAuth = use_auth

        self.session = Session()

        retries = urllib3.Retry(backoff_factor=1)
        adapter = requests.adapters.HTTPAdapter(max_retries=retries)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)

        if self.useAuth:
            if url[-1] == "/":
                self.api_url = url + "a"
            else:
                self.api_url = url + "/a"

            # get credentials from .netrc if one exists
            self.session.auth = get_netrc_auth(self.api_url)
        else:
            self.api_url = url.rstrip("/")

    def endpoint(self, path="/"):
        return GerritEndpoint(session=self, path=path)

    def changes(self):
        return Changes(session=self)

    def change(self, changeid, **kwargs):
        return Change(changeid, session=self, **kwargs)

    def change_in(self, changeid, **kwargs):
        return ChangeIn(changeid, session=self, **kwargs)

    def change_revision_commit(self, project_branch_id, **kwargs):
        return ChangeRevisionCommit(project_branch_id, session=self, **kwargs)

    def submitted_together(self, changeid, **kwargs):
        return SubmittedTogether(changeid, session=self, **kwargs)

    def depends_ons(self, project_branch_changeid, **kwargs):
        return DependsOns(project_branch_changeid, session=self, **kwargs)

    def change_detail(self, changeid, revisionid="current", **kwargs):
        return ChangeDetail(changeid, revisionid, session=self, **kwargs)

    def submodule_project_from_url(self, submodule_url):
        split_url = submodule_url.split(self.url)
        if len(split_url) > 1:
            return split_url[-1].lstrip("/").replace(".git", "")
        return None

    def change_number_from_url(self, url: str) -> str:
        """Parses and returns the change number from the given URL.
        It also accepts a plain change number (as a string).
        """

        if re.match("[0-9]+$", url):
            return url

        # verify URL is pointing at this Gerrit instance
        if not url.startswith(self.url):
            return None

        # match either:
        # /r/{change-number}
        # /r/c/{change-number}
        # {project-name}/+/{change-number}
        # {project-name}/+/{change-number}/{revision}
        # (all of which are allowed to have a trailing slash)
        match = re.search(r"/r/(?:c/)?([0-9]+)/?$", url) or re.search(
            r"/\+/([0-9]+)(?:/[0-9]+)?/?$", url
        )

        if match is None:
            return None

        return match.group(1)

    def change_revisions(self, changeid, **kwargs):
        return ChangeRevisions(changeid, session=self, **kwargs)

    def change_files(self, changeid, **kwargs):
        return ChangeRevisionsFiles(changeid, session=self, **kwargs)

    def project_branch(self, project, branch, **kwargs):
        return ProjectBranch(project, branch, session=self, **kwargs)

    def project_branches(self, project, **kwargs):
        return ProjectBranches(project, session=self, **kwargs)

    def get(self, *args, **kwargs):
        return self.session.get(*args, **kwargs)

    def post(self, *args, **kwargs):
        return self.session.post(*args, **kwargs)


class GerritEndpoint(object):
    """base class for gerrit api endpoints"""

    # derived classes override the path section of the uri for each endpoint
    _path = "/"
    _uri_template = None
    _session = None

    def __init__(self, session=None, path="/"):
        """
        Create a generic endpoint instance with the specified path.
        The path template should be a string with placeholder ${variables}
        for the dynamic parts of the path component of the api url.
        """
        if session is None:
            session = GerritSession()

        self._session = session
        self._path = path

    def _url(self, **kwargs):
        """Builds the url for http requests to this endpoint.
        This is done by combining api_url with self._path and then replacing
        variable placeholders in the url with values from self.__dict__

        Variables can be overridden by calling this method with arbitrary
        keyword arguments which will take precedence over values from __dict__
        """
        if self._uri_template is None:
            self._uri_template = Template("/".join((self._session.api_url, self._path)))

        return self._uri_template.safe_substitute(urlencode_map(self.__dict__, kwargs))

    def get(self, **kwargs) -> "AttrDict":
        """Call the api with a http get request"""
        params = kwargs.pop("params", None)
        uri = self._url(**kwargs)

        debug_log("uri: %s", uri)
        debug_log("_path: %s", self._path)
        res = self._session.get(uri, params=params, timeout=30)
        return self.load(res)

    def load(self, res) -> "AttrDict":
        res.raise_for_status()
        if res.status_code == 200:
            try:
                # gerrit prepends junk to the response, strip it:
                data = res.text.lstrip(")]}'")
                json_str = "".join(data[1:])
                self.data = json.loads(json_str, object_hook=AttrDict)
                return self.data
            except Exception as e:
                error_log("Could not decode response: %s", res.text)
                print(json_str)
                raise e
        else:
            raise Exception(f"Unexpected HTTP response code: {res.status_code}")

    def post(self, data={}, **kwargs):
        """make a http POST request to this api endpoint"""
        uri = self._url()
        debug_log("POST to: %s", uri)
        debug_log("POST Data: %s", data)
        debug_dump_json(data)
        res = self._session.post(uri, json=data, timeout=30)
        debug_log("Response: %s", res.text)
        return self.load(res)

    def put(self, data={}, **kwargs):
        """make a http PUT request to this api endpoint"""
        uri = self._url()
        debug_log("PUT to: %s", uri)
        debug_log("PUT Data: %s", data)
        debug_dump_json(data)
        res = self._session.put(uri, json=data, timeout=30)
        debug_log("Response: %s", res.text)
        return self.load(res)

    def __call__(self, path, **kwargs):
        """
        Clone this endpoint instance, append the path and return the modified
        instance.
        This allows us to chain method calls instead of creating a subclass
        for every gerrit api method.

        Example:
        change = ChangeDetail(changeid)
        actions = change('actions').get()
        """
        _path = self._path
        _path = "/".join((_path, path))
        new_instance = GerritEndpoint(path=_path, session=self._session)
        for k in self.__dict__:
            if k[0] == "_":
                continue
            new_instance.__dict__[k] = self.__dict__[k]

        new_instance._parent = self
        return new_instance

    def __repr__(self):
        """return a string representation of this object's data"""
        return gerrit_encoder(indent=2).encode(self.data)


class Changes(GerritEndpoint):
    """Query Gerrit changes"""

    def __init__(self, **kwargs):
        super().__init__(path="changes/", **kwargs)

    def query(self, q="status:open", n=10):
        return self.get(params={"q": q, "n": n})


class Change(GerritEndpoint):
    """get details for a gerrit change"""

    changeid = None
    revisionid = "current"

    def __init__(self, changeid, revisionid="current", **kwargs):
        super().__init__(path="changes/%s" % changeid, **kwargs)
        self.changeid = changeid
        self.revision = ChangeRevisions(
            changeid, revisionid=revisionid, session=self._session
        )


class ChangeIn(GerritEndpoint):
    """Get the branches and tags the change is included in"""

    changeid = None
    revisionid = "current"

    def __init__(self, changeid, revisionid="current", **kwargs):
        super().__init__(path="changes/%s/in" % changeid, **kwargs)
        self.changeid = changeid
        self.revision = ChangeRevisions(
            changeid, revisionid=revisionid, session=self._session
        )


class ChangeRevisionCommit(GerritEndpoint):
    """Get the commit hash of a change"""

    project_branch_id = None
    revisionid = "current"

    def __init__(self, project_branch_id, revisionid="current", **kwargs):
        super().__init__(
            path="changes/%s/revisions/%s/commit" % (project_branch_id, revisionid),
            **kwargs,
        )
        self.project_branch_id = project_branch_id
        self.revision = ChangeRevisions(
            project_branch_id, revisionid=revisionid, session=self._session
        )


class SubmittedTogether(GerritEndpoint):
    """get the submitted together changes for a gerrit change"""

    changeid = None
    revisionid = "current"

    def __init__(self, changeid, revisionid="current", **kwargs):
        super().__init__(
            path="changes/%s/submitted_together?o=NON_VISIBLE_CHANGES" % changeid,
            **kwargs,
        )
        self.changeid = changeid
        self.revision = ChangeRevisions(
            changeid, revisionid=revisionid, session=self._session
        )


class DependsOns(GerritEndpoint):
    """get the Depends-On changes for a gerrit change"""

    changeid = None
    revisionid = "current"

    def __init__(self, project_branch_changeid, revisionid="current", **kwargs):
        super().__init__(
            path="changes/%s/revisions/current/crd" % project_branch_changeid, **kwargs
        )
        self.changeid = project_branch_changeid
        self.revision = ChangeRevisions(
            project_branch_changeid, revisionid=revisionid, session=self._session
        )


class ChangeDetail(GerritEndpoint):
    """get details for a gerrit change"""

    changeid = None
    revisionid = "current"

    def __init__(self, changeid, revisionid="current", **kwargs):
        options = "?o=COMMIT_FOOTERS&o=CURRENT_COMMIT"
        if revisionid == "current":
            options += "&o=CURRENT_REVISION"
        elif revisionid == "all":
            options += "&o=ALL_REVISIONS"
            self.revisionid = "current"
        super().__init__(path="changes/%s/detail%s" % (changeid, options), **kwargs)
        self.changeid = changeid
        self.revision = ChangeRevisions(changeid, revisionid, session=self._session)


class ChangeRevisions(GerritEndpoint):
    revisionid = "current"

    def __init__(self, changeid, revisionid="current", **kwargs):
        super().__init__(
            path="changes/%s/revisions/%s" % (changeid, revisionid), **kwargs
        )
        self.changeid = changeid
        self.revisionid = revisionid


class ChangeRevisionsFiles(GerritEndpoint):
    revisionid = "current"

    def __init__(self, changeid, revisionid="current", **kwargs):
        super().__init__(
            path="changes/%s/revisions/%s/files" % (changeid, revisionid), **kwargs
        )
        self.changeid = changeid
        self.revisionid = revisionid


class ProjectBranch(GerritEndpoint):
    def __init__(self, project, branch, **kwargs):
        super().__init__(path="projects/%s/branches/%s" % (project, branch), **kwargs)
        self.project = project
        self.branch = branch


class ProjectBranches(GerritEndpoint):
    project = None

    def __init__(self, project, **kwargs):
        super().__init__(path="projects/%s/branches" % (project), **kwargs)
        self.project = project

    def create(self, branch, revision):
        newbranch = ProjectBranch(self.project, branch, session=self._session)
        data = {"revision": revision}
        return newbranch.put(data=data)


def debug_dump_json(data):
    """dump an object to the debug logger as pretty-printed json"""
    logger = logging.getLogger()
    if logger.isEnabledFor(logging.DEBUG):
        try:
            json_str = gerrit_encoder(indent=2).encode(data)
            output = highlight(json_str, JsonLexer(), TerminalFormatter())
            logger.debug(output)
        except Exception as e:
            logger.debug(e)
            logger.debug(data)


class gerrit_encoder(JSONEncoder):
    """encode python objects to json"""

    def default(self, o):
        if hasattr(o, "__dump__"):
            return o.__dump__()
        if hasattr(o, "data"):
            return o.data
        if hasattr(o, "__dict__"):
            return o.__dict__
        return JSONEncoder.default(self, o)


class AttrDict(dict):
    """A class for accessing dict keys as attributes.
    The gerrit api returns json object trees which are decoded into python
    dictionary objects, then wrapped in AttrDict to allow easy access to
    nested attributes within the data structure.

    For example, this allows the following:
        change.data.labels.Verified
    Instead of:
        change.data['labels']['Verified']
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__dict__ = self

    def __getattr__(self, key):
        if key in self:
            return self[key]
        # avoid key errors
        return None
