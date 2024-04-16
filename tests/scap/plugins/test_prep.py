import os.path
from unittest.mock import patch

from scap.plugins.prep import CheckoutMediaWiki
import scap.cli


@patch.object(CheckoutMediaWiki, "new_history")
@patch("scap.plugins.prep.git")
def test_scap_prep_sets_a_push_url(git, _):
    scap_prep = scap.cli.Application.factory(["prep", "1.99.0-wmf.33"])
    scap_prep.setup(use_global_config=False)

    # config has been loaded
    assert "gerrit_push_url" in scap_prep.config
    assert scap_prep.config["gerrit_push_url"] == "ssh://gerrit.wikimedia.org:29418/"

    # change config to ensure it is taken in account
    test_push_url = "ssh://review.example.org"
    scap_prep.config["gerrit_push_url"] = test_push_url

    repo_name = "some/repository"
    dest_dir = "/dest"

    scap_prep._clone_or_update_repo(
        os.path.join(scap_prep.config["gerrit_url"], repo_name),
        "wmf/1.99.0.wmf-33",
        dest_dir,
        None,
    )
    git.gitcmd.assert_any_call(
        "remote",
        "set-url",
        "--push",
        "origin",
        os.path.join(test_push_url, repo_name),
        cwd=dest_dir,
    )
