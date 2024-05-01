import json
import os
import pytest

from scap import cli, git


def test_scap_clean(tmpdir):
    wikiversions_file = os.path.join(tmpdir, "wikiversions.json")

    with open(wikiversions_file, "w") as f:
        json.dump(
            {
                "mywiki": "php-1.42.0-wmf.25",
                "yourwiki": "php-1.42.0-wmf.22",
                "hiswiki": "php-1.42.0-wmf.25",
                "herwiki": "php-1.42.0-wmf.22",
            },
            f,
        )

    for php in [
        "php-1.42.0-wmf.21",
        "php-1.42.0-wmf.22",
        "php-1.42.0-wmf.23",
        "php-1.42.0-wmf.25",
    ]:
        dir = os.path.join(tmpdir, php)
        git.init(dir)
        readme = os.path.join(dir, "README")
        with open(readme, "w") as f:
            f.write("Hello world!\n")
        git.add_all(dir, message="testing")

    clean = cli.Application.factory(["clean", "auto", f"-Dstage_dir:{tmpdir}"])
    clean.setup(use_global_config=False)

    assert clean._autoselect_versions_to_remove() == ["1.42.0-wmf.21"]

    # Ensure that we're not allowed to clean an active version
    for version in ["1.42.0-wmf.22", "1.42.0-wmf.25"]:
        with pytest.raises(SystemExit):
            clean.cleanup_branch(version)
