import json
import os
import pytest

from scap import cli, git
from scap.clean import Clean
from scap.runcmd import gitcmd, touch


def test_scap_clean(tmpdir):
    wikiversions_file = os.path.join(tmpdir, "wikiversions.json")

    with open(wikiversions_file, "w") as f:
        json.dump(
            {
                "mywiki": "php-1.42.0-wmf.25",
                "yourwiki": "php-1.42.0-wmf.23",
                "hiswiki": "php-1.42.0-wmf.25",
                "herwiki": "php-1.42.0-wmf.23",
            },
            f,
        )

    for php in [
        "php-1.42.0-wmf.21",
        "php-1.42.0-wmf.22",
        "php-1.42.0-wmf.23",
        "php-1.42.0-wmf.25",
        "php-1.42.0-wmf.26",
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
    for version in ["1.42.0-wmf.23", "1.42.0-wmf.25"]:
        with pytest.raises(SystemExit):
            clean.cleanup_branch(version)


def test__get_submodules_path(tmpdir):
    def make_repo(name, submodules=()):
        repo = os.path.join(tmpdir, name)
        git.init(repo)
        touch("README", cwd=repo)
        for path, submodule in submodules:
            # git refuses a submodule on a local path without this
            gitcmd(
                "-c",
                "protocol.file.allow=always",
                "submodule",
                "add",
                submodule,
                path,
                cwd=repo,
            )
        git.add_all(repo, "testing")
        return repo

    ve = make_repo("VisualEditor")
    core = make_repo(
        "core",
        [
            ("extensions/AbuseFilter", make_repo("AbuseFilter")),
            (
                "extensions/VisualEditor",
                make_repo("extension-VisualEditor", [("lib/ve", ve)]),
            ),
            ("vendor", make_repo("vendor")),
        ],
    )
    gitcmd(
        "-c",
        "protocol.file.allow=always",
        "submodule",
        "update",
        "--init",
        "--recursive",
        cwd=core,
    )

    assert [
        os.path.join(core, "extensions/AbuseFilter"),
        os.path.join(core, "extensions/VisualEditor"),
        os.path.join(core, "extensions/VisualEditor/lib/ve"),
        os.path.join(core, "vendor"),
    ] == Clean._get_submodules_paths(core)
