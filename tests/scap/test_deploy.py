import os
import tempfile
import yaml

import unittest
from unittest.mock import patch, MagicMock, Mock

import scap.cli
import scap.config
import scap.git
import scap.script
import scap.utils
from scap.deploy import Deploy, DeployLocal, get_git_binary_managers
from .test_config import override_default_config

import pytest
from pytest import param as case

# Aliasing for a nicer list of test cases
Check = Mock

valid_chk_testcases = [
    # Parameters:
    # - expected
    # - a Check with group and stage,
    # - current stage
    # - current group
    # - when (before or after stage)
    case(
        True,
        Check(group=None, after="promote"),
        "promote",
        None,
        "after",
        id="When on promote stage, accept promote check",
    ),
    case(
        False,
        Check(group=None, after="promote", when="after"),
        "fetch",
        None,
        "after",
        id="When on fetch stage, reject promote check",
    ),
    case(
        True,
        Check(group="canaries", after="promote"),
        "promote",
        None,
        "after",
        id="When on promote stage, accept promote check for canaries",
    ),
    case(
        False,
        Check(group="canaries", after="promote"),
        "fetch",
        None,
        "after",
        id="When on promote stage, reject fetch check for canaries",
    ),
    # Before
    case(
        True,
        Check(group=None, before="promote"),
        "promote",
        None,
        "before",
        id="When checking before stage, accepts before check",
    ),
    case(
        False,
        Check(group=None, after="promote"),
        "promote",
        None,
        "before",
        id="When checking before stage, reject after check",
    ),
    # After
    case(
        False,
        Check(group=None, before="promote"),
        "promote",
        None,
        "after",
        id="When checking after stage, reject before check",
    ),
    case(
        True,
        Check(group=None, after="promote"),
        "promote",
        None,
        "after",
        id="When checking after stage, accept after check",
    ),
    # With groups
    case(
        True,
        Check(group=None, after="promote"),
        "promote",
        "canaries",
        "after",
        id="When doing canaries, accept a check without group",
    ),
    case(
        True,
        Check(group="canaries", after="promote"),
        "promote",
        "canaries",
        "after",
        id="When doing canaries, accept a check for canaries",
    ),
    case(
        False,
        Check(group=None, after="promote"),
        "fetch",
        "canaries",
        "after",
        id="When doing canaries, reject check for different stage",
    ),
    case(
        False,
        Check(group="canaries", after="promote"),
        "fetch",
        "canaries",
        "after",
        id="When doing canaries, reject canaries check for different stage",
    ),
    case(
        False,
        Check(group="servers", after="promote"),
        "promote",
        "canaries",
        "after",
        id="When doing canaries, reject check for different group",
    ),
    case(
        True,
        Check(group=None, after="restart_service"),
        "handle_service",
        None,
        "after",
        id="When checking after 'handle_service', accept 'restart_service' as a synonym",
    ),
]


def test_DeployLocal__load_config_exits_on_missing_git_server():
    scap_deploy = scap.cli.Application.factory(
        [
            "deploy-local",
            "--repo",
            "/tmp/whatever",
        ]
    )
    with pytest.raises(
        SystemExit, match="'git_server' is not set in scap configuration"
    ):
        scap_deploy._load_config(use_global_config=False)


def test_DeployLocal__load_config_passes_when_git_server_is_set():
    git_server = "deploy001"
    repo_path = "/tmp/whatever"

    scap_deploy = scap.cli.Application.factory(
        [
            "deploy-local",
            "--repo",
            repo_path,
        ]
    )
    configured = {
        **scap.config.DEFAULT_CONFIG,
        **{"git_server": (str, git_server)},
    }
    with override_default_config(configured):
        # Avoid a HTTP call made by _get_remote_overrides
        with patch.object(scap_deploy, "_get_config_overrides"):
            scap_deploy._load_config(use_global_config=False)

    assert scap_deploy.server_url == "http://" + git_server + repo_path


def test_get_git_binary_managers():
    logger = Mock()

    def managers(git_fat, git_binary_manager):
        return get_git_binary_managers(
            {"git_fat": git_fat, "git_binary_manager": git_binary_manager}, logger
        )

    assert managers(False, None) == []
    assert managers(False, "git-lfs") == ["git-lfs"]
    assert managers(False, "git-fat, git-lfs") == ["git-fat", "git-lfs"]
    assert logger.warning.call_count == 0

    # The deprecated git_fat config wins over git_binary_manager
    assert managers(True, "git-lfs") == ["git-fat"]
    assert logger.warning.call_count == 1


def make_deploy_local(tmp_path, git_binary_manager, git_submodules):
    deploy_local = DeployLocal("deploy-local")
    deploy_local.arguments = Mock(force=False)
    deploy_local.rev = "somerev"
    deploy_local.server_url = "http://deploy001/repo"
    deploy_local.context = Mock(cache_dir=str(tmp_path / "cache"))
    # A revision directory which does not exist yet
    deploy_local.context.rev_path.return_value = str(tmp_path / "revs" / "somerev")
    deploy_local.config = {
        "git_fat": False,
        "git_binary_manager": git_binary_manager,
        "git_submodules": git_submodules,
        "git_upstream_submodules": False,
    }

    return deploy_local


def git_call_names(mock_git):
    return [name for name, _, _ in mock_git.mock_calls]


@patch("scap.deploy.git")
def test_DeployLocal_fetch_installs_lfs_config(mock_git, tmp_path):
    mock_git.LFS = scap.git.LFS
    mock_git.FAT = scap.git.FAT

    deploy_local = make_deploy_local(tmp_path, "git-lfs", git_submodules=True)
    rev_dir = deploy_local.context.rev_path.return_value

    deploy_local.fetch()

    # The filter config of the revision directory must be in place before the
    # checkout, and the config of the submodules before the pull
    assert git_call_names(mock_git) == [
        "fetch",
        "update_submodules",
        "fetch",
        "lfs_install_local",
        "checkout",
        "update_submodules",
        "lfs_install_local_submodules",
        "largefile_pull",
    ]
    mock_git.lfs_install_local.assert_called_once_with(rev_dir)
    mock_git.lfs_install_local_submodules.assert_called_once_with(rev_dir)


@patch("scap.deploy.git")
def test_DeployLocal_fetch_installs_lfs_config_without_submodules(mock_git, tmp_path):
    mock_git.LFS = scap.git.LFS
    mock_git.FAT = scap.git.FAT

    deploy_local = make_deploy_local(tmp_path, "git-lfs", git_submodules=False)

    deploy_local.fetch()

    mock_git.lfs_install_local.assert_called_once()
    mock_git.lfs_install_local_submodules.assert_not_called()


@patch("scap.deploy.git")
@pytest.mark.parametrize("git_binary_manager", [None, "git-fat"])
def test_DeployLocal_fetch_without_lfs(mock_git, git_binary_manager, tmp_path):
    mock_git.LFS = scap.git.LFS
    mock_git.FAT = scap.git.FAT

    deploy_local = make_deploy_local(tmp_path, git_binary_manager, git_submodules=True)

    deploy_local.fetch()

    mock_git.lfs_install_local.assert_not_called()
    mock_git.lfs_install_local_submodules.assert_not_called()


def make_deploy(tmp_path, git_binary_manager, git_submodules):
    """Return a Deploy which runs in --init mode, so main() stops after setup."""

    deploy = Deploy("deploy")
    deploy.arguments = MagicMock(
        init=True, stages=None, service_restart=False, dry_run=False, rev=None
    )
    deploy.context = Mock(root=str(tmp_path))
    deploy.all_targets = ["target1"]
    deploy._build_deploy_groups = Mock()
    deploy.get_keyholder_key = Mock(return_value="somekey")
    deploy.get_scap3_lock_file = Mock(return_value=str(tmp_path / "lock"))
    deploy.lock = MagicMock()
    deploy.Timer = MagicMock()
    deploy.config_deploy_setup = Mock()
    deploy.checks_setup = Mock()
    deploy.config = {
        "git_repo": "somerepo",
        "git_rev": None,
        "environment": None,
        "tags_to_keep": 20,
        "git_fat": False,
        "git_binary_manager": git_binary_manager,
        "git_submodules": git_submodules,
    }

    return deploy


@patch("scap.deploy.git")
def test_Deploy_installs_lfs_config_on_the_deploy_server(mock_git, tmp_path):
    mock_git.LFS = scap.git.LFS
    mock_git.FAT = scap.git.FAT
    mock_git.info.return_value = {
        "headSHA1": "1234567890",
        "remoteURL": "http://gerrit/somerepo",
    }

    deploy = make_deploy(tmp_path, "git-lfs", git_submodules=True)

    assert deploy.main() == 0

    names = git_call_names(mock_git)
    assert names.index("update_server_info") < names.index("lfs_install_local")
    mock_git.lfs_install_local.assert_called_once_with(str(tmp_path))
    mock_git.lfs_install_local_submodules.assert_called_once_with(str(tmp_path))


@patch("scap.deploy.git")
def test_Deploy_without_lfs(mock_git, tmp_path):
    mock_git.LFS = scap.git.LFS
    mock_git.FAT = scap.git.FAT
    mock_git.info.return_value = {
        "headSHA1": "1234567890",
        "remoteURL": "http://gerrit/somerepo",
    }

    deploy = make_deploy(tmp_path, None, git_submodules=True)

    assert deploy.main() == 0

    mock_git.lfs_install_local.assert_not_called()
    mock_git.lfs_install_local_submodules.assert_not_called()


@pytest.mark.parametrize("expected,check,stage,group,when", valid_chk_testcases)
def test_DeployLocal_check_applies(expected, check, stage, group, when):
    assert DeployLocal._check_applies(check, stage, group, when) == expected, (
        "_check_applies(stage: %s, group: %s, when: %s) " % (stage, group, when)
        + "accepts"
        if expected
        else "rejects" + " <Check stage: %s, group: %s>" % (stage, group)
    )


@patch.dict("os.environ", clear=True)
@patch("scap.deploy.checks.execute")
@patch("scap.deploy.context")
def test_DeployLocal_before_after_checks(scap_context, checks_execute, tmp_path):
    TESTED_STAGE = "fetch"

    scap_deploy = scap.cli.Application.factory(
        [
            "deploy-local",
            "--repo",
            tmp_path.name,
            "--stage",
            TESTED_STAGE,
        ]
    )

    configured = {
        **scap.config.DEFAULT_CONFIG,
        **{"git_server": (str, "deploy001")},
    }
    with override_default_config(configured):
        scap_deploy._load_config(use_global_config=False)

    scap_deploy._setup_loggers()

    scap.script.register("runme-before-stage", "/path/to/runme-before")
    scap.script.register("runme-after-stage", "/path/to/runme-after")
    scap.script.register("runme-stage", "/path/to/runme")

    scap_deploy.config = {
        "perform_checks": True,
        "nrpe_dir": "/invalidII7I",
        "git_rev": "some_sha1",
        "git_server": "some_sha1",
        "git_repo": "some_sha1",
    }
    scap_deploy.config["checks"] = {
        "before-runme": {
            "type": "script",
            "command": "runme-before-stage",
            "before": TESTED_STAGE,
        },
        "after-runme": {
            "type": "script",
            "command": "runme-after-stage",
            "after": TESTED_STAGE,
        },
        "runme": {
            "type": "script",
            "command": "runme-stage",
            "stage": TESTED_STAGE,
        },
        # Exercise T372921
        "dontrunme": {},
    }

    run_stage = Mock(name="run stage")
    setattr(scap_deploy, TESTED_STAGE, run_stage)

    checks_execute.return_value = (True, [])
    scap_deploy_status = scap_deploy.main()

    # Ensure we ran the scripts before or after

    # With python 3.8 we could retrieve args directly
    # args = checks_execute.mock_calls[0].args[0]
    (name, args, kwargs) = checks_execute.mock_calls[0]
    checks = args[0]

    invoked_check_names = [check.name for check in checks]
    assert invoked_check_names == ["before-runme"]

    run_stage.assert_called()

    # With python 3.8 we could retrieve args directly
    # args = checks_execute.mock_calls[0].args[0]
    (name, args, kwargs) = checks_execute.mock_calls[1]
    checks = args[0]
    invoked_check_names = [check.name for check in checks]
    assert invoked_check_names == ["after-runme", "runme"]

    assert scap_deploy_status == 0

    # Ensure before script failing does not execute anything else

    run_stage.reset_mock()

    checks_execute.reset_mock()
    checks_execute.return_value = (False, [])

    scap_deploy_status = scap_deploy.main()

    # With python 3.8 we could retrieve args directly
    # args = checks_execute.mock_calls[0].args[0]
    (name, args, kwargs) = checks_execute.mock_calls[0]
    checks = args[0]
    invoked_check_names = [check.name for check in checks]
    assert invoked_check_names == ["before-runme"]

    run_stage.assert_not_called()

    assert scap_deploy_status == 2


class TestDeploy(unittest.TestCase):
    def test_bad_invocation(self):
        scap_deploy = scap.cli.Application.factory(["deploy"])

        with tempfile.TemporaryDirectory() as tmpdir:
            with scap.utils.cd(tmpdir):
                with pytest.raises(SystemExit) as e:
                    scap_deploy._load_config(use_global_config=False)
                assert (
                    e.value.code
                    == """Deployment configuration not found.
For `scap deploy` to work, the current directory must be the top level of a git repo containing a scap/scap.cfg file."""
                )
                # Verify that no files were created.
                assert len(os.listdir(tmpdir)) == 0

    def test_blank_check(self):
        # Test T372921/T149668
        scap_deploy = scap.cli.Application.factory(["deploy", "--environment", "env1"])

        with tempfile.TemporaryDirectory() as tmpdir:
            with scap.utils.cd(tmpdir):
                scap.git.init(tmpdir)
                os.makedirs(os.path.join(tmpdir, "scap", "environments", "env1"))

                with open(os.path.join(tmpdir, "scap", "scap.cfg"), "w") as f:
                    f.write(
                        """
[global]
# hello world
"""
                    )
                with open(os.path.join(tmpdir, "scap", "checks.yaml"), "w") as f:
                    checks = {
                        "checks": {
                            "check1": {
                                "after": "fetch",
                            },
                            "check2": {
                                "before": "promote",
                            },
                        }
                    }
                    yaml.dump(checks, f, indent=4)

                with open(
                    os.path.join(tmpdir, "scap", "environments", "env1", "checks.yaml"),
                    "w",
                ) as f:
                    # Disable inherited check1.
                    checks = {"checks": {"check1": {}}}
                    yaml.dump(checks, f, indent=4)

                scap.git.add_all(tmpdir)

                scap_deploy.setup(use_global_config=False)
                scap_deploy.checks_setup()

                # Verify that the override of check1 took effect.
                assert not scap_deploy.deploy_info["checks"]["check1"]
