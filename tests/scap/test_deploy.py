from unittest.mock import patch, Mock

import scap.cli
import scap.script
from scap.deploy import DeployLocal

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

    case(True, Check(group=None, after="promote"), "promote", None, "after",
         id="When on promote stage, accept promote check"),
    case(False, Check(group=None, after="promote", when="after"), "fetch", None, "after",
         id="When on fetch stage, reject promote check"),

    case(True, Check(group="canaries", after="promote"), "promote", None, "after",
         id="When on promote stage, accept promote check for canaries"),
    case(False, Check(group="canaries", after="promote"), "fetch", None, "after",
         id="When on promote stage, reject fetch check for canaries"),

    # Before
    case(True, Check(group=None, before="promote"), "promote", None, "before",
         id="When checking before stage, accepts before check"),
    case(False, Check(group=None, after="promote"), "promote", None, "before",
         id="When checking before stage, reject after check"),
    # After
    case(False, Check(group=None, before="promote"), "promote", None, "after",
         id="When checking after stage, reject before check"),
    case(True, Check(group=None, after="promote"), "promote", None, "after",
         id="When checking after stage, accept after check"),

    # With groups
    case(True, Check(group=None, after="promote"), "promote", "canaries", "after",
         id="When doing canaries, accept a check without group"),
    case(True, Check(group="canaries", after="promote"), "promote", "canaries", "after",
         id="When doing canaries, accept a check for canaries"),
    case(False, Check(group=None, after="promote"), "fetch", "canaries", "after",
         id="When doing canaries, reject check for different stage"),
    case(False, Check(group="canaries", after="promote"), "fetch", "canaries", "after",
         id="When doing canaries, reject canaries check for different stage"),
    case(False, Check(group="servers", after="promote"), "promote", "canaries", "after",
         id="When doing canaries, reject check for different group"),
]


@pytest.mark.parametrize("expected,check,stage,group,when",
                         valid_chk_testcases)
def test_DeployLocal_valid_chk(expected, check, stage, group, when):
    assert DeployLocal._valid_chk(check, stage, group, when) == expected, (
        "_valid_check(stage: %s, group: %s, when: %s) " % (stage, group, when)
        + "accepts" if expected else "rejects"
        + " <Check stage: %s, group: %s>" % (stage, group)
    )


@patch.dict('os.environ', clear=True)
@patch('scap.deploy.checks.execute')
@patch('scap.deploy.context')
def test_DeployLocal_before_after_checks(
        scap_context, checks_execute, tmp_path
):
    TESTED_STAGE = "fetch"

    scap_deploy = scap.cli.Application.factory([
        "deploy-local",
        "--repo", tmp_path.name,
        "--stage", TESTED_STAGE,
        ])
    scap_deploy._load_config()
    scap_deploy._setup_loggers()

    scap.script.register('runme-before-stage', '/path/to/runme-before')
    scap.script.register('runme-after-stage', '/path/to/runme-after')
    scap.script.register('runme-stage', '/path/to/runme')

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
        }
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
    assert invoked_check_names == ['before-runme']

    # With python 3.6 we could use assert_called()
    run_stage.assert_called_with()

    # With python 3.8 we could retrieve args directly
    # args = checks_execute.mock_calls[0].args[0]
    (name, args, kwargs) = checks_execute.mock_calls[1]
    checks = args[0]
    invoked_check_names = [check.name for check in checks]
    assert invoked_check_names == ['after-runme', 'runme']

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
    assert invoked_check_names == ['before-runme']

    run_stage.assert_not_called()

    assert scap_deploy_status == 2