from unittest.mock import Mock

import pytest
from pytest import param as case
from scap.deploy import DeployLocal

# Aliasing for a nicer list of test cases
Check = Mock

valid_chk_testcases = [
    # Parameters:
    # - expected
    # - a Check with group and stage,
    # - current stage
    # - current group

    case(True, Check(group=None, stage="promote"), "promote", None,
         id="When on promote stage, accept promote check"),
    case(False, Check(group=None, stage="promote"), "fetch", None,
         id="When on fetch stage, reject promote check"),

    case(True, Check(group="canaries", stage="promote"), "promote", None,
         id="When on promote stage, accept promote check for canaries"),
    case(False, Check(group="canaries", stage="promote"), "fetch", None,
         id="When on promote stage, reject fetch check for canaries"),

    # With groups
    case(True, Check(group=None, stage="promote"), "promote", "canaries",
         id="When doing canaries, accept a check without group"),
    case(True, Check(group="canaries", stage="promote"), "promote", "canaries",
         id="When doing canaries, accept a check for canaries"),
    case(False, Check(group=None, stage="promote"), "fetch", "canaries",
         id="When doing canaries, reject check for different stage"),
    case(False, Check(group="canaries", stage="promote"), "fetch", "canaries",
         id="When doing canaries, reject canaries check for different stage"),
    case(False, Check(group="servers", stage="promote"), "promote", "canaries",
         id="When doing canaries, reject check for different group"),
]


@pytest.mark.parametrize("expected,check,stage,group", valid_chk_testcases)
def test_DeployLocal_valid_chk(expected, check, stage, group):
    assert DeployLocal._valid_chk(check, stage, group) == expected, (
        "_valid_check(stage: %s, group: %s) " % (stage, group)
        + "accepts" if expected else "rejects"
        + " <Check stage: %s, group: %s>" % (stage, group)
    )
