from unittest.mock import patch

from scap.deploy_promote import DeployPromote
import pytest

messages_tests = [
    (
        "",
        "group3 wikis to 1.42.0-wmf.00",
        "group3 wikis to 1.42.0-wmf.00",
    ),
    (
        "T777",
        "group3 wikis to 1.42.0-wmf.00  refs T777",
        (
            "group3 wikis to 1.42.0-wmf.00\n"
            "\n"
            "Bug: T777"
        ),
     ),
]


@pytest.mark.parametrize("task,announce,commit", messages_tests)
def test_set_messages(task, announce, commit):
    p = get_DeployPromote_with_messages(task)

    assert p.announce_message == announce
    assert p.commit_message == commit
    assert "\n" not in p.announce_message


@patch.object(DeployPromote, '__init__', return_value=None)
def get_DeployPromote_with_messages(task, init):
    with patch.object(DeployPromote, '_get_train_task') as gtt:
        gtt.return_value = task

        p = DeployPromote()
        p.group = "group3"
        p.promote_version = "1.42.0-wmf.00"

        p._set_messages()

    return p
