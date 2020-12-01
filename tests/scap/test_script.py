from __future__ import absolute_import

import pytest

from scap import checks
from scap import script


def test_register():
    script.register("check.sh", "/usr/local/bin/check.sh")

    check = script.ScriptCheck("name", stage="promote", command="check.sh")
    assert isinstance(check, script.ScriptCheck)


def test_unregistered_check():
    with pytest.raises(checks.CheckInvalid):
        script.ScriptCheck("name", stage="promote", command="notafile.sh")
