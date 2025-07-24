import argparse
import json
import os
import pytest
from unittest.mock import patch

from scap import utils


def test_version_re():
    match = utils.BRANCH_RE.match("1.29.0-wmf.12")

    assert match is not None

    version = match.groupdict()
    assert version["major"] == "1"
    assert version["minor"] == "29"
    assert version["patch"] == "0"
    assert version["prerelease"] == "12"

    match = utils.BRANCH_RE.match("1.290.0-wmf.12")
    assert match is None

    match = utils.BRANCH_RE.match("1.29.0wmf.12")
    assert match is None

    match = utils.BRANCH_RE.match("1.28.0-wmf.6")
    assert match is not None

    match = utils.BRANCH_RE.match("1.28.15-wmf.6")
    assert match is not None

    version = match.groupdict()
    assert version["prerelease"] == "6"


def test_get_env_specific_filename():
    base_dir = os.path.join(os.path.dirname(__file__), "env-test")

    filepath = os.path.join(base_dir, "README")
    filepath_fake = os.path.join(base_dir, "fake")
    test_filepath = os.path.join(base_dir, "environments", "test", "README")
    test_filepath_fake = os.path.join(base_dir, "environments", "test", "fake")

    equals = [
        (_env(filepath, None), filepath, "Existing files should work without env"),
        (
            _env(filepath_fake, None),
            filepath_fake,
            "Non-existent files should work without env",
        ),
        (_env(filepath), test_filepath, "Existent files should work with an env"),
        (
            _env(filepath_fake),
            filepath_fake,
            "Non-existent files with an environment should return the" "original path",
        ),
    ]

    not_equals = [
        (
            _env(filepath_fake),
            test_filepath_fake,
            "Non-existent files within an environment should not return"
            "an environment specific path",
        ),
        (
            _env(filepath),
            filepath,
            "Files that exist within an environment should not return"
            "their non-environment specific path",
        ),
    ]

    for test in equals:
        assert test[0] == test[1], test[2]

    for test in not_equals:
        assert test[0] != test[1], test[2]


def test_cpus_for_jobs():
    cpus = utils.cpus_for_jobs()
    assert cpus > 0


def test_make_sudo_check_call_env():
    os.environ["PHP"] = "phptest"
    os.environ["FOO"] = "bar"
    os.environ["BAZ"] = "bam"
    assert utils.make_sudo_check_call_env(["PHP"]) == 'PHP="phptest"'
    assert 'PHP="phptest"' in utils.make_sudo_check_call_env(["PHP", "FOO"])
    assert 'FOO="bar"' in utils.make_sudo_check_call_env(["PHP", "FOO"])
    assert "bam" in utils.make_sudo_check_call_env(["BAZ"])
    assert "" == utils.make_sudo_check_call_env(["BAM"])


def _env(path, env="test"):
    if env is None:
        return utils.get_env_specific_filename(path)
    return utils.get_env_specific_filename(path, env)


def test_is_phabricator_task_id():
    assert utils.is_phabricator_task_id("T0") is True
    assert utils.is_phabricator_task_id("T123") is True
    assert utils.is_phabricator_task_id("") is False
    assert utils.is_phabricator_task_id("T") is False
    assert utils.is_phabricator_task_id("T123X") is False
    assert utils.is_phabricator_task_id(" T1") is False
    assert utils.is_phabricator_task_id("T1 ") is False


def test_list_intersection():
    assert utils.list_intersection([1, 2, 3], [2, 3, 4]) == [2, 3]
    assert utils.list_intersection([2, 3, 4], [1, 2, 3]) == [2, 3]
    assert utils.list_intersection([1, 2], [3, 4]) == []


def test_parse_rsync_stats():
    test_string = """
stuff before
Number of files: 184,935 (reg: 171,187, dir: 13,596, link: 152)
Number of created files: 0
Number of deleted files: 0
Number of regular files transferred: 1
Total file size: 8,756,954,367 bytes
Total transferred file size: 815,772 bytes
Literal data: 0 bytes
Matched data: 815,772 bytes
File list size: 4,744,396
File list generation time: 0.517 seconds
File list transfer time: 0.000 seconds
Total bytes sent: 5,603
Total bytes received: 4,744,454
stuff after
    """

    assert utils.parse_rsync_stats(test_string) == {
        "files": 184935,
        "files_created": 0,
        "files_deleted": 0,
        "regular_files_transferred": 1,
        "total_file_size": 8756954367,
        "total_transferred_file_size": 815772,
        "literal_data": 0,
        "matched_data": 815772,
        "file_list_size": 4744396,
        "total_bytes_sent": 5603,
        "total_bytes_received": 4744454,
    }


def test_temp_to_permanent_file(tmpdir):
    testfile = os.path.join(tmpdir, "test_temp_to_permanent_file")

    with utils.temp_to_permanent_file(testfile) as f:
        f.write("Very important")

    with open(testfile) as f:
        assert f.read() == "Very important"

    try:
        with utils.temp_to_permanent_file(testfile) as f:
            tmpname = f.name
            f.write("Overwritten")
            raise Exception("Failed operation")
    except Exception:
        pass

    # Verify that the final file remains unchanged
    with open(testfile) as f:
        assert f.read() == "Very important"
    import subprocess

    subprocess.run("ls -l {}".format(tmpname), shell=True)
    # Verify that the temp file was deleted
    assert not os.path.exists(tmpname)


def test_write_file_if_needed(tmpdir):
    testfile = os.path.join(tmpdir, "testfile")

    assert utils.write_file_if_needed(testfile, "First write") is True
    assert utils.write_file_if_needed(testfile, "First write") is False
    assert utils.write_file_if_needed(testfile, "Second write") is True


@pytest.fixture
def sample_wikiversions_file(tmpdir):
    wikiversions_file = os.path.join(tmpdir, "wikiversions-test.json")

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

    return wikiversions_file


@patch.dict("os.environ", {"FORCE_MW_VERSION": "1.23.4-wmf.5"})
def test_read_wikiversions_force_mw_version(tmpdir, sample_wikiversions_file):
    res = utils.read_wikiversions(tmpdir, "test")

    for value in res.values():
        assert value == "php-1.23.4-wmf.5"


def test_get_active_wikiversions(tmpdir, sample_wikiversions_file):
    # Test invalid return_type.
    with pytest.raises(ValueError):
        utils.get_active_wikiversions(None, None, return_type=str)

    res = utils.get_active_wikiversions(tmpdir, "test", return_type=list)
    assert res == ["1.42.0-wmf.22", "1.42.0-wmf.25"]

    res = utils.get_active_wikiversions(tmpdir, "test", return_type=dict)
    assert res == {"1.42.0-wmf.22": "herwiki", "1.42.0-wmf.25": "hiswiki"}


@pytest.mark.parametrize(
    "ver", ["auto", "1.42.0-wmf.22", "master", "branch_cut_pretest", "next"]
)
def test_version_argument_parser_good(ver):
    assert utils.version_argument_parser(ver) == ver
    if ver != "auto":
        assert utils.valid_version(ver) is True


@pytest.mark.parametrize("ver", ["", "123", "1.42.0-wmf.22 ", "testing", "auto"])
def test_version_argument_parser_bad(ver):
    with pytest.raises(argparse.ArgumentTypeError):
        utils.version_argument_parser(ver, allow_auto=False)
    assert utils.valid_version(ver) is False


def test_parse_wmf_version():
    assert (
        utils.parse_wmf_version("1.23.0-wmf.1")
        < utils.parse_wmf_version("1.23.0-wmf.2")
        < utils.parse_wmf_version("branch_cut_pretest")
        < utils.parse_wmf_version("master")
    )
    assert utils.parse_wmf_version("branch_cut_pretest") == utils.parse_wmf_version(
        "next"
    )


@pytest.fixture
def messy_patches_dir(tmpdir):
    for version in [
        "123",
        "1.42.0-wmf.22",
        "testing",
        "auto",
        "next",
        "master",
    ]:
        os.mkdir(tmpdir / version)

    return tmpdir


def test_get_patch_versions(messy_patches_dir):
    assert utils.get_patch_versions(messy_patches_dir) == [
        "1.42.0-wmf.22",
        "next",
        "master",
    ]


def test_select_latest_patches(tmpdir):
    assert utils.select_latest_patches(tmpdir) is None

    next = tmpdir / "next"
    os.mkdir(next)

    assert utils.select_latest_patches(tmpdir) == next
    assert utils.select_latest_patches(tmpdir, before_next=True) is None

    # Add a version that's later than "next"
    master = tmpdir / "master"
    os.mkdir(master)
    assert utils.select_latest_patches(tmpdir) == master
    assert utils.select_latest_patches(tmpdir, before_next=True) is None

    # Add a normal version
    normal = tmpdir / "1.42.0-wmf.22"
    os.mkdir(normal)
    assert utils.select_latest_patches(tmpdir) == master
    assert utils.select_latest_patches(tmpdir, before_next=True) == normal


def test_get_wikiversions_ondisk(tmpdir):
    for dir in [
        "php-master",
        "php-1.40.0-wmf.4",
        "php-1.29.0-wmf.3",
        "php-1.29.0-wmf.2",
    ]:
        os.mkdir(os.path.join(tmpdir, dir))
    open(tmpdir / "php-1.40.0-wmf.2", "w").close()
    assert utils.get_wikiversions_ondisk(tmpdir) == [
        "1.29.0-wmf.2",
        "1.29.0-wmf.3",
        "1.40.0-wmf.4",
        "master",
    ]


def test_string_to_base64_string():
    assert utils.string_to_base64_string("bruce") == "YnJ1Y2U="
