from __future__ import absolute_import

import os

from scap import utils


def test_version_re():
    match = utils.BRANCH_RE.match('1.29.0-wmf.12')

    assert match is not None

    version = match.groupdict()
    assert version['major'] == '1'
    assert version['minor'] == '29'
    assert version['patch'] == '0'
    assert version['prerelease'] == '12'

    match = utils.BRANCH_RE.match('1.290.0-wmf.12')
    assert match is None

    match = utils.BRANCH_RE.match('1.29.0wmf.12')
    assert match is None

    match = utils.BRANCH_RE.match('1.28.0-wmf.6')
    assert match is not None

    match = utils.BRANCH_RE.match('1.28.15-wmf.6')
    assert match is not None

    version = match.groupdict()
    assert version['prerelease'] == '6'


def test_get_env_specific_filename():
    base_dir = os.path.join(os.path.dirname(__file__), 'env-test')

    filepath = os.path.join(base_dir, 'README')
    filepath_fake = os.path.join(base_dir, 'fake')
    test_filepath = os.path.join(base_dir, 'environments', 'test', 'README')
    test_filepath_fake = os.path.join(base_dir, 'environments', 'test', 'fake')

    equals = [
        (
            _env(filepath, None),
            filepath,
            'Existing files should work without env'
        ),
        (
            _env(filepath_fake, None),
            filepath_fake,
            'Non-existent files should work without env'
        ),
        (
            _env(filepath),
            test_filepath,
            'Existent files should work with an env'
        ),
        (
            _env(filepath_fake),
            filepath_fake,
            'Non-existent files with an environment should return the'
            'original path'
        ),
    ]

    not_equals = [
        (
            _env(filepath_fake),
            test_filepath_fake,
            'Non-existent files within an environment should not return'
            'an environment specific path'
        ),
        (
            _env(filepath),
            filepath,
            'Files that exist within an environment should not return'
            'their non-environment specific path'
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
    os.environ['PHP'] = 'phptest'
    os.environ['FOO'] = 'bar'
    os.environ['BAZ'] = 'bam'
    assert utils.make_sudo_check_call_env(['PHP']) == 'PHP="phptest"'
    assert 'PHP="phptest"' in utils.make_sudo_check_call_env(
        ['PHP', 'FOO']
    )
    assert 'FOO="bar"' in utils.make_sudo_check_call_env(
        ['PHP', 'FOO']
    )
    assert 'bam' in utils.make_sudo_check_call_env(['BAZ'])
    assert '' == utils.make_sudo_check_call_env(['BAM'])


def _env(path, env='test'):
    if env is None:
        return utils.get_env_specific_filename(path)
    return utils.get_env_specific_filename(path, env)
