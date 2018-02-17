from __future__ import absolute_import

import os
import pytest
from scap import targets


_HOSTS = [
    'test01',
    'test02',
    'test03',
    'test01.eqiad.wmnet',
    'test02.eqiad.wmnet',
    'test01.staging.eqiad.wmflabs',
    'test02.staging.eqiad.wmflabs'
]


def test_dsh_target_refactor():
    dsh_file = 'test-targets'
    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
    target_obj = targets.get(dsh_file, {dsh_file: dsh_file},
                             extra_paths=[dsh_path])
    assert sorted(_HOSTS) == sorted(
        target_obj.get_deploy_groups()['all_targets'])


def test_dsh_targets():
    dsh_file = 'test-targets'
    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
    deploy_groups = targets.get(dsh_file,
                                {dsh_file: dsh_file},
                                extra_paths=[dsh_path])
    assert sorted(_HOSTS) == sorted(deploy_groups.all)


def test_dsh_target_default_groups():
    dsh_file = 'test-targets'
    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
    target_obj = targets.get(dsh_file, {dsh_file: dsh_file},
                             extra_paths=[dsh_path])
    deploy_groups = target_obj.groups

    # Without a group in the config, the only group should be "default"
    assert len(deploy_groups.keys()) == 1
    assert deploy_groups.keys()[0] == 'default'


def test_dsh_target_other_groups():
    dsh_file = 'test-targets'
    canary_test_targets = 'canary_test-targets'

    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')

    cfg = {'server_groups': 'canary,default',
           dsh_file: dsh_file,
           canary_test_targets: canary_test_targets}

    target_obj = targets.get(dsh_file, cfg, extra_paths=[dsh_path])
    deploy_groups = target_obj.groups

    # Since server_groups is set, there should be 2 groups:
    # canary and default
    assert len(deploy_groups.keys()) == 2
    assert deploy_groups.keys()[0] == 'canary'


def test_dsh_global_group_size():
    dsh_file = 'test-targets'
    canary_test_targets = 'canary_test-targets'

    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')

    cfg = {'server_groups': 'canary,default',
           'group_size': 2,
           dsh_file: dsh_file,
           canary_test_targets: canary_test_targets}

    target_obj = targets.get(dsh_file, cfg, extra_paths=[dsh_path])
    deploy_groups = target_obj.groups

    assert len(deploy_groups) == 2
    assert 'default' in deploy_groups
    assert 'canary' in deploy_groups
    assert isinstance(deploy_groups['default'], targets.DeployGroup)
    assert isinstance(deploy_groups['canary'], targets.DeployGroup)
    assert deploy_groups['default'].size == 2
    assert deploy_groups['canary'].size == 2


def test_dsh_target_group_size():
    dsh_file = 'test-targets'
    canary_test_targets = 'canary_test-targets'

    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')

    cfg = {'server_groups': 'canary,default',
           'canary_group_size': 2,
           dsh_file: dsh_file,
           canary_test_targets: canary_test_targets}

    target_obj = targets.get(dsh_file, cfg, extra_paths=[dsh_path])
    deploy_groups = target_obj.groups

    assert len(deploy_groups) == 2
    assert 'default' in deploy_groups
    assert 'canary' in deploy_groups
    assert isinstance(deploy_groups['default'], targets.DeployGroup)
    assert isinstance(deploy_groups['canary'], targets.DeployGroup)
    assert deploy_groups['canary'].size == 2
    assert deploy_groups['default'].size != 2


def test_limit_target_hosts():
    tests = [
        (_gth('*'),
         sorted(_HOSTS)),
        (_gth('all'),
         sorted(_HOSTS)),
        (_gth('test01FeqiadFwmnet'), []),
        (_gth('test01.eqiad.wmnet'),
         ['test01.eqiad.wmnet']),
        (_gth('test[01:02].eqiad.*'),
         ['test01.eqiad.wmnet', 'test02.eqiad.wmnet']),
        (_gth('test*.eqiad.*'),
         ['test01.eqiad.wmnet', 'test01.staging.eqiad.wmflabs',
          'test02.eqiad.wmnet', 'test02.staging.eqiad.wmflabs']),
        (_gth('!test[01:02].eqiad.*'),
         ['test01', 'test01.staging.eqiad.wmflabs',
          'test02', 'test02.staging.eqiad.wmflabs',
          'test03']),
        (_gth('~test.*.eqiad.wm(net|flabs)'),
         ['test01.eqiad.wmnet', 'test01.staging.eqiad.wmflabs',
          'test02.eqiad.wmnet', 'test02.staging.eqiad.wmflabs']),
        (_gth('test*.eqia[a:d].wmflabs'),
         ['test01.staging.eqiad.wmflabs', 'test02.staging.eqiad.wmflabs']),
        (_gth('test[01:02]'),
         ['test01', 'test02']),
        (_gth('test02'),
         ['test02']),
        (_gth('tes?02'), []),
        (_gth('tes.02'), []),
    ]

    for test in tests:
        assert test[0] == test[1]

    with pytest.raises(ValueError):
        _gth('test[10:01]')
        _gth('test[z:a]')


def test_get_deploy_groups__excludes_empty_groups():
    dsh_file = 'empty-targets'
    dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
    target_obj = targets.get(dsh_file, {dsh_file: dsh_file},
                             extra_paths=[dsh_path])
    assert not target_obj.groups


def _gth(x):
    return sorted(targets.limit_target_hosts(x, _HOSTS))


_TARGETS = ['target1', 'target2', 'target3']


def test_size__defaults_to_length_of_targets():
    group = targets.DeployGroup('foo', _TARGETS)
    assert group.size == len(_TARGETS)


def test_size__must_be_greater_than_zero():
    with pytest.raises(ValueError):
        targets.DeployGroup('foo', _TARGETS, size=0)
        targets.DeployGroup('foo', _TARGETS, size=-1)


def test_targets__must_not_be_empty():
    with pytest.raises(ValueError):
        targets.DeployGroup('foo', [])


def test_failure_limit__defaults_to_one():
    group = targets.DeployGroup('foo', _TARGETS)
    assert group.failure_limit == 1


def test_failure_limit__accepts_int():
    group = targets.DeployGroup('foo', _TARGETS, failure_limit=2)
    assert group.failure_limit == 2


def test_failure_limit__converts_and_floors_percentage():
    group = targets.DeployGroup('foo', _TARGETS, failure_limit='75%')
    assert group.failure_limit == 2

    group = targets.DeployGroup('foo', _TARGETS, failure_limit='33.4%')
    assert group.failure_limit == 1


def test_failure_limit__percentage_is_based_on_original_size():
    group = targets.DeployGroup('foo', _TARGETS,
                                size=2, failure_limit='75%')
    assert group.failure_limit == 2


def test_subgroups__splits_and_labels_targets_according_to_size():
    group = targets.DeployGroup('foo', _TARGETS, 2)
    subgroups = list(group.subgroups())
    assert subgroups == [('foo1', ['target1', 'target2']),
                         ('foo2', ['target3'])]


def test_subgroups__with_no_size_maintains_original_group():
    group = targets.DeployGroup('foo', _TARGETS)
    subgroups = list(group.subgroups())
    assert subgroups == [('foo', _TARGETS)]


def test_subgroups__with_excluded_target():
    group = targets.DeployGroup('foo', _TARGETS, 2)
    group.exclude('target2')
    subgroups = list(group.subgroups())
    assert subgroups == [('foo', ['target1', 'target3'])]
