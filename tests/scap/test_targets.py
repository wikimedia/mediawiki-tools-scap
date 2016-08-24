#!/usr/bin/env python2

import os

import unittest

from scap import targets


class TargetsTest(unittest.TestCase):
    hosts = [
        'test01',
        'test02',
        'test03',
        'test01.eqiad.wmnet',
        'test02.eqiad.wmnet',
        'test01.staging.eqiad.wmflabs',
        'test02.staging.eqiad.wmflabs'
    ]

    def test_dsh_target_refactor(self):
        dsh_file = 'test-targets'
        dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
        target_obj = targets.get(dsh_file,
                                 {dsh_file: dsh_file},
                                 extra_paths=[dsh_path])
        self.assertEqual(sorted(self.hosts),
                         sorted(target_obj.get_deploy_groups()['all_targets']))

    def test_dsh_targets(self):
        dsh_file = 'test-targets'
        dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
        deploy_groups = targets.get(dsh_file,
                                    {dsh_file: dsh_file},
                                    extra_paths=[dsh_path])
        self.assertEqual(sorted(self.hosts),
                         sorted(deploy_groups.all))

    def test_dsh_target_default_groups(self):
        dsh_file = 'test-targets'
        dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')
        target_obj = targets.get(dsh_file,
                                 {dsh_file: dsh_file},
                                 extra_paths=[dsh_path])
        deploy_groups = target_obj.groups

        # Without a group in the config, the only group should be "default"
        self.assertEqual(1, len(deploy_groups.keys()))
        self.assertEqual('default', deploy_groups.keys()[0])

    def test_dsh_target_other_groups(self):
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
        self.assertEqual(2, len(deploy_groups.keys()))
        self.assertEqual('canary', deploy_groups.keys()[0])

    def test_dsh_global_group_size(self):
        dsh_file = 'test-targets'
        canary_test_targets = 'canary_test-targets'

        dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')

        cfg = {'server_groups': 'canary,default',
               'group_size': 2,
               dsh_file: dsh_file,
               canary_test_targets: canary_test_targets}

        target_obj = targets.get(dsh_file, cfg, extra_paths=[dsh_path])
        deploy_groups = target_obj.groups

        self.assertEqual(4, len(deploy_groups.keys()))
        self.assertEqual(['canary1', 'canary2', 'default1', 'default2'],
                         deploy_groups.keys())

    def test_dsh_target_group_size(self):
        dsh_file = 'test-targets'
        canary_test_targets = 'canary_test-targets'

        dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')

        cfg = {'server_groups': 'canary,default',
               'canary_group_size': 2,
               dsh_file: dsh_file,
               canary_test_targets: canary_test_targets}

        target_obj = targets.get(dsh_file, cfg, extra_paths=[dsh_path])
        deploy_groups = target_obj.groups

        # The canary group, containing 3 hosts in this case, should have been
        # split into 2 distinct groups according to the group_size of 2
        self.assertEqual(3, len(deploy_groups.keys()))
        self.assertEqual(['canary1', 'canary2', 'default'],
                         deploy_groups.keys())
        self.assertEqual(2, len(deploy_groups['canary1']))
        self.assertEqual(1, len(deploy_groups['canary2']))
        self.assertEqual(0, len(set(deploy_groups['canary1']) &
                                set(deploy_groups['canary2'])))

    def test_dsh_target_group_size_ignored(self):
        dsh_file = 'test-targets'
        canary_test_targets = 'canary_test-targets'

        dsh_path = os.path.join(os.path.dirname(__file__), 'targets-test')

        cfg = {'server_groups': 'canary,default',
               'canary_group_size': 4,
               dsh_file: dsh_file,
               canary_test_targets: canary_test_targets}

        target_obj = targets.get(dsh_file, cfg, extra_paths=[dsh_path])
        deploy_groups = target_obj.groups

        # The group_size is ignored when greater than the total number of
        # hosts in the group
        self.assertEqual(2, len(deploy_groups.keys()))
        self.assertEqual('canary', deploy_groups.keys()[0])
        self.assertEqual(3, len(deploy_groups['canary']))

    def test_limit_target_hosts(self):
        tests = [(self._gth('*'), sorted(self.hosts)),
                 (self._gth('all'), sorted(self.hosts)),
                 (self._gth('test01FeqiadFwmnet'), []),
                 (self._gth('test01.eqiad.wmnet'),
                     ['test01.eqiad.wmnet']),
                 (self._gth('test[01:02].eqiad.*'),
                     ['test01.eqiad.wmnet',
                      'test02.eqiad.wmnet']),
                 (self._gth('test*.eqiad.*'),
                     ['test01.eqiad.wmnet',
                      'test01.staging.eqiad.wmflabs',
                      'test02.eqiad.wmnet',
                      'test02.staging.eqiad.wmflabs']),
                 (self._gth('!test[01:02].eqiad.*'),
                     ['test01',
                      'test01.staging.eqiad.wmflabs',
                      'test02',
                      'test02.staging.eqiad.wmflabs',
                      'test03']),
                 (self._gth('~test.*.eqiad.wm(net|flabs)'),
                     ['test01.eqiad.wmnet',
                      'test01.staging.eqiad.wmflabs',
                      'test02.eqiad.wmnet',
                      'test02.staging.eqiad.wmflabs']),
                 (self._gth('test*.eqia[a:d].wmflabs'),
                     ['test01.staging.eqiad.wmflabs',
                      'test02.staging.eqiad.wmflabs']),
                 (self._gth('test[01:02]'), ['test01', 'test02']),
                 (self._gth('test02'), ['test02']),
                 (self._gth('tes?02'), []),
                 (self._gth('tes.02'), []),
                 ]

        for test in tests:
            self.assertEqual(test[0], test[1])

        with self.assertRaises(ValueError):
            self._gth('test[10:01]')
            self._gth('test[z:a]')

    def _gth(self, x):
        return sorted(targets.limit_target_hosts(x, self.hosts))


if __name__ == '__main__':
    unittest.main()
