#!/usr/bin/env python2

import unittest
from scap import targets


class TargetsTest(unittest.TestCase):
    hosts = [
        'test01',
        'test02',
        'test01.eqiad.wmnet',
        'test02.eqiad.wmnet',
        'test01.staging.eqiad.wmflabs',
        'test02.staging.eqiad.wmflabs'
    ]

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
                      'test02.staging.eqiad.wmflabs']),
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
