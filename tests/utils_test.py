#!/usr/bin/env python2

import os
import unittest
from scap import utils


class UtilsTest(unittest.TestCase):
    hosts = [
        'test01',
        'test02',
        'test01.eqiad.wmnet',
        'test02.eqiad.wmnet',
        'test01.staging.eqiad.wmflabs',
        'test02.staging.eqiad.wmflabs'
    ]

    def test_check_php_opening_tag(self):
        dir = os.path.join(os.path.dirname(__file__), 'php-data')
        files = os.listdir(dir)
        for filename in files:
            path = os.path.join(dir, filename)
            if filename.startswith('good'):
                utils.check_php_opening_tag(path)
                # No exception thrown
                self.assertTrue(True)
            elif filename.startswith('bad'):
                self.assertRaises(
                    ValueError,
                    utils.check_php_opening_tag,
                    path
                )

    def test_check_valid_json_file(self):
        dir = os.path.join(os.path.dirname(__file__), 'json-data')
        files = os.listdir(dir)
        for filename in files:
            path = os.path.join(dir, filename)
            if filename.startswith('good'):
                utils.check_valid_json_file(path)
                # No exception thrown
                self.assertTrue(True)
            elif filename.startswith('bad'):
                self.assertRaises(
                    ValueError,
                    utils.check_valid_json_file,
                    path
                )

    def test_check_target_hosts(self):
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
        return sorted(utils.get_target_hosts(x, self.hosts))

    def test_get_env_specific_filename(self):
        base_dir = os.path.join(os.path.dirname(__file__), 'env-test')

        filepath = os.path.join(base_dir, 'README')
        filepath_fake = os.path.join(base_dir, 'fake')
        test_filepath = os.path.join(base_dir,
                                     'environments', 'test', 'README')
        test_filepath_fake = os.path.join(base_dir,
                                          'environments', 'test', 'fake')

        equals = [
            (
                self._env(filepath, None),
                filepath,
                'Existing files should work without env'
            ),
            (
                self._env(filepath_fake, None),
                filepath_fake,
                'Non-existent files should work without env'
            ),
            (
                self._env(filepath),
                test_filepath,
                'Existent files should work with an env'
            ),
            (
                self._env(filepath_fake),
                filepath_fake,
                'Non-existent files with an environment should return the'
                'original path'
            ),
        ]

        not_equals = [
            (
                self._env(filepath_fake),
                test_filepath_fake,
                'Non-existent files within an environment should not return'
                'an environment specific path'
            ),
            (
                self._env(filepath),
                filepath,
                'Files that exist within an environment should not return'
                'their non-environment specific path'
            ),
        ]

        for test in equals:
            self.assertEqual(test[0], test[1], test[2])

        for test in not_equals:
            self.assertNotEqual(test[0], test[1], test[2])

    def _env(self, path, env='test'):
        if env is None:
            return utils.get_env_specific_filename(path)
        else:
            return utils.get_env_specific_filename(path, env)


if __name__ == '__main__':
    unittest.main()
