import unittest

from scap.plugins import gerrit


class TestGerritSession(unittest.TestCase):

    def test_change_number_from_url(self):
        session = gerrit.GerritSession('http://gerrit.example/')

        matchingURLs = [
            'http://gerrit.example/c/mediawiki/core/+/123456',
            'http://gerrit.example/c/mediawiki/core/+/123456/1',
        ]

        unknownURLs = [
            'http://some.other.gerrit.example/c/mediawiki/core/+/123456',
        ]

        for url in matchingURLs:
            with self.subTest(url=url):
                self.assertEqual(session.change_number_from_url(url), '123456')

        for url in unknownURLs:
            with self.subTest(url=url):
                self.assertIsNone(session.change_number_from_url(url))
