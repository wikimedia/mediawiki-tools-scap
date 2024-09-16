import unittest

from scap.plugins import gerrit


class TestGerritSession(unittest.TestCase):
    def test_change_number_from_url(self):
        session = gerrit.GerritSession("http://gerrit.example/")

        matchingURLs = [
            "123456",
            "http://gerrit.example/c/mediawiki/core/+/123456",
            "http://gerrit.example/c/mediawiki/core/+/123456/1",
            "http://gerrit.example/r/123456",
            "http://gerrit.example/r/c/123456",
        ]

        badURLs = [
            "http://some.other.gerrit.example/c/mediawiki/core/+/123456",
            "-123456",
            # non-ascii digits
            "http://gerrit.example/r/１２３４５６",
        ]

        for url in matchingURLs:
            with self.subTest(url=url):
                self.assertEqual(session.change_number_from_url(url), "123456")

        for url in badURLs:
            with self.subTest(url=url):
                self.assertIsNone(session.change_number_from_url(url))
