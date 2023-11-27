import unittest

from scap import InstallWorld


class InstallWorldTest(unittest.TestCase):
    def test_classify_targets_by_dc_master(self):
        install_world = InstallWorld("TestInstallWorld")
        install_world.masters = ["m.dc1.com", "m.dc2.com"]
        install_world.targets = [
            "t1.dc1.com",
            "t2.dc1.com",
            "t.dc2.com",
            "t.dc2.org",
            "t.no.com",
        ]

        (
            targets_by_master,
            targets_no_master,
        ) = install_world._map_targets_to_master_by_dc()

        self.assertDictEqual(
            targets_by_master,
            dict(
                {"m.dc1.com": ["t1.dc1.com", "t2.dc1.com"], "m.dc2.com": ["t.dc2.com"]}
            ),
        )
        self.assertSetEqual(set(targets_no_master), {"t.dc2.org", "t.no.com"})
