import unittest

from scap import InstallWorld


class InstallWorldTest(unittest.TestCase):
    def test_classify_targets_by_dc_master(self):
        install_world = InstallWorld("TestInstallWorld")
        install_world.masters = ["m.dc1.com", "m.dc2.com"]
        install_world.selected_targets = [
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

    def test_parse_codename(self):
        self.assertEqual(InstallWorld._parse_codename("bookworm\n"), "bookworm")
        self.assertEqual(
            InstallWorld._parse_codename("Warning: something\nbullseye\n"), "bullseye"
        )
        self.assertIsNone(InstallWorld._parse_codename(""))
        self.assertIsNone(
            InstallWorld._parse_codename("bash: lsb_release: command not found\n")
        )

    def test_examples(self):
        targets = ["t1.dc1.com", "t2.dc1.com", "t3.dc1.com"]

        self.assertEqual(
            InstallWorld._examples(targets, 80), "t1.dc1.com, t2.dc1.com, t3.dc1.com"
        )
        self.assertEqual(
            InstallWorld._examples(targets, 30), "t1.dc1.com, ... (2 more)"
        )
        # A target that does not fit is cut short, and the suffix is kept
        self.assertEqual(InstallWorld._examples(targets, 20), "t1.dc…, ... (2 more)")
        # Too narrow for the suffix
        self.assertEqual(InstallWorld._examples(targets, 5), "t1.d…")
        # One target has no suffix, cut or not
        self.assertEqual(InstallWorld._examples(["t1.dc1.com"], 5), "t1.d…")
        self.assertEqual(InstallWorld._examples(["t1.dc1.com"], 80), "t1.dc1.com")
        self.assertEqual(InstallWorld._examples([], 80), "")

    def test_inventory_list(self):
        inventory = {
            "bullseye": ["t1.dc1.com"],
            "bookworm": ["t2.dc1.com", "t3.dc1.com"],
        }

        self.assertEqual(
            InstallWorld._inventory_list(inventory),
            [
                "bookworm (2 targets)",
                "  t2.dc1.com",
                "  t3.dc1.com",
                "",
                "bullseye (1 target)",
                "  t1.dc1.com",
            ],
        )

    def test_inventory_list_is_empty(self):
        self.assertEqual(InstallWorld._inventory_list({}), [])

    def test_inventory_table(self):
        def row_of(table, codename):
            return next(
                line
                for line in table.splitlines()
                if line.startswith(f"\u2502 {codename}")
            )

        inventory = {
            "bullseye": ["t1.dc1.com"],
            "bookworm": ["t%d.dc1.com" % i for i in range(2, 20)],
        }

        narrow = InstallWorld._inventory_table(inventory, 60).get_string()
        wide = InstallWorld._inventory_table(inventory, 100).get_string()

        for line in narrow.splitlines():
            self.assertLessEqual(len(line), 60)
        for line in wide.splitlines():
            self.assertLessEqual(len(line), 100)

        # The codename with the most targets comes first
        rows = [line for line in narrow.splitlines() if "dc1.com" in line]
        self.assertEqual(len(rows), 2)
        self.assertIn("bookworm", rows[0])
        self.assertIn("bullseye", rows[1])
        self.assertNotIn("more)", rows[1])

        # A wider table lists more examples
        self.assertGreater(
            row_of(wide, "bookworm").count("dc1.com"),
            row_of(narrow, "bookworm").count("dc1.com"),
        )
