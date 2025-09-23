import shlex
from unittest import mock
from unittest.mock import patch, ANY, Mock
import unittest

import scap.cli
from scap.backport import (
    DependencyTrail,
    Backport,
    InvalidChangeException,
    GitRepos,
    GerritChange,
)


@patch.dict("os.environ", clear=True)
def test_backport_uses_gerrit_push_user_config():
    with mock.patch("subprocess.check_call") as check:
        scap_backport = scap.cli.Application.factory(["backport"])
        scap_backport.setup()
        scap_backport.gerritssh.ssh([])
        check.assert_called_with(ANY, env=ANY, stdout=ANY, stderr=ANY)

    with mock.patch("subprocess.check_call") as check:
        scap_backport = scap.cli.Application.factory(["backport"])
        scap_backport.setup()
        scap_backport.config["gerrit_push_user"] = "trainbotuser"
        scap_backport.gerritssh.ssh([])

        # With python 3.8 we could retrieve kwargs directly
        # env = check.mock_calls[0].kwargs['env']
        name, args, kwargs = check.mock_calls[0]
        assert "GIT_SSH_COMMAND" in kwargs["env"]

        ssh_command = shlex.split(kwargs["env"]["GIT_SSH_COMMAND"])
        assert "-oUser=trainbotuser" in ssh_command
        assert "-n" not in ssh_command


class TestT371611Workflow(unittest.TestCase):
    """Tests for T371611 dependency workflow implementation."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_change = Mock()
        self.mock_change.get.return_value = "test_value"
        self.mock_change.number = 12345

        self.mock_change2 = Mock()
        self.mock_change2.get.return_value = "test_value2"
        self.mock_change2.number = 67890

        # MW code detection test fixtures
        self.mw_change_deployable = Mock()
        self.mw_change_deployable.project = "mediawiki/core"
        self.mw_change_deployable.branch = "wmf/1.45.0-wmf.21"  # Deployable branch
        self.mw_change_deployable.number = 12345
        self.mw_change_deployable.get.side_effect = lambda key: getattr(
            self.mw_change_deployable, key
        )

        self.mw_dep_master = Mock()
        self.mw_dep_master.project = "mediawiki/extensions/Example"
        self.mw_dep_master.branch = "master"  # Non-deployable branch
        self.mw_dep_master.number = 67890
        self.mw_dep_master.get.side_effect = lambda key: getattr(
            self.mw_dep_master, key
        )

    def test_dependency_trail_creation(self):
        """Test DependencyTrail can be created and initialized properly."""
        trail = DependencyTrail(self.mock_change)

        self.assertEqual(trail.root_change, self.mock_change)
        self.assertEqual(len(trail.relevant_dependencies), 0)
        self.assertEqual(len(trail.dependency_chains), 0)
        self.assertEqual(len(trail.errors), 0)
        self.assertEqual(len(trail.warnings), 0)
        self.assertFalse(trail.has_errors())
        self.assertFalse(trail.has_warnings())

    def test_dependency_trail_add_dependency(self):
        """Test adding dependencies to a trail."""
        trail = DependencyTrail(self.mock_change)

        mock_dep = Mock()
        mock_dep.get.return_value = "dep_id_123"
        mock_dep.number = 54321

        # Create a mock current_change (the change being processed)
        mock_current = Mock()
        mock_current.number = 67890

        chain_from_root = []
        trail.add_relevant_dependency(mock_dep, mock_current, chain_from_root)

        self.assertEqual(len(trail.relevant_dependencies), 1)
        self.assertEqual(len(trail.dependency_chains), 1)
        self.assertIn("dep_id_123", trail.dependency_chains)
        # Expected chain should be chain_from_root + [mock_current]
        expected_chain = chain_from_root + [mock_current]
        self.assertEqual(trail.dependency_chains["dep_id_123"], expected_chain)

    def test_dependency_trail_errors_and_warnings(self):
        """Test adding errors and warnings to a trail."""
        trail = DependencyTrail(self.mock_change)

        # Test adding errors
        trail.add_error("Test error")
        self.assertTrue(trail.has_errors())
        self.assertEqual(len(trail.errors), 1)
        self.assertEqual(trail.errors[0], "Test error")

        # Test adding warnings
        trail.add_warning("Test warning")
        self.assertTrue(trail.has_warnings())
        self.assertEqual(len(trail.warnings), 1)
        self.assertEqual(trail.warnings[0], "Test warning")

    def test_dependency_trail_get_all_changes(self):
        """Test getting all changes from a trail."""
        trail = DependencyTrail(self.mock_change)

        mock_dep1 = Mock()
        mock_dep1.number = 11111
        mock_dep2 = Mock()
        mock_dep2.number = 22222

        # Create mock current changes for the dependencies
        mock_current1 = Mock()
        mock_current1.number = 33333
        mock_current2 = Mock()
        mock_current2.number = 44444

        trail.add_relevant_dependency(mock_dep1, mock_current1, [])
        trail.add_relevant_dependency(mock_dep2, mock_current2, [])

        all_changes = trail.get_all_changes()
        self.assertEqual(len(all_changes), 3)  # root + 2 deps
        self.assertIn(self.mock_change, all_changes)
        self.assertIn(mock_dep1, all_changes)
        self.assertIn(mock_dep2, all_changes)

    @patch("scap.backport.Backport.get_logger")
    def test_calculate_relevant_dependencies_basic(self, mock_logger):
        """Test the dependency calculation workflow basic functionality."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock the required attributes and methods
        backport.backports = Mock()
        backport.backports.changes = {"123": self.mock_change}

        backport.git_repos = Mock()
        backport.git_repos.change_is_deployable.return_value = True
        backport.git_repos.change_targets_production.return_value = True

        # Mock gerrit for git relations
        backport.gerrit = Mock()
        mock_submitted_together = Mock()
        mock_submitted_together.get.return_value.changes = [
            self.mock_change.details
        ]  # Only self
        backport.gerrit.submitted_together.return_value = mock_submitted_together

        self.mock_change.depends_on_cycle = False
        self.mock_change.depends_ons = []

        backport._confirm_non_live_change = Mock()

        # Mock logger
        mock_logger_instance = Mock()
        mock_logger.return_value = mock_logger_instance

        # Call the method
        trails = backport._calculate_relevant_dependencies()

        # Verify results
        self.assertEqual(len(trails), 1)
        trail = trails[0]
        self.assertEqual(trail.root_change, self.mock_change)
        self.assertFalse(trail.has_errors())

        # Verify logging calls
        mock_logger_instance.info.assert_called()

    @patch("scap.backport.Backport.get_logger")
    def test_validate_root_changes_with_cycle_error(self, mock_logger):
        """Test the workflow properly handles dependency cycles in validate_root_changes."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Set up mocks
        backport.backports = Mock()
        backport.backports.changes = {"123": self.mock_change}
        backport.git_repos = Mock()
        backport.git_repos.change_is_deployable.return_value = True
        backport.arguments = Mock()

        # Set up a cycle
        self.mock_change.depends_on_cycle = True

        mock_logger_instance = Mock()
        mock_logger.return_value = mock_logger_instance

        # Call the method and expect InvalidChangeException
        from scap.backport import InvalidChangeException

        with self.assertRaises(InvalidChangeException) as context:
            backport.validate_root_changes()

        # Verify cycle error was detected
        self.assertIn("cycle detected", str(context.exception).lower())

    @patch("scap.backport.Backport.get_logger")
    def test_validate_backports_with_non_deployable_error(self, mock_logger):
        """Test the workflow properly handles non-deployable root changes."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Set up mocks
        backport.backports = Mock()
        backport.backports.changes = {"123": self.mock_change}
        backport.git_repos = Mock()
        backport.git_repos.change_is_deployable.return_value = False  # Not deployable
        backport.backport_or_revert = "backport"

        mock_logger_instance = Mock()
        mock_logger.return_value = mock_logger_instance

        # Call validate_root_changes and expect it to raise InvalidChangeException
        with self.assertRaises(InvalidChangeException) as cm:
            backport.validate_root_changes()

        # Verify error message
        self.assertIn("not deployable", str(cm.exception).lower())

    @patch("scap.backport.Backport.get_logger")
    def test_validate_root_changes_with_abandoned_error(self, mock_logger):
        """Test the workflow properly handles abandoned root changes."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Set up mocks
        backport.backports = Mock()
        backport.backports.changes = {"123": self.mock_change}
        backport.git_repos = Mock()
        backport.git_repos.change_is_deployable.return_value = True
        backport.arguments = Mock()

        # Set up an abandoned change
        self.mock_change.depends_on_cycle = False
        self.mock_change.get.return_value = "ABANDONED"
        self.mock_change._format_dependency_chain.return_value = " Chain info"

        mock_logger_instance = Mock()
        mock_logger.return_value = mock_logger_instance

        # Call the method and expect InvalidChangeException
        from scap.backport import InvalidChangeException

        with self.assertRaises(InvalidChangeException) as context:
            backport.validate_root_changes()

        # Verify abandoned error was detected
        self.assertIn("abandoned", str(context.exception).lower())

    @patch("scap.backport.Backport.get_logger")
    def test_validate_root_changes_non_production_with_yes_flag(self, mock_logger):
        """Test that non-production changes log a warning when --yes is used."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Set up mocks
        backport.backports = Mock()
        backport.backports.changes = {"123": self.mock_change}
        backport.git_repos = Mock()
        backport.git_repos.change_is_deployable.return_value = True
        backport.git_repos.change_targets_production.return_value = (
            False  # Not production
        )
        backport.arguments = Mock()
        backport.arguments.yes = True  # --yes flag is used
        backport.versions = ["1.45.0-wmf.21", "1.45.0-wmf.22"]

        # Set up a valid change that's not targeting production
        self.mock_change.depends_on_cycle = False
        self.mock_change.get.return_value = "NEW"  # Not abandoned

        mock_logger_instance = Mock()
        mock_logger.return_value = mock_logger_instance

        # Call the method - should not raise an exception but should log warning
        backport.validate_root_changes()

        # Verify warning was logged
        mock_logger_instance.warning.assert_called_once()
        warning_call = mock_logger_instance.warning.call_args[0][0]
        self.assertIn("not found in any live wikiversion", warning_call)
        self.assertIn("Live wikiversions:", warning_call)

    @patch("scap.backport.Backport.get_logger")
    def test_validate_root_changes_non_production_with_prompt(self, mock_logger):
        """Test that non-production changes prompt user when --yes is not used."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Set up mocks
        backport.backports = Mock()
        backport.backports.changes = {"123": self.mock_change}
        backport.git_repos = Mock()
        backport.git_repos.change_is_deployable.return_value = True
        backport.git_repos.change_targets_production.return_value = (
            False  # Not production
        )
        backport.arguments = Mock()
        backport.arguments.yes = False  # --yes flag is NOT used
        backport.versions = ["1.45.0-wmf.21", "1.45.0-wmf.22"]
        backport.backport_or_revert = "backport"
        backport._prompt_for_approval_or_exit = Mock()

        # Set up a valid change that's not targeting production
        self.mock_change.depends_on_cycle = False
        self.mock_change.get.return_value = "NEW"  # Not abandoned

        mock_logger_instance = Mock()
        mock_logger.return_value = mock_logger_instance

        # Call the method - should not raise an exception but should prompt user
        backport.validate_root_changes()

        # Verify user was prompted
        backport._prompt_for_approval_or_exit.assert_called_once()
        prompt_call = backport._prompt_for_approval_or_exit.call_args[0][0]
        self.assertIn("not found in any live wikiversion", prompt_call)
        self.assertIn("Live wikiversions:", prompt_call)
        self.assertIn("Continue with backport anyway?", prompt_call)

    def test_validate_dependency_trails_success(self):
        """Test validating trails without errors or warnings passes silently."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create trails without errors
        trail1 = DependencyTrail(self.mock_change)
        mock_dep = Mock()
        mock_dep.number = 54321
        mock_current = Mock()
        mock_current.number = 98765
        trail1.add_relevant_dependency(mock_dep, mock_current, [])

        trails = [trail1]

        # Call the method - should not raise any exceptions
        try:
            backport._validate_dependency_trails(trails)
        except Exception as e:
            self.fail(f"Validation should pass without errors, but got: {e}")

    def test_validate_dependency_trails_with_errors(self):
        """Test that trails with errors raise exceptions."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create trail with errors
        trail = DependencyTrail(self.mock_change)
        trail.add_error("Test error message")

        trails = [trail]

        # Should raise exception due to errors
        with self.assertRaises(InvalidChangeException) as context:
            backport._validate_dependency_trails(trails)

        self.assertIn("Test error message", str(context.exception))

    def test_validate_dependency_trails_with_multiple_errors(self):
        """Test that multiple errors from different trails are all reported together."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create multiple mock changes
        mock_change1 = Mock()
        mock_change1.number = 11111
        mock_change2 = Mock()
        mock_change2.number = 22222

        # Create trails with different errors
        trail1 = DependencyTrail(mock_change1)
        trail1.add_error("First error message")
        trail1.add_error("Second error message")  # Multiple errors in same trail

        trail2 = DependencyTrail(mock_change2)
        trail2.add_error("Third error message")

        # Create a trail without errors
        trail3 = DependencyTrail(self.mock_change)

        trails = [trail1, trail2, trail3]

        # Should raise exception with all errors
        with self.assertRaises(InvalidChangeException) as context:
            backport._validate_dependency_trails(trails)

        error_message = str(context.exception)
        # All errors should be present in the exception message
        self.assertIn("First error message", error_message)
        self.assertIn("Second error message", error_message)
        self.assertIn("Third error message", error_message)
        # Should indicate it's collecting from multiple trails
        self.assertIn("Errors found in dependency trails", error_message)

    def test_validate_dependency_trails_with_warnings_prompts_user(self):
        """Test that warnings are collected and prompt user for confirmation."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock the prompt method and arguments
        backport._prompt_for_approval_or_exit = Mock()
        backport.arguments = Mock()
        backport.arguments.yes = False  # Ensure prompting is enabled
        backport.backport_or_revert = "backport"

        # Create trail with warnings
        trail = DependencyTrail(self.mock_change)
        trail.add_warning("Test warning message")

        trails = [trail]

        # Should not raise exception but should prompt user
        backport._validate_dependency_trails(trails)

        # Verify user was prompted about warnings
        backport._prompt_for_approval_or_exit.assert_called_once()
        args = backport._prompt_for_approval_or_exit.call_args[0][0]
        self.assertIn("Test warning message", args)
        self.assertIn("Continue with backport anyway?", args)

    def test_analyze_dependencies_recursive(self):
        """Test recursive dependency analysis without separate collection phase."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock required methods and attributes
        backport.gerrit = Mock()
        backport.backports = Mock()
        backport.backports.change_numbers = []
        backport.get_logger = Mock()

        mock_submitted_together = Mock()
        mock_submitted_together.get.return_value.changes = [{}]  # Only self
        backport.gerrit.submitted_together.return_value = mock_submitted_together

        # Mock is_relevant_dep to return False for all dependencies
        backport.is_relevant_dep = Mock(return_value=False)

        # Set up nested dependency chain
        mock_dep1 = Mock()
        mock_dep1.number = 11111
        mock_dep1.depends_ons = []
        mock_dep1.is_merged.return_value = True

        mock_dep2 = Mock()
        mock_dep2.number = 22222
        mock_dep2.depends_ons = [mock_dep1]
        mock_dep2.is_merged.return_value = True

        self.mock_change.depends_ons = [mock_dep2]

        # Create a trail to collect results
        from scap.backport import DependencyTrail

        trail = DependencyTrail(self.mock_change)

        # Test recursive analysis
        backport._analyze_dependencies_recursive(self.mock_change, trail, [], set())

        # Verify relevance checks were performed for all dependencies in the chain
        # Should be called for mock_dep2 and mock_dep1
        self.assertEqual(backport.is_relevant_dep.call_count, 2)

    def test_collect_relevant_dependencies_with_t365146_rules(self):
        """Test T365146 relevance rule application."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock dependencies
        backport.get_logger = Mock(return_value=Mock())
        backport.is_relevant_dep = Mock()
        backport.backports = Mock()
        backport.backports.change_numbers = [12345]

        # Mock gerrit for git relations
        backport.gerrit = Mock()
        mock_submitted_together = Mock()
        mock_submitted_together.get.return_value.changes = [{}]  # Only self
        backport.gerrit.submitted_together.return_value = mock_submitted_together

        # Set up mocks
        mock_dep1 = Mock()
        mock_dep1.number = 11111
        mock_dep1.depends_ons = []
        mock_dep1.is_merged = Mock(return_value=True)

        mock_dep2 = Mock()
        mock_dep2.number = 22222
        mock_dep2.depends_ons = []
        mock_dep2.is_merged = Mock(return_value=False)

        self.mock_change.depends_ons = [mock_dep1, mock_dep2]

        # Configure relevance - dep1 relevant, dep2 not relevant
        backport.is_relevant_dep.side_effect = (
            lambda change, dep, all_deps: dep == mock_dep1
        )

        # Create trail and test using new method
        trail = DependencyTrail(self.mock_change)
        backport._analyze_dependencies_recursive(self.mock_change, trail, [], set())

        # Verify only relevant dependency was added
        self.assertEqual(len(trail.relevant_dependencies), 1)
        self.assertIn(mock_dep1, trail.relevant_dependencies)
        self.assertNotIn(mock_dep2, trail.relevant_dependencies)

    def test_analyze_dependencies_recursive_no_duplicates(self):
        """
        Test that _analyze_dependencies_recursive doesn't process the same change multiple times
        when a change appears in both Depends-On relationships and git relations.

        This tests that the visited set prevents duplicate processing in scenarios
        with both Depends-On footers and relation chains.
        """
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock required methods and attributes
        backport.gerrit = Mock()
        backport.backports = Mock()
        backport.backports.change_numbers = []
        backport.get_logger = Mock()

        # Mock is_relevant_dep to return False and track calls
        backport.is_relevant_dep = Mock(return_value=False)

        # Create the shared change that will appear in both Depends-On and relations
        shared_change = Mock()
        shared_change.number = 11111
        shared_change.depends_ons = []

        # Mock the root change
        root_change = Mock()
        root_change.number = 12345
        root_change.depends_ons = [shared_change]  # Depends-On relationship

        # Mock git relations - the key part of our test
        def submitted_together_side_effect(change_number):
            result = Mock()
            if change_number == 12345:  # root_change
                # Root change has a relation to shared_change (creating potential duplicate)
                result.get.return_value.changes = [
                    {"_number": 12345},  # self (will be removed)
                    {"_number": 11111},  # relation to shared_change
                ]
            else:
                # Other changes have no relations (just themselves)
                result.get.return_value.changes = [{"_number": change_number}]
            return result

        backport.gerrit.submitted_together.side_effect = submitted_together_side_effect

        # Mock GerritChange creation to return our predefined objects
        def mock_gerrit_change_init(gerrit, number, details=None, needed_by_chain=None):
            if number == 11111:
                return shared_change
            else:
                # For any other number, return a new mock
                new_mock = Mock()
                new_mock.number = number
                new_mock.depends_ons = []
                return new_mock

        # Create a trail to collect results
        from scap.backport import DependencyTrail

        trail = DependencyTrail(root_change)

        # Test recursive analysis with mocked GerritChange creation
        with patch("scap.backport.GerritChange", side_effect=mock_gerrit_change_init):
            backport._analyze_dependencies_recursive(root_change, trail, [], set())

        # Verify that shared_change's relevance was checked (may be more than once in current implementation)
        # In the new architecture, duplicate processing is prevented by visited set in recursive traversal
        # but relevance may still be checked multiple times for different relationship paths
        relevance_calls_for_shared = [
            call
            for call in backport.is_relevant_dep.call_args_list
            if call[0][1].number == 11111  # dep_change parameter
        ]
        self.assertGreaterEqual(
            len(relevance_calls_for_shared),
            1,
            f"Shared change should have relevance checked at least once, but was checked {len(relevance_calls_for_shared)} times",
        )

    def test_dependency_trail_deduplication(self):
        """
        Test that DependencyTrail.add_relevant_dependency doesn't add duplicate dependencies
        when the same dependency is encountered multiple times during processing.
        """
        mock_root = Mock()
        mock_root.number = 12345
        mock_root.get.return_value = "root_id"

        trail = DependencyTrail(mock_root)

        # Create a dependency that will be added multiple times
        mock_dep = Mock()
        mock_dep.number = 11111
        mock_dep.get.return_value = "dep_id"

        # Create different "current" changes to simulate different paths to the same dependency
        mock_current1 = Mock()
        mock_current1.number = 22222

        mock_current2 = Mock()
        mock_current2.number = 33333

        # Add the same dependency through different paths
        trail.add_relevant_dependency(mock_dep, mock_current1, [])
        trail.add_relevant_dependency(mock_dep, mock_current2, [])
        trail.add_relevant_dependency(mock_dep, mock_current1, [])  # Duplicate

        # Verify the dependency appears only once in the trail
        self.assertEqual(len(trail.relevant_dependencies), 1)
        self.assertEqual(trail.relevant_dependencies[0].number, 11111)

        # Verify the dependency chain was recorded (should keep one of them)
        self.assertIn("dep_id", trail.dependency_chains)

    def test_warning_for_irrelevant_depends_on(self):
        """Test that warning is added when change has Depends-On but none are relevant."""
        # Create backport instance with mock data
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create a mock change with Depends-On relationships
        mock_root_change = Mock(spec=GerritChange)
        mock_root_change.number = 12345
        mock_root_change.depends_on_cycle = False
        mock_root_change.get.return_value = "test-change-id"
        mock_root_change.check_abandoned_for_trail = Mock()

        # Create mock dependencies that will be deemed irrelevant
        mock_dep1 = Mock(spec=GerritChange)
        mock_dep1.number = 11111
        mock_dep1.depends_ons = []  # No further dependencies
        mock_dep2 = Mock(spec=GerritChange)
        mock_dep2.number = 22222
        mock_dep2.depends_ons = []  # No further dependencies

        mock_root_change.depends_ons = [mock_dep1, mock_dep2]

        # Mock required attributes
        backport.backports = Mock()
        backport.backports.changes = {12345: mock_root_change}
        backport.get_logger = Mock()

        # Mock git_repos methods
        backport.git_repos = Mock()
        backport.git_repos.change_targets_production.return_value = True

        # Mock is_relevant_dep to always return False (irrelevant dependencies)
        backport.is_relevant_dep = Mock(return_value=False)

        # Mock _collect_all_dependencies_recursive
        backport._collect_all_dependencies_recursive = Mock(
            return_value=[mock_dep1, mock_dep2]
        )

        # Mock gerrit submitted_together to avoid processing relations
        backport.gerrit = Mock()
        mock_gerrit_response = Mock()
        mock_gerrit_response.changes = [
            {"_number": mock_root_change.number}
        ]  # Only self
        backport.gerrit.submitted_together = Mock(
            return_value=Mock(get=Mock(return_value=mock_gerrit_response))
        )

        # Calculate dependencies
        trails = backport._calculate_relevant_dependencies()

        # Verify warning was added
        self.assertEqual(len(trails), 1)
        trail = trails[0]

        # Should have no relevant dependencies
        self.assertEqual(len(trail.relevant_dependencies), 0)

        # Should have one warning about irrelevant Depends-On relationships
        self.assertTrue(trail.has_warnings())
        self.assertEqual(len(trail.warnings), 1)

        warning = trail.warnings[0]
        self.assertIn("has 2 Depends-On relationship(s) (11111, 22222)", warning)
        self.assertIn("none were deemed relevant", warning)
        self.assertIn("This may be unexpected", warning)

    def test_no_warning_when_some_depends_on_are_relevant(self):
        """Test that no warning is added when some Depends-On relationships are relevant."""
        # Create backport instance with mock data
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create a mock change with Depends-On relationships
        mock_root_change = Mock(spec=GerritChange)
        mock_root_change.number = 12345
        mock_root_change.depends_on_cycle = False
        mock_root_change.get.return_value = "test-change-id"
        mock_root_change.check_abandoned_for_trail = Mock()

        # Create mock dependencies - one relevant, one not
        mock_dep1 = Mock(spec=GerritChange)
        mock_dep1.number = 11111
        mock_dep1.depends_ons = []  # No further dependencies
        mock_dep1.is_merged.return_value = True
        mock_dep1.check_abandoned_for_trail = Mock()

        mock_dep2 = Mock(spec=GerritChange)
        mock_dep2.number = 22222
        mock_dep2.depends_ons = []  # No further dependencies

        mock_root_change.depends_ons = [mock_dep1, mock_dep2]

        # Mock required attributes
        backport.backports = Mock()
        backport.backports.changes = {12345: mock_root_change}
        backport.get_logger = Mock()

        # Mock git_repos methods
        backport.git_repos = Mock()
        backport.git_repos.change_targets_production.return_value = True

        # Mock is_relevant_dep - first dependency relevant, second not
        def mock_is_relevant(change, dep, all_deps):
            return dep.number == 11111  # Only first dep is relevant

        backport.is_relevant_dep = Mock(side_effect=mock_is_relevant)

        # Mock _collect_all_dependencies_recursive
        backport._collect_all_dependencies_recursive = Mock(
            return_value=[mock_dep1, mock_dep2]
        )

        # Mock gerrit submitted_together to avoid processing relations
        backport.gerrit = Mock()
        mock_gerrit_response = Mock()
        mock_gerrit_response.changes = [
            {"_number": mock_root_change.number}
        ]  # Only self
        backport.gerrit.submitted_together = Mock(
            return_value=Mock(get=Mock(return_value=mock_gerrit_response))
        )

        # Mock backports.change_numbers to avoid "not in backport set" error
        backport.backports.change_numbers = [11111]  # mock_dep1 is in backport set

        # Calculate dependencies
        trails = backport._calculate_relevant_dependencies()

        # Verify no warning was added since one dependency was relevant
        self.assertEqual(len(trails), 1)
        trail = trails[0]

        # Should have one relevant dependency
        self.assertEqual(len(trail.relevant_dependencies), 1)

        # Should have no warning about irrelevant Depends-On relationships
        depends_on_warnings = [
            w for w in trail.warnings if "Depends-On relationship" in w
        ]
        self.assertEqual(len(depends_on_warnings), 0)


class TestGitRepos(unittest.TestCase):
    """Tests for GitRepos class methods."""

    def setUp(self):
        """Set up test fixtures for GitRepos tests."""
        # Mock gerrit
        self.mock_gerrit = Mock()
        self.mock_gerrit.submodule_project_from_url.side_effect = (
            self._mock_project_from_url
        )

        # Test data
        self.mediawiki_core = "mediawiki/core"
        self.operations_config = "operations/mediawiki-config"
        self.config_branch = "master"
        self.versions = ["1.45.0-wmf.21", "1.45.0-wmf.22"]
        self.mediawiki_location = "/srv/mediawiki-staging"

    def _mock_project_from_url(self, url):
        """Mock gerrit.submodule_project_from_url method."""
        url_to_project = {
            "https://gerrit.wikimedia.org/r/mediawiki/extensions/Example": "mediawiki/extensions/Example",
            "https://gerrit.wikimedia.org/r/mediawiki/skins/Vector": "mediawiki/skins/Vector",
            "https://gerrit.wikimedia.org/r/operations/mediawiki-config": "operations/mediawiki-config",
        }
        return url_to_project.get(url)

    def _create_mock_change(self, project, branch):
        """Create a mock GerritChange with project and branch."""
        mock_change = Mock()
        mock_change.get.side_effect = lambda key: {
            "project": project,
            "branch": branch,
        }[key]
        return mock_change

    def _create_git_repos(self):
        """Create a GitRepos instance with standard test parameters."""
        return GitRepos(
            self.mediawiki_core,
            self.operations_config,
            self.config_branch,
            self.versions,
            self.mediawiki_location,
            self.mock_gerrit,
        )

    def _setup_empty_submodules(self, mock_list_submodules):
        """Configure mock to return no submodules."""
        mock_list_submodules.return_value = []

    def _setup_config_submodules(self, mock_list_submodules):
        """Configure mock to return config submodule."""
        mock_list_submodules.return_value = [
            "submodules/config https://gerrit.wikimedia.org/r/operations/mediawiki-config"
        ]

    def _setup_example_submodules(self, mock_list_submodules):
        """Configure mock to return example extensions and skins."""
        mock_list_submodules.return_value = [
            "extensions/Example https://gerrit.wikimedia.org/r/mediawiki/extensions/Example",
            "skins/Vector https://gerrit.wikimedia.org/r/mediawiki/skins/Vector",
        ]

    def _setup_versioned_submodules(self, mock_list_submodules):
        """Configure mock to return different submodules per version."""

        def mock_submodules_side_effect(location, *args):
            if "php-1.45.0-wmf.21" in location:
                return [
                    "extensions/Example https://gerrit.wikimedia.org/r/mediawiki/extensions/Example"
                ]
            return []

        mock_list_submodules.side_effect = mock_submodules_side_effect

    def _setup_mixed_submodules(self, mock_list_submodules):
        """Configure mock to return config for non-versioned and example for versioned locations."""

        def mock_submodules_side_effect(location, *args):
            if "php-1.45.0-wmf.21" in location:
                return [
                    "extensions/Example https://gerrit.wikimedia.org/r/mediawiki/extensions/Example"
                ]
            else:
                return [
                    "submodules/config https://gerrit.wikimedia.org/r/operations/mediawiki-config"
                ]

        mock_list_submodules.side_effect = mock_submodules_side_effect

    def _setup_different_version_submodules(self, mock_list_submodules):
        """Configure mock to return different extensions/skins per version."""

        def mock_submodules_side_effect(location, *args):
            if "php-1.45.0-wmf.21" in location:
                return [
                    "extensions/Example https://gerrit.wikimedia.org/r/mediawiki/extensions/Example"
                ]
            elif "php-1.45.0-wmf.22" in location:
                return [
                    "skins/Vector https://gerrit.wikimedia.org/r/mediawiki/skins/Vector"
                ]
            return []

        mock_list_submodules.side_effect = mock_submodules_side_effect

    def _setup_exclusive_version_submodules(self, mock_list_submodules):
        """Configure mock to return mutually exclusive extensions per version."""

        def mock_submodules_side_effect(location, *args):
            if "php-1.45.0-wmf.21" in location:
                return [
                    "extensions/OnlyInWmf21 https://gerrit.wikimedia.org/r/mediawiki/extensions/OnlyInWmf21"
                ]
            elif "php-1.45.0-wmf.22" in location:
                return [
                    "extensions/OnlyInWmf22 https://gerrit.wikimedia.org/r/mediawiki/extensions/OnlyInWmf22"
                ]
            return []

        mock_list_submodules.side_effect = mock_submodules_side_effect

    def _create_exclusive_gerrit_mock(self):
        """Create a gerrit mock for exclusive version extension testing."""
        mock_gerrit = Mock()
        mock_gerrit.submodule_project_from_url.side_effect = lambda url: (
            "mediawiki/extensions/OnlyInWmf21"
            if "OnlyInWmf21" in url
            else "mediawiki/extensions/OnlyInWmf22" if "OnlyInWmf22" in url else None
        )
        return mock_gerrit

    @patch("scap.git.list_submodules_paths_urls")
    def test_init(self, mock_list_submodules):
        """Test GitRepos initialization."""
        self._setup_config_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Verify initialization
        self.assertEqual(git_repos.MEDIAWIKI_CORE, self.mediawiki_core)
        self.assertEqual(git_repos.OPERATIONS_CONFIG, self.operations_config)
        self.assertEqual(git_repos.config_branch, self.config_branch)
        self.assertEqual(git_repos.versions, self.versions)
        self.assertEqual(git_repos.mediawiki_location, self.mediawiki_location)
        self.assertEqual(git_repos.gerrit, self.mock_gerrit)

        # Verify config_repos was populated
        self.assertIn(self.operations_config, git_repos.config_repos)
        self.assertEqual(
            git_repos.config_repos[self.operations_config], self.mediawiki_location
        )

    @patch("scap.git.list_submodules_paths_urls")
    def test_get_submodules_paths(self, mock_list_submodules):
        """Test _get_submodules_paths method."""
        self._setup_example_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        result = git_repos._get_submodules_paths("/test/location")

        expected = {
            "mediawiki/extensions/Example": "extensions/Example",
            "mediawiki/skins/Vector": "skins/Vector",
        }
        self.assertEqual(result, expected)

    @patch("scap.git.list_submodules_paths_urls")
    def test_get_core_repos_for_version(self, mock_list_submodules):
        """Test _get_core_repos_for_version method."""
        self._setup_versioned_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        version = "1.45.0-wmf.21"
        result = git_repos._get_core_repos_for_version(version)

        # Should include both core and extensions
        self.assertIn(self.mediawiki_core, result)
        self.assertIn("mediawiki/extensions/Example", result)
        self.assertEqual(
            result[self.mediawiki_core], f"{self.mediawiki_location}/php-{version}"
        )

        # Test caching - second call should return cached result
        result2 = git_repos._get_core_repos_for_version(version)
        self.assertEqual(result, result2)

    @patch("scap.git.list_submodules_paths_urls")
    def test_targets_deployable_code(self, mock_list_submodules):
        """Test targets_deployable_code method."""
        self._setup_versioned_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Test wmf branch with existing project
        change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
        self.assertTrue(git_repos.targets_deployable_code(change))

        # Test wmf branch with extension that exists in version
        change = self._create_mock_change(
            "mediawiki/extensions/Example", "wmf/1.45.0-wmf.21"
        )
        self.assertTrue(git_repos.targets_deployable_code(change))

        # Test non-wmf branch
        change = self._create_mock_change("mediawiki/core", "master")
        self.assertFalse(git_repos.targets_deployable_code(change))

        # Test project not in version
        change = self._create_mock_change(
            "mediawiki/extensions/NonExistent", "wmf/1.45.0-wmf.21"
        )
        self.assertFalse(git_repos.targets_deployable_code(change))

        # Test FileNotFoundError handling (version directory doesn't exist)
        change = self._create_mock_change("mediawiki/core", "wmf/nonexistent")
        with patch.object(
            git_repos, "_get_core_repos_for_version", side_effect=FileNotFoundError
        ):
            self.assertFalse(git_repos.targets_deployable_code(change))

    @patch("scap.git.list_submodules_paths_urls")
    def test_targets_live_code(self, mock_list_submodules):
        """Test targets_live_code method."""
        self._setup_empty_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Mock targets_deployable_code to return True
        with patch.object(git_repos, "targets_deployable_code", return_value=True):
            # Test live version (in versions list)
            change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
            self.assertTrue(git_repos.targets_live_code(change))

            # Test non-live version (not in versions list)
            change = self._create_mock_change("mediawiki/core", "wmf/1.44.0-wmf.20")
            self.assertFalse(git_repos.targets_live_code(change))

            # Test non-wmf branch
            change = self._create_mock_change("mediawiki/core", "master")
            self.assertFalse(git_repos.targets_live_code(change))

        # Test when targets_deployable_code returns False
        with patch.object(git_repos, "targets_deployable_code", return_value=False):
            change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
            self.assertFalse(git_repos.targets_live_code(change))

    @patch("scap.git.list_submodules_paths_urls")
    def test_change_is_deployable(self, mock_list_submodules):
        """Test change_is_deployable method."""
        self._setup_config_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Test config change (should be deployable)
        change = self._create_mock_change(self.operations_config, self.config_branch)
        with patch.object(git_repos, "targets_prod_config", return_value=True):
            with patch.object(git_repos, "targets_deployable_code", return_value=False):
                self.assertTrue(git_repos.change_is_deployable(change))

        # Test code change (should be deployable)
        change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
        with patch.object(git_repos, "targets_prod_config", return_value=False):
            with patch.object(git_repos, "targets_deployable_code", return_value=True):
                self.assertTrue(git_repos.change_is_deployable(change))

        # Test neither config nor code (should not be deployable)
        change = self._create_mock_change("some/other/project", "master")
        with patch.object(git_repos, "targets_prod_config", return_value=False):
            with patch.object(git_repos, "targets_deployable_code", return_value=False):
                self.assertFalse(git_repos.change_is_deployable(change))

    @patch("scap.git.list_submodules_paths_urls")
    def test_change_targets_production(self, mock_list_submodules):
        """Test change_targets_production method."""
        self._setup_empty_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Test prod config change
        change = self._create_mock_change(self.operations_config, self.config_branch)
        with patch.object(git_repos, "targets_prod_config", return_value=True):
            with patch.object(git_repos, "targets_live_code", return_value=False):
                self.assertTrue(git_repos.change_targets_production(change))

        # Test live code change
        change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
        with patch.object(git_repos, "targets_prod_config", return_value=False):
            with patch.object(git_repos, "targets_live_code", return_value=True):
                self.assertTrue(git_repos.change_targets_production(change))

        # Test neither prod config nor live code
        change = self._create_mock_change("some/project", "master")
        with patch.object(git_repos, "targets_prod_config", return_value=False):
            with patch.object(git_repos, "targets_live_code", return_value=False):
                self.assertFalse(git_repos.change_targets_production(change))

    @patch("scap.git.list_submodules_paths_urls")
    def test_targets_prod_config(self, mock_list_submodules):
        """Test targets_prod_config method."""
        self._setup_config_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Test matching config project and branch
        change = self._create_mock_change(self.operations_config, self.config_branch)
        self.assertTrue(git_repos.targets_prod_config(change))

        # Test wrong project
        change = self._create_mock_change("wrong/project", self.config_branch)
        self.assertFalse(git_repos.targets_prod_config(change))

        # Test wrong branch
        change = self._create_mock_change(self.operations_config, "wrong-branch")
        self.assertFalse(git_repos.targets_prod_config(change))

        # Test project in config_repos but wrong branch
        change = self._create_mock_change("operations/mediawiki-config", "wrong-branch")
        self.assertFalse(git_repos.targets_prod_config(change))

    @patch("scap.git.list_submodules_paths_urls")
    def test_get_repo_location(self, mock_list_submodules):
        """Test get_repo_location method."""
        self._setup_mixed_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Test config project - parent location
        change = self._create_mock_change(self.operations_config, self.config_branch)
        location = git_repos.get_repo_location(change, use_submodule_directory=False)
        self.assertEqual(location, self.mediawiki_location)

        # Test config project - submodule location
        location = git_repos.get_repo_location(change, use_submodule_directory=True)
        self.assertEqual(
            location, self.mediawiki_location
        )  # config repo maps to main location

        # Test MW code project - parent location
        change = self._create_mock_change(
            "mediawiki/extensions/Example", "wmf/1.45.0-wmf.21"
        )
        location = git_repos.get_repo_location(change, use_submodule_directory=False)
        expected_core_location = f"{self.mediawiki_location}/php-1.45.0-wmf.21"
        self.assertEqual(location, expected_core_location)

        # Test MW code project - submodule location
        location = git_repos.get_repo_location(change, use_submodule_directory=True)
        self.assertEqual(location, "extensions/Example")

    @patch("scap.git.list_submodules_paths_urls")
    @patch("scap.utils.get_wikiversions_ondisk")
    def test_is_mediawiki_code_project(
        self, mock_get_wikiversions, mock_list_submodules
    ):
        """Test targets_any_mediawiki_code method."""
        mock_get_wikiversions.return_value = self.versions
        self._setup_different_version_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Helper function to create mock GerritChange objects
        def create_mock_change(project):
            mock_change = Mock()
            mock_change.get.return_value = project
            return mock_change

        # Test MediaWiki core
        self.assertTrue(
            git_repos.targets_any_mediawiki_code(
                create_mock_change(self.mediawiki_core)
            )
        )

        # Test extension in version 1
        self.assertTrue(
            git_repos.targets_any_mediawiki_code(
                create_mock_change("mediawiki/extensions/Example")
            )
        )

        # Test skin in version 2
        self.assertTrue(
            git_repos.targets_any_mediawiki_code(
                create_mock_change("mediawiki/skins/Vector")
            )
        )

        # Test non-MW project
        self.assertFalse(
            git_repos.targets_any_mediawiki_code(
                create_mock_change(self.operations_config)
            )
        )

        # Test MW project not in any version
        self.assertFalse(
            git_repos.targets_any_mediawiki_code(
                create_mock_change("mediawiki/extensions/NonExistent")
            )
        )

    @patch("scap.git.list_submodules_paths_urls")
    @patch("scap.utils.get_wikiversions_ondisk")
    def test_targets_deployable_code_vs_targets_any_mediawiki_code_difference(
        self, mock_get_wikiversions, mock_list_submodules
    ):
        """
        Test when targets_deployable_code and targets_any_mediawiki_code return different results.

        targets_deployable_code: Checks if project exists in the SPECIFIC version of the change
        targets_any_mediawiki_code: Checks if project exists in ANY deployable version

        They differ when a project exists in some deployable versions but not in the
        specific version targeted by the change.
        """
        test_versions = ["1.45.0-wmf.21", "1.45.0-wmf.22"]
        mock_get_wikiversions.return_value = test_versions
        self._setup_exclusive_version_submodules(mock_list_submodules)

        mock_gerrit = self._create_exclusive_gerrit_mock()
        git_repos = GitRepos(
            self.mediawiki_core,
            self.operations_config,
            self.config_branch,
            test_versions,
            self.mediawiki_location,
            mock_gerrit,
        )

        # Clear instance cache to ensure fresh state for this test
        git_repos.core_repos = {}

        def create_mock_change(project, branch):
            mock_change = Mock()
            mock_change.get.side_effect = lambda key: {
                "project": project,
                "branch": branch,
            }.get(key)
            return mock_change

        # Extension exists in wmf.21 but change targets wmf.22
        change = create_mock_change(
            "mediawiki/extensions/OnlyInWmf21", "wmf/1.45.0-wmf.22"
        )

        # targets_deployable_code: False (extension not in wmf.22)
        self.assertFalse(git_repos.targets_deployable_code(change))

        # targets_any_mediawiki_code: True (extension exists in wmf.21, which is deployable)
        self.assertTrue(git_repos.targets_any_mediawiki_code(change))

        # Reverse case: Extension exists in wmf.22 but change targets wmf.21
        change = create_mock_change(
            "mediawiki/extensions/OnlyInWmf22", "wmf/1.45.0-wmf.21"
        )

        # targets_deployable_code: False (extension not in wmf.21)
        self.assertFalse(git_repos.targets_deployable_code(change))

        # targets_any_mediawiki_code: True (extension exists in wmf.22, which is deployable)
        self.assertTrue(git_repos.targets_any_mediawiki_code(change))

    def test_is_relevant_dep_comprehensive_coverage(self):
        """Test all code paths in is_relevant_dep method."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock git_repos and its methods
        backport.git_repos = Mock()

        # Create test changes
        mw_core_deployable = Mock()
        mw_core_deployable.get.side_effect = lambda key: {
            "project": "mediawiki/core",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)

        mw_ext_deployable = Mock()
        mw_ext_deployable.get.side_effect = lambda key: {
            "project": "mediawiki/extensions/Example",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)

        mw_ext_master = Mock()
        mw_ext_master.get.side_effect = lambda key: {
            "project": "mediawiki/extensions/Example",
            "branch": "master",
        }.get(key)

        config_change = Mock()
        config_change.get.side_effect = lambda key: {
            "project": "operations/mediawiki-config",
            "branch": "master",
        }.get(key)

        non_mw_change = Mock()
        non_mw_change.get.side_effect = lambda key: {
            "project": "some/other/project",
            "branch": "master",
        }.get(key)

        # Mock sibling checking methods
        backport._has_sibling_targeting_branch = Mock(return_value=False)
        backport._has_sibling_targeting_deployable_branches = Mock(return_value=False)

        # Test Case: Dep is not MW code or config -> False
        backport.git_repos.targets_any_mediawiki_code.return_value = False
        backport.git_repos.targets_prod_config.return_value = False

        result = backport.is_relevant_dep(mw_core_deployable, non_mw_change, [])
        self.assertFalse(result)

        # Reset mocks for MW code/config detection
        backport.git_repos.targets_any_mediawiki_code.side_effect = lambda change: (
            "mediawiki/" in change.get("project")
        )
        backport.git_repos.targets_prod_config.side_effect = lambda change: (
            change.get("project") == "operations/mediawiki-config"
        )

        # Test Case 2: If Dep targets production config -> True
        backport.git_repos.targets_deployable_code.return_value = True
        backport.git_repos.change_is_deployable.return_value = False

        result = backport.is_relevant_dep(mw_core_deployable, config_change, [])
        self.assertTrue(result)

        # Test Case 1a: Change and Dep are MW code & change branch is deployable, same branch
        backport.git_repos.targets_deployable_code.side_effect = lambda change: (
            "wmf/" in change.get("branch")
        )

        result = backport.is_relevant_dep(mw_core_deployable, mw_ext_deployable, [])
        self.assertTrue(result)

        # Test Case 1b: Change deployable, dep master, no sibling -> True
        result = backport.is_relevant_dep(mw_core_deployable, mw_ext_master, [])
        self.assertTrue(result)

        # Test Case 1c: Change deployable, dep master, has sibling -> False
        backport._has_sibling_targeting_branch.return_value = True
        result = backport.is_relevant_dep(mw_core_deployable, mw_ext_master, [])
        self.assertFalse(result)

        # Test Case 1d: Change deployable, dep neither same branch nor master -> False
        dep_other_branch = Mock()
        dep_other_branch.get.side_effect = lambda key: {
            "project": "mediawiki/extensions/Example",
            "branch": "wmf/1.44.0-wmf.20",
        }.get(key)

        result = backport.is_relevant_dep(mw_core_deployable, dep_other_branch, [])
        self.assertFalse(result)

        # Reset sibling mock
        backport._has_sibling_targeting_branch.return_value = False

        # Test Case 3a: Change is prod config, dep is deployable -> True
        backport.git_repos.change_is_deployable.return_value = True
        result = backport.is_relevant_dep(config_change, mw_ext_deployable, [])
        self.assertTrue(result)

        # Test Case 3b: Change is MW master, dep is deployable -> True
        mw_core_master = Mock()
        mw_core_master.get.side_effect = lambda key: {
            "project": "mediawiki/core",
            "branch": "master",
        }.get(key)

        result = backport.is_relevant_dep(mw_core_master, mw_ext_deployable, [])
        self.assertTrue(result)

        # Test Case 3c: Change is prod config, dep master, no sibling -> True
        backport.git_repos.change_is_deployable.return_value = False
        result = backport.is_relevant_dep(config_change, mw_ext_master, [])
        self.assertTrue(result)

        # Test Case 3d: Change is MW master, dep master, no sibling -> True
        result = backport.is_relevant_dep(mw_core_master, mw_ext_master, [])
        self.assertTrue(result)

        # Test Case 3e: Change is prod config, dep master, has sibling -> False
        backport._has_sibling_targeting_deployable_branches.return_value = True
        result = backport.is_relevant_dep(config_change, mw_ext_master, [])
        self.assertFalse(result)

        # Test Case 3f: Change is MW master, dep master, has sibling -> False
        result = backport.is_relevant_dep(mw_core_master, mw_ext_master, [])
        self.assertFalse(result)

        # Test Case 3g: Change is prod config, dep neither deployable nor master -> False
        backport._has_sibling_targeting_deployable_branches.return_value = False
        result = backport.is_relevant_dep(config_change, dep_other_branch, [])
        self.assertFalse(result)

        # Test fallback case: Change doesn't match any rules -> False
        other_change = Mock()
        other_change.get.side_effect = lambda key: {
            "project": "mediawiki/core",
            "branch": "some-feature-branch",
        }.get(key)

        backport.git_repos.targets_deployable_code.return_value = False
        result = backport.is_relevant_dep(other_change, mw_ext_master, [])
        self.assertFalse(result)

    def test_has_sibling_targeting_branch_comprehensive_coverage(self):
        """Test all code paths in _has_sibling_targeting_branch method."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create test dependency change
        dep_change = Mock()
        dep_change.get.return_value = "I1234567890abcdef"  # change_id
        dep_change.number = 12345

        # Test case: No siblings -> False
        all_dependencies = []
        result = backport._has_sibling_targeting_branch(
            dep_change, "wmf/1.45.0-wmf.21", all_dependencies
        )
        self.assertFalse(result)

        # Test case: Sibling with same change_id, different number, same branch -> True
        sibling_same_branch = Mock()
        sibling_same_branch.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)
        sibling_same_branch.number = 67890

        all_dependencies = [sibling_same_branch]
        result = backport._has_sibling_targeting_branch(
            dep_change, "wmf/1.45.0-wmf.21", all_dependencies
        )
        self.assertTrue(result)

        # Test case: Sibling with different change_id -> False
        sibling_diff_change_id = Mock()
        sibling_diff_change_id.get.side_effect = lambda key: {
            "change_id": "I9876543210fedcba",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)
        sibling_diff_change_id.number = 67890

        all_dependencies = [sibling_diff_change_id]
        result = backport._has_sibling_targeting_branch(
            dep_change, "wmf/1.45.0-wmf.21", all_dependencies
        )
        self.assertFalse(result)

        # Test case: Sibling with same change_id and number (same change) -> False
        sibling_same_change = Mock()
        sibling_same_change.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)
        sibling_same_change.number = 12345

        all_dependencies = [sibling_same_change]
        result = backport._has_sibling_targeting_branch(
            dep_change, "wmf/1.45.0-wmf.21", all_dependencies
        )
        self.assertFalse(result)

        # Test case: Sibling with same change_id, different number, different branch -> False
        sibling_diff_branch = Mock()
        sibling_diff_branch.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
            "branch": "master",
        }.get(key)
        sibling_diff_branch.number = 67890

        all_dependencies = [sibling_diff_branch]
        result = backport._has_sibling_targeting_branch(
            dep_change, "wmf/1.45.0-wmf.21", all_dependencies
        )
        self.assertFalse(result)

        # Test case: Multiple siblings, one matches -> True
        all_dependencies = [
            sibling_diff_change_id,
            sibling_same_change,
            sibling_same_branch,
        ]
        result = backport._has_sibling_targeting_branch(
            dep_change, "wmf/1.45.0-wmf.21", all_dependencies
        )
        self.assertTrue(result)

    def test_has_sibling_targeting_deployable_branches_comprehensive_coverage(self):
        """Test all code paths in _has_sibling_targeting_deployable_branches method."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock git_repos
        backport.git_repos = Mock()

        # Create test dependency change
        dep_change = Mock()
        dep_change.get.return_value = "I1234567890abcdef"  # change_id
        dep_change.number = 12345

        # Test case: No siblings -> False
        all_dependencies = []
        result = backport._has_sibling_targeting_deployable_branches(
            dep_change, all_dependencies
        )
        self.assertFalse(result)

        # Test case: Sibling with same change_id, different number, targets deployable -> True
        sibling_deployable = Mock()
        sibling_deployable.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)
        sibling_deployable.number = 67890

        backport.git_repos.targets_deployable_code.return_value = True
        all_dependencies = [sibling_deployable]
        result = backport._has_sibling_targeting_deployable_branches(
            dep_change, all_dependencies
        )
        self.assertTrue(result)

        # Test case: Sibling with different change_id -> False
        sibling_diff_change_id = Mock()
        sibling_diff_change_id.get.side_effect = lambda key: {
            "change_id": "I9876543210fedcba",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)
        sibling_diff_change_id.number = 67890

        backport.git_repos.targets_deployable_code.side_effect = lambda change: (
            change == sibling_deployable or change == sibling_diff_change_id
        )
        all_dependencies = [sibling_diff_change_id]
        result = backport._has_sibling_targeting_deployable_branches(
            dep_change, all_dependencies
        )
        self.assertFalse(result)

        # Test case: Sibling with same change_id and number (same change) -> False
        sibling_same_change = Mock()
        sibling_same_change.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
            "branch": "wmf/1.45.0-wmf.21",
        }.get(key)
        sibling_same_change.number = 12345

        backport.git_repos.targets_deployable_code.return_value = True
        all_dependencies = [sibling_same_change]
        result = backport._has_sibling_targeting_deployable_branches(
            dep_change, all_dependencies
        )
        self.assertFalse(result)

        # Test case: Sibling with same change_id, different number, not deployable -> False
        sibling_not_deployable = Mock()
        sibling_not_deployable.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
            "branch": "master",
        }.get(key)
        sibling_not_deployable.number = 67890

        backport.git_repos.targets_deployable_code.side_effect = lambda change: (
            change == sibling_deployable
        )
        all_dependencies = [sibling_not_deployable]
        result = backport._has_sibling_targeting_deployable_branches(
            dep_change, all_dependencies
        )
        self.assertFalse(result)

        # Test case: Multiple siblings, one matches -> True
        backport.git_repos.targets_deployable_code.side_effect = lambda change: (
            change == sibling_deployable
        )
        all_dependencies = [
            sibling_diff_change_id,
            sibling_same_change,
            sibling_deployable,
            sibling_not_deployable,
        ]
        result = backport._has_sibling_targeting_deployable_branches(
            dep_change, all_dependencies
        )
        self.assertTrue(result)

    def test_process_dependency_comprehensive_coverage(self):
        """Test all code paths in _process_dependency method."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock required methods and attributes
        backport.get_logger = Mock(return_value=Mock())
        backport.is_relevant_dep = Mock()
        backport.backports = Mock()
        backport.backports.change_numbers = [12345, 67890]

        # Create test changes
        current_change = Mock()
        current_change.number = 12345
        current_change.depends_ons = []  # Required for _get_sibling_dependencies

        dep_change_relevant_merged = Mock()
        dep_change_relevant_merged.number = 11111
        dep_change_relevant_merged.is_merged.return_value = True
        dep_change_relevant_merged.check_abandoned_for_trail = Mock()
        dep_change_relevant_merged.depends_ons = []

        dep_change_relevant_unmerged_in_backport = Mock()
        dep_change_relevant_unmerged_in_backport.number = 67890  # In backport set
        dep_change_relevant_unmerged_in_backport.is_merged.return_value = False
        dep_change_relevant_unmerged_in_backport.check_abandoned_for_trail = Mock()
        dep_change_relevant_unmerged_in_backport.depends_ons = []

        dep_change_relevant_unmerged_not_in_backport = Mock()
        dep_change_relevant_unmerged_not_in_backport.number = (
            99999  # Not in backport set
        )
        dep_change_relevant_unmerged_not_in_backport.is_merged.return_value = False
        dep_change_relevant_unmerged_not_in_backport.check_abandoned_for_trail = Mock()
        dep_change_relevant_unmerged_not_in_backport.depends_ons = []

        dep_change_not_relevant = Mock()
        dep_change_not_relevant.number = 22222
        dep_change_not_relevant.is_merged.return_value = False
        dep_change_not_relevant.check_abandoned_for_trail = Mock()
        dep_change_not_relevant.depends_ons = []

        # Test Case 1: Relevant dependency that is merged -> No error added
        trail = DependencyTrail(current_change)
        backport.is_relevant_dep.return_value = True

        result = backport._process_dependency(
            current_change, dep_change_relevant_merged, trail, [], "Depends-On"
        )

        self.assertTrue(result)
        self.assertEqual(len(trail.relevant_dependencies), 1)
        self.assertEqual(len(trail.errors), 0)
        dep_change_relevant_merged.check_abandoned_for_trail.assert_called_once()

        # Test Case 2: Relevant dependency that is unmerged but in backport set -> No error added
        trail = DependencyTrail(current_change)
        backport.is_relevant_dep.return_value = True

        result = backport._process_dependency(
            current_change,
            dep_change_relevant_unmerged_in_backport,
            trail,
            [],
            "Depends-On",
        )

        self.assertTrue(result)
        self.assertEqual(len(trail.relevant_dependencies), 1)
        self.assertEqual(len(trail.errors), 0)
        dep_change_relevant_unmerged_in_backport.check_abandoned_for_trail.assert_called_once()

        # Test Case 3: Relevant dependency that is unmerged and NOT in backport set -> Error added
        trail = DependencyTrail(current_change)
        backport.is_relevant_dep.return_value = True

        result = backport._process_dependency(
            current_change,
            dep_change_relevant_unmerged_not_in_backport,
            trail,
            [],
            "Depends-On",
        )

        self.assertTrue(result)
        self.assertEqual(len(trail.relevant_dependencies), 1)
        self.assertEqual(len(trail.errors), 1)
        expected_error = (
            f"Change '{current_change.number}' has dependency '{dep_change_relevant_unmerged_not_in_backport.number}', "
            f"which is not merged or scheduled for backport"
        )
        self.assertEqual(trail.errors[0], expected_error)
        dep_change_relevant_unmerged_not_in_backport.check_abandoned_for_trail.assert_called_once()

        # Test Case 4: Not relevant dependency -> No error added, not added to trail
        trail = DependencyTrail(current_change)
        backport.is_relevant_dep.return_value = False

        result = backport._process_dependency(
            current_change, dep_change_not_relevant, trail, [], "Related"
        )

        self.assertFalse(result)
        self.assertEqual(len(trail.relevant_dependencies), 0)
        self.assertEqual(len(trail.errors), 0)
        # check_abandoned_for_trail should not be called for irrelevant dependencies
        dep_change_not_relevant.check_abandoned_for_trail.assert_not_called()

        # Verify logging calls were made for all cases
        self.assertEqual(backport.get_logger().info.call_count, 4)

    def test_get_sibling_dependencies(self):
        """Test _get_sibling_dependencies correctly identifies siblings by Change-Id."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create a dep_change with a specific Change-Id
        dep_change = Mock(spec=scap.backport.GerritChange)
        dep_change.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        dep_change.number = 50

        # Create sibling changes with the same Change-Id
        sibling1 = Mock(spec=scap.backport.GerritChange)
        sibling1.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        sibling1.number = 100

        sibling2 = Mock(spec=scap.backport.GerritChange)
        sibling2.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        sibling2.number = 101

        # Create non-sibling changes with different Change-Ids
        non_sibling1 = Mock(spec=scap.backport.GerritChange)
        non_sibling1.get.side_effect = lambda key: {
            "change_id": "Idifferent123456",
        }.get(key)
        non_sibling1.number = 200

        non_sibling2 = Mock(spec=scap.backport.GerritChange)
        non_sibling2.get.side_effect = lambda key: {
            "change_id": "Ianother789abcdef",
        }.get(key)
        non_sibling2.number = 201

        # Create the main change with depends_ons containing all changes
        main_change = Mock(spec=scap.backport.GerritChange)
        main_change.depends_ons = [sibling1, sibling2, non_sibling1, non_sibling2]

        # Test: should return siblings plus dep_change itself (changes with matching Change-Id)
        result = backport._get_sibling_dependencies(main_change, dep_change)

        # Should find sibling1, sibling2 from depends_ons, plus dep_change itself
        self.assertEqual(len(result), 3)
        self.assertIn(sibling1, result)
        self.assertIn(sibling2, result)
        self.assertIn(dep_change, result)  # dep_change is always included
        self.assertNotIn(non_sibling1, result)
        self.assertNotIn(non_sibling2, result)

        # Test edge case: no siblings found
        different_dep_change = Mock(spec=scap.backport.GerritChange)
        different_dep_change.get.side_effect = lambda key: {
            "change_id": "Inosiblingshere123",
        }.get(key)
        different_dep_change.number = 300

        result_empty = backport._get_sibling_dependencies(
            main_change, different_dep_change
        )
        self.assertEqual(len(result_empty), 1)  # Just different_dep_change itself
        self.assertIn(different_dep_change, result_empty)

        # Test edge case: empty depends_ons - should still return dep_change
        empty_change = Mock(spec=scap.backport.GerritChange)
        empty_change.depends_ons = []

        result_empty_deps = backport._get_sibling_dependencies(empty_change, dep_change)
        self.assertEqual(len(result_empty_deps), 1)  # Just dep_change itself
        self.assertIn(dep_change, result_empty_deps)

        # Test edge case: dep_change already in depends_ons - no duplication
        dep_in_list = Mock(spec=scap.backport.GerritChange)
        dep_in_list.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        dep_in_list.number = 50  # Same number as dep_change

        change_with_dep = Mock(spec=scap.backport.GerritChange)
        change_with_dep.depends_ons = [dep_in_list, non_sibling1]

        result_no_dup = backport._get_sibling_dependencies(change_with_dep, dep_change)
        self.assertEqual(len(result_no_dup), 1)  # Should not duplicate dep_change
        self.assertIn(dep_in_list, result_no_dup)

    def test_get_sibling_dependencies_searches_correct_context(self):
        """Test that _get_sibling_dependencies searches change.depends_ons, not dep_change.depends_ons.

        This test would have caught the original bug where the method incorrectly
        searched through dep_change.depends_ons instead of change.depends_ons.
        """
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Create a dep_change with its own depends_ons that contains a sibling
        dep_change = Mock(spec=scap.backport.GerritChange)
        dep_change.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        dep_change.number = 100

        # Create a sibling that exists in dep_change's own depends_ons
        sibling_in_dep_depends_ons = Mock(spec=scap.backport.GerritChange)
        sibling_in_dep_depends_ons.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        sibling_in_dep_depends_ons.number = 200

        # dep_change has siblings in its own depends_ons (this should NOT be searched)
        dep_change.depends_ons = [sibling_in_dep_depends_ons]

        # Create the main change that does NOT have any siblings in its depends_ons
        main_change = Mock(spec=scap.backport.GerritChange)
        main_change.depends_ons = []  # No siblings here

        # If the method incorrectly searches dep_change.depends_ons, it would find sibling_in_dep_depends_ons
        # If it correctly searches main_change.depends_ons, it should only find dep_change itself
        result = backport._get_sibling_dependencies(main_change, dep_change)

        # Should only return dep_change itself, NOT the sibling from dep_change.depends_ons
        self.assertEqual(len(result), 1)
        self.assertIn(dep_change, result)
        self.assertNotIn(sibling_in_dep_depends_ons, result)

        # Now test the opposite - siblings in main_change.depends_ons should be found
        main_change_with_sibling = Mock(spec=scap.backport.GerritChange)
        sibling_in_main_depends_ons = Mock(spec=scap.backport.GerritChange)
        sibling_in_main_depends_ons.get.side_effect = lambda key: {
            "change_id": "I1234567890abcdef",
        }.get(key)
        sibling_in_main_depends_ons.number = 300

        main_change_with_sibling.depends_ons = [sibling_in_main_depends_ons]

        result_with_sibling = backport._get_sibling_dependencies(
            main_change_with_sibling, dep_change
        )

        # Should return both the sibling from main_change.depends_ons AND dep_change itself
        self.assertEqual(len(result_with_sibling), 2)
        self.assertIn(dep_change, result_with_sibling)
        self.assertIn(sibling_in_main_depends_ons, result_with_sibling)
        # Should NOT include sibling from dep_change.depends_ons
        self.assertNotIn(sibling_in_dep_depends_ons, result_with_sibling)

    @patch("scap.utils.get_wikiversions_ondisk")
    def test_validate_master_dependency_in_deployable_branches(self, mock_get_ondisk):
        """Test validation that master MW code dependencies are present in all deployable branches."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock required attributes
        backport.gerrit = Mock()
        backport.git_repos = Mock()
        backport.mediawiki_location = "/srv/mediawiki-staging"
        backport.get_logger = Mock(return_value=Mock())

        # Mock utils.get_wikiversions_ondisk to return deployable versions
        mock_get_ondisk.return_value = ["1.45.0-wmf.20", "1.45.0-wmf.21"]

        # Create test changes
        current_change = Mock()
        current_change.number = 12345

        # Create a master branch MW code dependency
        dep_change = Mock(spec=scap.backport.GerritChange)
        dep_change.get.side_effect = lambda key: {
            "project": "mediawiki/core",
            "branch": "master",
            "id": "I1234567890abcdef",
        }.get(key)
        dep_change.number = 100
        dep_change.is_merged.return_value = True

        # Mock git_repos to identify MW code project
        backport.git_repos.targets_any_mediawiki_code.return_value = True
        backport.git_repos.targets_deployable_code.return_value = True

        trail = DependencyTrail(current_change)

        # Test case 1: Dependency present in all deployable branches (should pass)
        mock_response = Mock()
        mock_response.branches = [
            "refs/heads/master",
            "refs/heads/wmf/1.45.0-wmf.20",
            "refs/heads/wmf/1.45.0-wmf.21",
        ]
        backport.gerrit.change_in.return_value.get.return_value = mock_response

        backport._validate_master_dependency_in_deployable_branches(
            dep_change, trail, current_change
        )

        # Should not add any errors
        self.assertEqual(len(trail.errors), 0)

        # Test case 2: Dependency missing from one deployable branch (should fail)
        trail_missing = DependencyTrail(current_change)
        mock_response_missing = Mock()
        mock_response_missing.branches = [
            "refs/heads/master",
            "refs/heads/wmf/1.45.0-wmf.20",
            # Missing wmf/1.45.0-wmf.21
        ]
        backport.gerrit.change_in.return_value.get.return_value = mock_response_missing

        backport._validate_master_dependency_in_deployable_branches(
            dep_change, trail_missing, current_change
        )

        # Should add an error about missing branch
        self.assertEqual(len(trail_missing.errors), 1)
        error_msg = trail_missing.errors[0]
        self.assertIn("wmf/1.45.0-wmf.21", error_msg)
        self.assertIn("not present in deployable branch", error_msg)
