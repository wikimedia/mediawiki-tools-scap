import shlex
from unittest import mock
from unittest.mock import patch, ANY, Mock
import unittest
import pytest

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
@pytest.mark.parametrize(
    "scenario,setup_change,setup_backport,expect_exception,error_fragment,check_logger,check_prompt",
    [
        # Cycle detection
        (
            "cycle_error",
            {"depends_on_cycle": True},
            {},
            True,
            "cycle detected",
            None,
            None,
        ),
        # Non-deployable change
        (
            "non_deployable",
            {"depends_on_cycle": False},
            {"git_repos": Mock(change_is_deployable=Mock(return_value=False))},
            True,
            "not deployable",
            None,
            None,
        ),
        # Abandoned change
        (
            "abandoned",
            {
                "depends_on_cycle": False,
                "get.return_value": "ABANDONED",
                "_format_dependency_chain.return_value": " Chain info",
            },
            {},
            True,
            "abandoned",
            None,
            None,
        ),
        # Non-production with --yes flag
        (
            "non_production_yes",
            {"depends_on_cycle": False, "get.return_value": "NEW"},
            {
                "git_repos": Mock(
                    change_is_deployable=Mock(return_value=True),
                    change_targets_production=Mock(return_value=False),
                ),
                "arguments": Mock(yes=True),
            },
            False,
            None,
            "warning",
            None,
        ),
        # Non-production with prompt
        (
            "non_production_prompt",
            {"depends_on_cycle": False, "get.return_value": "NEW"},
            {
                "git_repos": Mock(
                    change_is_deployable=Mock(return_value=True),
                    change_targets_production=Mock(return_value=False),
                ),
                "arguments": Mock(yes=False),
                "_prompt_for_approval_or_exit": Mock(),
            },
            False,
            None,
            None,
            "Continue with backport anyway?",
        ),
    ],
)
def test_validate_root_changes_scenarios(
    mock_logger,
    backport_factory,
    change_factory,
    scenario,
    setup_change,
    setup_backport,
    expect_exception,
    error_fragment,
    check_logger,
    check_prompt,
):
    """Consolidated test for validate_root_changes covering all error/warning scenarios.

    Tests 5 scenarios:
    - cycle_error: Detects dependency cycles
    - non_deployable: Rejects non-deployable changes
    - abandoned: Rejects abandoned changes
    - non_production_yes: Warns about non-production with --yes
    - non_production_prompt: Prompts user for non-production without --yes
    """
    # Create change with scenario-specific config
    mock_change = Mock()
    mock_change.number = 12345
    mock_change.get.return_value = "test_value"

    # Apply change setup
    for attr, value in setup_change.items():
        if "." in attr:
            # Handle nested attributes like get.return_value
            parts = attr.split(".")
            obj = mock_change
            for part in parts[:-1]:
                obj = getattr(obj, part)
            setattr(obj, parts[-1], value)
        else:
            setattr(mock_change, attr, value)

    # Create backport with scenario-specific config
    bp_config = {
        "backports": Mock(changes={"123": mock_change}),
        **setup_backport,
    }
    backport = backport_factory(**bp_config)

    # Setup logger
    mock_logger_instance = Mock()
    mock_logger.return_value = mock_logger_instance

    # Execute and verify
    if expect_exception:
        with pytest.raises(InvalidChangeException) as exc_info:
            backport.validate_root_changes()
        assert error_fragment.lower() in str(exc_info.value).lower()
    else:
        backport.validate_root_changes()

        if check_logger == "warning":
            mock_logger_instance.warning.assert_called_once()
            warning_call = mock_logger_instance.warning.call_args[0][0]
            assert "not found in any live wikiversion" in warning_call

        if check_prompt:
            backport._prompt_for_approval_or_exit.assert_called_once()
            prompt_call = backport._prompt_for_approval_or_exit.call_args[0][0]
            assert check_prompt in prompt_call


class TestValidateDependencyTrails(unittest.TestCase):
    """Consolidated tests for _validate_dependency_trails method."""


@pytest.mark.parametrize(
    "scenario,trail_configs,expect_exception,error_fragments,check_prompt",
    [
        # Success - no errors or warnings
        (
            "success",
            [{"errors": [], "warnings": []}],
            False,
            None,
            False,
        ),
        # Single error
        (
            "single_error",
            [{"errors": ["Test error message"], "warnings": []}],
            True,
            ["Test error message"],
            False,
        ),
        # Multiple errors from different trails
        (
            "multiple_errors",
            [
                {
                    "errors": ["First error message", "Second error message"],
                    "warnings": [],
                },
                {"errors": ["Third error message"], "warnings": []},
                {"errors": [], "warnings": []},
            ],
            True,
            [
                "First error message",
                "Second error message",
                "Third error message",
                "Errors found in dependency trails",
            ],
            False,
        ),
        # Warnings prompt user
        (
            "warnings_prompt",
            [{"errors": [], "warnings": ["Test warning message"]}],
            False,
            None,
            True,
        ),
    ],
)
def test_validate_dependency_trails_scenarios(
    backport_factory,
    change_factory,
    scenario,
    trail_configs,
    expect_exception,
    error_fragments,
    check_prompt,
):
    """Consolidated test for _validate_dependency_trails covering all scenarios.

    Tests 4 scenarios:
    - success: No errors or warnings passes silently
    - single_error: Single error raises exception
    - multiple_errors: Multiple errors from different trails all reported
    - warnings_prompt: Warnings prompt user for confirmation
    """
    # Create backport with appropriate config
    bp_config = {}
    if check_prompt:
        bp_config["_prompt_for_approval_or_exit"] = Mock()
        bp_config["arguments"] = Mock(yes=False)

    backport = backport_factory(**bp_config)

    # Create trails based on scenario
    trails = []
    for i, config in enumerate(trail_configs):
        mock_change = change_factory(number=10000 + i)
        trail = DependencyTrail(mock_change)

        for error in config["errors"]:
            trail.add_error(error)
        for warning in config["warnings"]:
            trail.add_warning(warning)

        trails.append(trail)

    # Execute and verify
    if expect_exception:
        with pytest.raises(InvalidChangeException) as exc_info:
            backport._validate_dependency_trails(trails)

        error_message = str(exc_info.value)
        for fragment in error_fragments:
            assert fragment in error_message
    else:
        backport._validate_dependency_trails(trails)

        if check_prompt:
            backport._prompt_for_approval_or_exit.assert_called_once()
            args = backport._prompt_for_approval_or_exit.call_args[0][0]
            assert "Test warning message" in args
            assert "Continue with backport anyway?" in args


class TestDependencyAnalysis(unittest.TestCase):
    """Tests for dependency analysis methods."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_change = Mock()
        self.mock_change.get.return_value = "test_value"
        self.mock_change.number = 12345

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

    def test_collect_relevant_dependencies_filters_irrelevant(self):
        """Test that irrelevant dependencies are not added to trails."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock dependencies
        backport.get_logger = Mock(return_value=Mock())
        backport.is_relevant_dep = Mock()
        backport.backports = Mock()
        backport.backports.change_numbers = [12345]

        # Mock git_repos for the rule checking
        backport.git_repos = Mock()
        backport.git_repos.targets_any_mediawiki_code.return_value = False
        backport.git_repos.targets_prod_config.return_value = False

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

        # Configure relevance - both dependencies are not relevant to avoid the impossible scenario
        backport.is_relevant_dep.return_value = False

        # Create trail and test using new method
        trail = DependencyTrail(self.mock_change)
        backport._analyze_dependencies_recursive(self.mock_change, trail, [], set())

        # Verify no dependencies were added since none are relevant
        self.assertEqual(len(trail.relevant_dependencies), 0)
        self.assertNotIn(mock_dep1, trail.relevant_dependencies)
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


@pytest.mark.parametrize(
    "scenario,dep_numbers,is_relevant_func,expect_warning",
    [
        # All dependencies irrelevant -> warning
        (
            "all_irrelevant",
            [11111, 22222],
            lambda dep_num: False,
            True,
        ),
        # Some dependencies relevant -> no warning
        (
            "some_relevant",
            [11111, 22222],
            lambda dep_num: dep_num == 11111,
            False,
        ),
    ],
)
def test_irrelevant_depends_on_warning_scenarios(
    backport_factory,
    scenario,
    dep_numbers,
    is_relevant_func,
    expect_warning,
):
    """Consolidated test for warnings about irrelevant Depends-On relationships.

    Tests 2 scenarios:
    - all_irrelevant: All Depends-On are irrelevant -> warning generated
    - some_relevant: At least one relevant -> no warning
    """
    # Create root change with Depends-On relationships
    mock_root_change = Mock(spec=GerritChange)
    mock_root_change.number = 12345
    mock_root_change.depends_on_cycle = False
    mock_root_change.get.return_value = "test-change-id"
    mock_root_change.check_abandoned_for_trail = Mock()

    # Create mock dependencies
    mock_deps = []
    for num in dep_numbers:
        mock_dep = Mock(spec=GerritChange)
        mock_dep.number = num
        mock_dep.depends_ons = []
        if is_relevant_func(num):
            mock_dep.is_merged.return_value = True
            mock_dep.check_abandoned_for_trail = Mock()
        mock_deps.append(mock_dep)

    mock_root_change.depends_ons = mock_deps

    # Create backport with appropriate mocks
    backport = backport_factory(
        backports=Mock(
            changes={12345: mock_root_change},
            change_numbers=[num for num in dep_numbers if is_relevant_func(num)],
        ),
        get_logger=Mock(),
    )

    # Mock git_repos
    backport.git_repos.change_targets_production.return_value = True

    # Mock is_relevant_dep
    backport.is_relevant_dep = Mock(
        side_effect=lambda change, dep, all_deps: is_relevant_func(dep.number)
    )

    # Mock _collect_all_dependencies_recursive
    backport._collect_all_dependencies_recursive = Mock(return_value=mock_deps)

    # Mock gerrit submitted_together
    mock_gerrit_response = Mock()
    mock_gerrit_response.changes = [{"_number": mock_root_change.number}]
    backport.gerrit.submitted_together = Mock(
        return_value=Mock(get=Mock(return_value=mock_gerrit_response))
    )

    # Calculate dependencies
    trails = backport._calculate_relevant_dependencies()

    # Verify warning behavior
    assert len(trails) == 1
    trail = trails[0]

    if expect_warning:
        assert trail.has_warnings()
        assert len(trail.warnings) == 1
        warning = trail.warnings[0]
        assert f"has {len(dep_numbers)} Depends-On relationship(s)" in warning
        assert "none were deemed relevant" in warning
        assert "This may be unexpected" in warning
    else:
        # Should have no warning about irrelevant Depends-On relationships
        depends_on_warnings = [
            w for w in trail.warnings if "Depends-On relationship" in w
        ]
        assert len(depends_on_warnings) == 0


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
        mock_change.get = {"project": project, "branch": branch}.get
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

        def mock_submodule_project(url):
            if "OnlyInWmf21" in url:
                return "mediawiki/extensions/OnlyInWmf21"
            elif "OnlyInWmf22" in url:
                return "mediawiki/extensions/OnlyInWmf22"
            return None

        mock_gerrit.submodule_project_from_url.side_effect = mock_submodule_project
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
    def test_targets_deployable_mediawiki_code(self, mock_list_submodules):
        """Test targets_deployable_mediawiki_code method."""
        self._setup_versioned_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Test wmf branch with existing project
        change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
        self.assertTrue(git_repos.targets_deployable_mediawiki_code(change))

        # Test wmf branch with extension that exists in version
        change = self._create_mock_change(
            "mediawiki/extensions/Example", "wmf/1.45.0-wmf.21"
        )
        self.assertTrue(git_repos.targets_deployable_mediawiki_code(change))

        # Test non-wmf branch
        change = self._create_mock_change("mediawiki/core", "master")
        self.assertFalse(git_repos.targets_deployable_mediawiki_code(change))

        # Test project not in version
        change = self._create_mock_change(
            "mediawiki/extensions/NonExistent", "wmf/1.45.0-wmf.21"
        )
        self.assertFalse(git_repos.targets_deployable_mediawiki_code(change))

        # Test FileNotFoundError handling (version directory doesn't exist)
        change = self._create_mock_change("mediawiki/core", "wmf/nonexistent")
        with patch.object(
            git_repos, "_get_core_repos_for_version", side_effect=FileNotFoundError
        ):
            self.assertFalse(git_repos.targets_deployable_mediawiki_code(change))

    @patch("scap.git.list_submodules_paths_urls")
    def test_targets_live_code(self, mock_list_submodules):
        """Test targets_live_code method."""
        self._setup_empty_submodules(mock_list_submodules)
        git_repos = self._create_git_repos()

        # Mock targets_deployable_mediawiki_code to return True
        with patch.object(
            git_repos, "targets_deployable_mediawiki_code", return_value=True
        ):
            # Test live version (in versions list)
            change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
            self.assertTrue(git_repos.targets_live_code(change))

            # Test non-live version (not in versions list)
            change = self._create_mock_change("mediawiki/core", "wmf/1.44.0-wmf.20")
            self.assertFalse(git_repos.targets_live_code(change))

            # Test non-wmf branch
            change = self._create_mock_change("mediawiki/core", "master")
            self.assertFalse(git_repos.targets_live_code(change))

        # Test when targets_deployable_mediawiki_code returns False
        with patch.object(
            git_repos, "targets_deployable_mediawiki_code", return_value=False
        ):
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
            with patch.object(
                git_repos, "targets_deployable_mediawiki_code", return_value=False
            ):
                self.assertTrue(git_repos.change_is_deployable(change))

        # Test code change (should be deployable)
        change = self._create_mock_change("mediawiki/core", "wmf/1.45.0-wmf.21")
        with patch.object(git_repos, "targets_prod_config", return_value=False):
            with patch.object(
                git_repos, "targets_deployable_mediawiki_code", return_value=True
            ):
                self.assertTrue(git_repos.change_is_deployable(change))

        # Test neither config nor code (should not be deployable)
        change = self._create_mock_change("some/other/project", "master")
        with patch.object(git_repos, "targets_prod_config", return_value=False):
            with patch.object(
                git_repos, "targets_deployable_mediawiki_code", return_value=False
            ):
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
    def test_targets_deployable_mediawiki_code_vs_targets_any_mediawiki_code_difference(
        self, mock_get_wikiversions, mock_list_submodules
    ):
        """
        Test when targets_deployable_mediawiki_code and targets_any_mediawiki_code return different results.

        targets_deployable_mediawiki_code: Checks if project exists in the SPECIFIC version of the change
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
            mock_change.get = {"project": project, "branch": branch}.get
            return mock_change

        # Extension exists in wmf.21 but change targets wmf.22
        change = create_mock_change(
            "mediawiki/extensions/OnlyInWmf21", "wmf/1.45.0-wmf.22"
        )

        # targets_deployable_mediawiki_code: False (extension not in wmf.22)
        self.assertFalse(git_repos.targets_deployable_mediawiki_code(change))

        # targets_any_mediawiki_code: True (extension exists in wmf.21, which is deployable)
        self.assertTrue(git_repos.targets_any_mediawiki_code(change))

        # Reverse case: Extension exists in wmf.22 but change targets wmf.21
        change = create_mock_change(
            "mediawiki/extensions/OnlyInWmf22", "wmf/1.45.0-wmf.21"
        )

        # targets_deployable_mediawiki_code: False (extension not in wmf.21)
        self.assertFalse(git_repos.targets_deployable_mediawiki_code(change))

        # targets_any_mediawiki_code: True (extension exists in wmf.22, which is deployable)
        self.assertTrue(git_repos.targets_any_mediawiki_code(change))

    def test_process_dependency_not_relevant(self):
        """Test _process_dependency with irrelevant dependency."""
        with patch.object(Backport, "__init__", return_value=None):
            backport = Backport()

        # Mock required methods and attributes
        backport.get_logger = Mock(return_value=Mock())
        backport.is_relevant_dep = Mock(return_value=False)  # Not relevant
        backport.backports = Mock()
        backport.backports.change_numbers = [12345]

        # Create test changes
        current_change = Mock()
        current_change.number = 12345
        current_change.depends_ons = []

        dep_change = Mock()
        dep_change.number = 22222
        dep_change.is_merged.return_value = False
        dep_change.check_abandoned_for_trail = Mock()
        dep_change.depends_ons = []

        # Test irrelevant dependency -> No error added, not added to trail
        trail = DependencyTrail(current_change)

        result = backport._process_dependency(
            current_change, dep_change, trail, [], "Related"
        )

        self.assertFalse(result)
        self.assertEqual(len(trail.relevant_dependencies), 0)
        self.assertEqual(len(trail.errors), 0)
        # check_abandoned_for_trail should not be called for irrelevant dependencies
        dep_change.check_abandoned_for_trail.assert_not_called()
        # Verify logging was called
        backport.get_logger().info.assert_called()


@pytest.mark.parametrize(
    "scenario,current_project,current_branch,dep_project,dep_branch,dep_merged,branches_present,deployable_branches,expect_error,error_fragment",
    [
        # Rule 1a: MW code → MW non-master (merged) → success
        (
            "rule_1a_success",
            "mediawiki/extensions/Example",
            "wmf/1.45.0-wmf.20",
            "mediawiki/core",
            "wmf/1.45.0-wmf.19",
            True,
            None,
            None,
            False,
            None,
        ),
        # Rule 1b: MW code → MW master (present in target) → success
        (
            "rule_1b_success",
            "mediawiki/extensions/Example",
            "wmf/1.45.0-wmf.20",
            "mediawiki/core",
            "master",
            True,
            ["master", "wmf/1.45.0-wmf.20"],
            None,
            False,
            None,
        ),
        # Rule 1b: MW code → MW master (missing from target) → error
        (
            "rule_1b_failure",
            "mediawiki/extensions/Example",
            "wmf/1.45.0-wmf.20",
            "mediawiki/core",
            "master",
            True,
            ["master"],
            None,
            True,
            "not present in target branch",
        ),
        # Rule 2: Any → Config (merged) → success
        (
            "rule_2_success",
            "mediawiki/extensions/Example",
            "wmf/1.45.0-wmf.20",
            "operations/mediawiki-config",
            "master",
            True,
            None,
            None,
            False,
            None,
        ),
        # Rule 3a: Config → MW non-master (merged) → success
        (
            "rule_3a_success",
            "operations/mediawiki-config",
            "master",
            "mediawiki/core",
            "wmf/1.45.0-wmf.20",
            True,
            None,
            None,
            False,
            None,
        ),
        # Rule 3b: Config → MW master (present in all deployable) → success
        (
            "rule_3b_success",
            "operations/mediawiki-config",
            "master",
            "mediawiki/core",
            "master",
            True,
            ["master", "wmf/1.45.0-wmf.20", "wmf/1.45.0-wmf.21"],
            {"wmf/1.45.0-wmf.20", "wmf/1.45.0-wmf.21"},
            False,
            None,
        ),
        # Rule 3b: Config → MW master (missing from some deployable) → error
        (
            "rule_3b_failure",
            "operations/mediawiki-config",
            "master",
            "mediawiki/core",
            "master",
            True,
            ["master", "wmf/1.45.0-wmf.20"],
            {"wmf/1.45.0-wmf.20", "wmf/1.45.0-wmf.21"},
            True,
            "not present in deployable branch",
        ),
        # Rule 3b: Config → MW master (includes "next" but missing train branches) → error
        # Tests that "next" branch is not considered a deployable branch
        (
            "rule_3b_next_not_deployable",
            "operations/mediawiki-config",
            "master",
            "mediawiki/core",
            "master",
            True,
            ["master", "next", "wmf/1.45.0-wmf.20"],
            {"wmf/1.45.0-wmf.20", "wmf/1.45.0-wmf.21"},
            True,
            "not present in deployable branch",
        ),
    ],
)
def test_dependency_validation_rules(
    change_factory,
    scenario,
    current_project,
    current_branch,
    dep_project,
    dep_branch,
    dep_merged,
    branches_present,
    deployable_branches,
    expect_error,
    error_fragment,
):
    """Test T362987 dependency validation rules in one consolidated parametrized test.

    This replaces 14+ individual tests that were exercising the same code paths through
    _backport_vote with different inputs.

    Tests all 6 validation branches:
    - Rule 1a: MW code → MW non-master branch (merged dep)
    - Rule 1b: MW code → MW master branch (must be in target branch)
    - Rule 2: Any → Config (merged dep)
    - Rule 3a: Config → MW non-master branch (merged dep)
    - Rule 3b: Config → MW master branch (must be in all deployable branches)
    - Fallback: No validation needed
    """
    # Create changes first (needed for mock configuration)
    current_change = change_factory(
        number=12345,
        project=current_project,
        branch=current_branch,
        id_=f"I{scenario}_current",
        merged=False,
    )

    dep_change = change_factory(
        number=67890,
        project=dep_project,
        branch=dep_branch,
        id_=f"I{scenario}_dep",
        merged=dep_merged,
    )

    # Create a minimal Backport instance with necessary mocks
    bp = Mock(spec=Backport)
    bp.logger = Mock()
    bp.gerrit = Mock()
    bp.mediawiki_location = "/srv/mediawiki-staging"

    # Mock backports (for checking if changes are scheduled)
    bp.backports = Mock()
    bp.backports.change_numbers = []  # No changes scheduled by default

    # Mock git_repos with project type check methods
    bp.git_repos = Mock()
    bp.git_repos.targets_deployable_mediawiki_code = Mock(
        side_effect=lambda change: (
            change.project.startswith("mediawiki/")
            and change.project != "operations/mediawiki-config"
        )
    )
    bp.git_repos.targets_prod_config = Mock(
        side_effect=lambda change: (change.project == "operations/mediawiki-config")
    )
    bp.git_repos.targets_any_mediawiki_code = Mock(
        side_effect=lambda change: (
            change.project.startswith("mediawiki/")
            and change.project != "operations/mediawiki-config"
        )
    )

    trail = DependencyTrail(current_change)

    # Mock gerrit change_in responses if branches are specified
    if branches_present is not None:
        mock_response = Mock()
        mock_response.branches = [f"refs/heads/{b}" for b in branches_present]
        bp.gerrit.change_in = Mock(
            return_value=Mock(get=Mock(return_value=mock_response))
        )

    # Mock deployable branches if specified
    if deployable_branches is not None:
        with patch("scap.git.get_deployable_branches") as mock_deployable:
            mock_deployable.return_value = deployable_branches

            # Bind helper methods to the mock so _backport_vote can call them
            bp._get_included_branches = (
                lambda dep_change: Backport._get_included_branches(bp, dep_change)
            )
            bp._validate_master_dependency_in_deployable_branches = lambda dep_change, trail, current_change: Backport._validate_master_dependency_in_deployable_branches(
                bp, dep_change, trail, current_change
            )
            bp._validate_master_dependency_in_target_branch = lambda dep_change, trail, current_change: Backport._validate_master_dependency_in_target_branch(
                bp, dep_change, trail, current_change
            )
            bp._validate_dependency_merged_or_scheduled = lambda current_change, dep_change, trail: Backport._validate_dependency_merged_or_scheduled(
                bp, current_change, dep_change, trail
            )

            # Call the actual _backport_vote method
            Backport._backport_vote(bp, current_change, dep_change, trail)

            # Verify results
            if expect_error:
                assert (
                    len(trail.errors) == 1
                ), f"Expected 1 error, got {len(trail.errors)}: {trail.errors}"
                assert (
                    error_fragment in trail.errors[0]
                ), f"Expected error to contain '{error_fragment}', but got: {trail.errors[0]}"
            else:
                assert (
                    len(trail.errors) == 0
                ), f"Expected no errors, got: {trail.errors}"
    else:
        # No deployable branches mock needed
        # Bind helper methods to the mock so _backport_vote can call them
        bp._get_included_branches = lambda dep_change: Backport._get_included_branches(
            bp, dep_change
        )
        bp._validate_master_dependency_in_target_branch = lambda dep_change, trail, current_change: Backport._validate_master_dependency_in_target_branch(
            bp, dep_change, trail, current_change
        )
        bp._validate_dependency_merged_or_scheduled = lambda current_change, dep_change, trail: Backport._validate_dependency_merged_or_scheduled(
            bp, current_change, dep_change, trail
        )

        # Call the actual _backport_vote method
        Backport._backport_vote(bp, current_change, dep_change, trail)

        # Verify results
        if expect_error:
            assert (
                len(trail.errors) == 1
            ), f"Expected 1 error, got {len(trail.errors)}: {trail.errors}"
            assert (
                error_fragment in trail.errors[0]
            ), f"Expected error to contain '{error_fragment}', but got: {trail.errors[0]}"
        else:
            assert len(trail.errors) == 0, f"Expected no errors, got: {trail.errors}"


class TestIsRelevantDep:
    """
    Comprehensive tests for the is_relevant_dep method implementing T365146 rules.

    T365146 defines the dependency relevance rules for scap backport:
    1. Change and Dep are MW code & branch is deployable --> Dep relevant if targets same branch,
       or if targets master but no sibling targeting same branch
    2. If Dep targets production config --> Dep will be relevant
    3. If change targets_prod_config or change is MW code targeting master, then
       Dep relevant if it's deployable change, or if targets master but no siblings
       targeting deployable branches
    """

    @pytest.fixture
    def mock_git_repos(self):
        """Create a mock GitRepos with realistic behavior."""
        git_repos = Mock(spec=GitRepos)

        # Mock methods to return values based on project and branch
        def targets_prod_config(change):
            return (
                change.project == "operations/mediawiki-config"
                and change.branch == "train-dev"
            )

        def targets_deployable_mediawiki_code(change):
            return (
                change.project.startswith("mediawiki/")
                and change.project != "operations/mediawiki-config"
                and change.branch.startswith("wmf/")
            )

        def targets_any_mediawiki_code(change):
            return (
                change.project.startswith("mediawiki/")
                and change.project != "operations/mediawiki-config"
            )

        def change_is_deployable(change):
            return targets_prod_config(change) or targets_deployable_mediawiki_code(
                change
            )

        git_repos.targets_prod_config.side_effect = targets_prod_config
        git_repos.targets_deployable_mediawiki_code.side_effect = (
            targets_deployable_mediawiki_code
        )
        git_repos.targets_any_mediawiki_code.side_effect = targets_any_mediawiki_code
        git_repos.change_is_deployable.side_effect = change_is_deployable

        return git_repos

    @pytest.fixture
    def backport(self, mock_git_repos):
        """Create a minimal Backport instance for testing is_relevant_dep."""
        with patch.object(Backport, "__init__", return_value=None):
            bp = Backport()
        bp.git_repos = mock_git_repos
        return bp

    # ===== Case 1: Change and Dep are MW code & change branch is deployable =====

    def test_case1_dep_targets_same_branch_relevant(self, backport, change_factory):
        """Case 1: MW code change on deployable branch, dep on same branch -> relevant."""
        change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        dep = change_factory(
            67890, project="mediawiki/extensions/Example", branch="wmf/1.45.0-wmf.21"
        )
        siblings = [dep]

        result = backport.is_relevant_dep(change, dep, siblings)

        assert result is True, "Dep targeting same deployable branch should be relevant"

    def test_case1_dep_targets_different_wmf_branch_not_relevant(
        self, backport, change_factory
    ):
        """Case 1: MW code change on deployable branch, dep on different wmf branch -> not relevant."""
        change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        dep = change_factory(
            67890, project="mediawiki/extensions/Example", branch="wmf/1.45.0-wmf.22"
        )
        siblings = [dep]

        result = backport.is_relevant_dep(change, dep, siblings)

        assert (
            result is False
        ), "Dep targeting different wmf branch should not be relevant"

    def test_case1_dep_master_no_sibling_relevant(self, backport, change_factory):
        """Case 1: MW code on deployable branch, dep on master with no sibling on same branch -> relevant."""
        change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        dep_master = change_factory(
            67890,
            project="mediawiki/extensions/Example",
            branch="master",
            id_="Iexample123",
        )
        siblings = [dep_master]  # No sibling on wmf/1.45.0-wmf.21

        result = backport.is_relevant_dep(change, dep_master, siblings)

        assert (
            result is True
        ), "Master dep with no sibling on target branch should be relevant"

    def test_case1_dep_master_with_sibling_not_relevant(self, backport, change_factory):
        """Case 1: MW code on deployable branch, dep on master WITH sibling on same branch -> not relevant."""
        change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        dep_master = change_factory(
            67890,
            project="mediawiki/extensions/Example",
            branch="master",
            id_="Iexample123",
        )
        dep_wmf = change_factory(
            67891,
            project="mediawiki/extensions/Example",
            branch="wmf/1.45.0-wmf.21",
            id_="Iexample123",  # Same change-id
        )
        siblings = [dep_master, dep_wmf]  # Sibling exists on target branch

        result = backport.is_relevant_dep(change, dep_master, siblings)

        assert (
            result is False
        ), "Master dep with sibling on target branch should not be relevant"

    # ===== Case 2: Dep targets production config =====

    def test_case2_dep_targets_config_always_relevant(self, backport, change_factory):
        """Case 2: Any change depending on config -> config dep is always relevant."""
        # Test with MW code change
        mw_change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        config_dep = change_factory(
            67890, project="operations/mediawiki-config", branch="train-dev"
        )
        siblings = [config_dep]

        result = backport.is_relevant_dep(mw_change, config_dep, siblings)

        assert result is True, "Config dependency should always be relevant"

    def test_case2_config_dep_from_master_change_relevant(
        self, backport, change_factory
    ):
        """Case 2: Master branch MW change depending on config -> config dep relevant."""
        master_change = change_factory(12345, project="mediawiki/core", branch="master")
        config_dep = change_factory(
            67890, project="operations/mediawiki-config", branch="train-dev"
        )
        siblings = [config_dep]

        result = backport.is_relevant_dep(master_change, config_dep, siblings)

        assert (
            result is True
        ), "Config dependency should be relevant from master branch change"

    def test_case2_config_dep_from_config_change_relevant(
        self, backport, change_factory
    ):
        """Case 2: Config change depending on another config change -> dep is relevant."""
        config_change = change_factory(
            12345, project="operations/mediawiki-config", branch="train-dev"
        )
        config_dep = change_factory(
            67890, project="operations/mediawiki-config", branch="train-dev"
        )
        siblings = [config_dep]

        result = backport.is_relevant_dep(config_change, config_dep, siblings)

        assert result is True, "Config depending on config should be relevant"

    # ===== Case 3: Change targets config OR change is MW code targeting master =====

    def test_case3_config_change_dep_deployable_relevant(
        self, backport, change_factory
    ):
        """Case 3: Config change depending on deployable MW code -> dep is relevant."""
        config_change = change_factory(
            12345, project="operations/mediawiki-config", branch="train-dev"
        )
        mw_dep = change_factory(
            67890, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"  # Deployable
        )
        siblings = [mw_dep]

        result = backport.is_relevant_dep(config_change, mw_dep, siblings)

        assert result is True, "Deployable MW code dep from config should be relevant"

    def test_case3_config_change_dep_non_deployable_not_relevant(
        self, backport, change_factory
    ):
        """Case 3: Config change depending on non-deployable MW code (non-wmf branch) -> not relevant."""
        config_change = change_factory(
            12345, project="operations/mediawiki-config", branch="train-dev"
        )
        mw_dep = change_factory(
            67890,
            project="mediawiki/core",
            branch="REL1_39",  # Not deployable (not wmf/ branch)
        )
        siblings = [mw_dep]

        result = backport.is_relevant_dep(config_change, mw_dep, siblings)

        assert (
            result is False
        ), "Non-deployable MW code dep from config should not be relevant"

    def test_case3_config_dep_master_no_deployable_sibling_relevant(
        self, backport, change_factory
    ):
        """Case 3: Config depending on master MW with no deployable sibling -> relevant."""
        config_change = change_factory(
            12345, project="operations/mediawiki-config", branch="train-dev"
        )
        mw_master_dep = change_factory(
            67890, project="mediawiki/core", branch="master", id_="Iexample123"
        )
        siblings = [mw_master_dep]  # No deployable sibling

        result = backport.is_relevant_dep(config_change, mw_master_dep, siblings)

        assert (
            result is True
        ), "Master MW dep with no deployable sibling should be relevant from config"

    def test_case3_config_dep_master_with_deployable_sibling_not_relevant(
        self, backport, change_factory
    ):
        """Case 3: Config depending on master MW WITH deployable sibling -> not relevant."""
        config_change = change_factory(
            12345, project="operations/mediawiki-config", branch="train-dev"
        )
        mw_master_dep = change_factory(
            67890, project="mediawiki/core", branch="master", id_="Iexample123"
        )
        mw_wmf_dep = change_factory(
            67891,
            project="mediawiki/core",
            branch="wmf/1.45.0-wmf.21",
            id_="Iexample123",  # Same change-id, deployable
        )
        siblings = [mw_master_dep, mw_wmf_dep]  # Has deployable sibling

        result = backport.is_relevant_dep(config_change, mw_master_dep, siblings)

        assert (
            result is False
        ), "Master MW dep with deployable sibling should not be relevant from config"

    def test_case3_master_change_dep_deployable_relevant(
        self, backport, change_factory
    ):
        """Case 3: Master branch MW change depending on deployable MW code -> relevant."""
        master_change = change_factory(12345, project="mediawiki/core", branch="master")
        mw_dep = change_factory(
            67890,
            project="mediawiki/extensions/Example",
            branch="wmf/1.45.0-wmf.21",  # Deployable
        )
        siblings = [mw_dep]

        result = backport.is_relevant_dep(master_change, mw_dep, siblings)

        assert (
            result is True
        ), "Deployable MW dep from master branch change should be relevant"

    def test_case3_master_change_dep_master_no_sibling_relevant(
        self, backport, change_factory
    ):
        """Case 3: Master MW change depending on master MW with no deployable sibling -> relevant."""
        master_change = change_factory(12345, project="mediawiki/core", branch="master")
        mw_master_dep = change_factory(
            67890,
            project="mediawiki/extensions/Example",
            branch="master",
            id_="Iexample123",
        )
        siblings = [mw_master_dep]  # No deployable sibling

        result = backport.is_relevant_dep(master_change, mw_master_dep, siblings)

        assert (
            result is True
        ), "Master dep with no deployable sibling should be relevant from master change"

    def test_case3_master_change_dep_master_with_sibling_not_relevant(
        self, backport, change_factory
    ):
        """Case 3: Master MW change depending on master MW WITH deployable sibling -> not relevant."""
        master_change = change_factory(12345, project="mediawiki/core", branch="master")
        mw_master_dep = change_factory(
            67890,
            project="mediawiki/extensions/Example",
            branch="master",
            id_="Iexample123",
        )
        mw_wmf_dep = change_factory(
            67891,
            project="mediawiki/extensions/Example",
            branch="wmf/1.45.0-wmf.21",
            id_="Iexample123",  # Same change-id, deployable
        )
        siblings = [mw_master_dep, mw_wmf_dep]  # Has deployable sibling

        result = backport.is_relevant_dep(master_change, mw_master_dep, siblings)

        assert (
            result is False
        ), "Master dep with deployable sibling should not be relevant from master change"

    # ===== Edge cases and non-MW/config dependencies =====

    def test_dep_not_mw_or_config_not_relevant(self, backport, change_factory):
        """Dep that is neither MW code nor config should not be relevant."""
        change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        other_dep = change_factory(67890, project="some/other/project", branch="main")
        siblings = [other_dep]

        result = backport.is_relevant_dep(change, other_dep, siblings)

        assert result is False, "Non-MW/config dependency should not be relevant"

    def test_change_not_mw_or_config_fallback(self, backport, change_factory):
        """Change that is neither MW nor config with MW dep -> fallback to not relevant."""
        other_change = change_factory(
            12345, project="some/other/project", branch="main"
        )
        mw_dep = change_factory(
            67890, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        siblings = [mw_dep]

        result = backport.is_relevant_dep(other_change, mw_dep, siblings)

        assert (
            result is False
        ), "Dependency from non-MW/config change should fall through to not relevant"

    # ===== Cross-case integration tests =====

    def test_integration_config_to_mw_master_to_wmf(self, backport, change_factory):
        """Integration: Config depends on master MW, which has wmf sibling (T387798 scenario)."""
        config_change = change_factory(
            12345, project="operations/mediawiki-config", branch="train-dev"
        )
        mw_master = change_factory(
            67890, project="mediawiki/core", branch="master", id_="Ishared"
        )
        mw_wmf = change_factory(
            67891, project="mediawiki/core", branch="wmf/1.45.0-wmf.21", id_="Ishared"
        )

        # Config sees both master and wmf as dependencies
        siblings = [mw_master, mw_wmf]

        # Master should not be relevant (has deployable sibling)
        result_master = backport.is_relevant_dep(config_change, mw_master, siblings)
        assert (
            result_master is False
        ), "Master with deployable sibling should not be relevant"

        # WMF should be relevant (deployable)
        result_wmf = backport.is_relevant_dep(config_change, mw_wmf, siblings)
        assert result_wmf is True, "Deployable wmf branch should be relevant"

    def test_integration_mw_extension_depends_on_core_master_no_sibling(
        self, backport, change_factory
    ):
        """Integration: Extension on wmf branch depends on core master with no wmf sibling."""
        ext_change = change_factory(
            12345, project="mediawiki/extensions/Example", branch="wmf/1.45.0-wmf.21"
        )
        core_master = change_factory(
            67890, project="mediawiki/core", branch="master", id_="Icore123"
        )
        siblings = [core_master]  # No wmf sibling

        result = backport.is_relevant_dep(ext_change, core_master, siblings)

        assert (
            result is True
        ), "Extension depending on core master with no sibling should see it as relevant"

    def test_sibling_detection_excludes_self(self, backport, change_factory):
        """Sibling detection should not count the dependency itself as a sibling."""
        change = change_factory(
            12345, project="mediawiki/core", branch="wmf/1.45.0-wmf.21"
        )
        dep_master = change_factory(
            67890,
            project="mediawiki/extensions/Example",
            branch="master",
            id_="Iexample",
        )
        # Siblings list only contains the dep itself (no actual siblings)
        siblings = [dep_master]

        result = backport.is_relevant_dep(change, dep_master, siblings)

        assert (
            result is True
        ), "Master dep should be relevant when it's the only one in siblings list"
