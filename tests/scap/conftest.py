import pytest
from unittest.mock import Mock, patch


class FakeChange:
    """Lightweight test double for GerritChange"""

    def __init__(
        self,
        number,
        project="",
        branch="",
        id_=None,
        depends_ons=None,
        merged=False,
        abandoned=False,
    ):
        self.number = number
        self.project = project
        self.branch = branch
        self.id = id_ or f"I{number}"
        self.depends_ons = depends_ons or []
        self.merged = merged
        self.abandoned = abandoned

    def get(self, key, default=None):
        return getattr(self, key, default)

    def is_merged(self):
        return self.merged

    def check_abandoned_for_trail(self, trail):
        if self.abandoned:
            trail.errors.append(f"Change {self.number} is abandoned")


@pytest.fixture
def change_factory():
    """Factory for creating FakeChange instances"""
    return FakeChange


@pytest.fixture
def backport_factory():
    """Factory for creating mock Backport instances with common setup.

    Returns a function that creates a Backport mock with commonly needed attributes
    pre-configured to reduce test boilerplate.
    """

    def _create_backport(**overrides):
        """Create a mock Backport with sensible defaults.

        Args:
            **overrides: Any attribute values to override from defaults

        Returns:
            Mock Backport instance with common attributes configured
        """
        from scap.backport import Backport

        # Default configuration
        defaults = {
            "backports": Mock(changes={}, change_numbers=[]),
            "git_repos": Mock(
                change_is_deployable=Mock(return_value=True),
                change_targets_production=Mock(return_value=True),
            ),
            "gerrit": Mock(),
            "arguments": Mock(yes=False),
            "versions": ["1.45.0-wmf.21", "1.45.0-wmf.22"],
            "backport_or_revert": "backport",
            "logger": Mock(),
        }

        # Merge overrides with defaults
        config = {**defaults, **overrides}

        # Create mock Backport with __init__ bypassed
        with patch.object(Backport, "__init__", return_value=None):
            bp = Backport()

        # Apply all configurations
        for attr, value in config.items():
            setattr(bp, attr, value)

        return bp

    return _create_backport


@pytest.fixture
def anyio_backend():
    return "asyncio"
