import pytest
from scap import cli


@pytest.fixture
def cmd(request):
    """Fully initialized class"""
    # This is quite convoluted.
    # We instantiate a dummy subclass from the factory.
    # By doing this, we return a fully configured
    # application.

    if not hasattr(request, "param"):
        app = cli.Application.factory(["sync-world"])
    else:
        app = cli.Application.factory(request.param)

    # Do some basic initialization
    app.setup()

    return app


def test_init():
    expected_name = "sync"
    app = cli.Application(expected_name)
    assert app.program_name == expected_name


@pytest.mark.parametrize(
    "cmd",
    [
        # Args ok
        ["sync-world", "-n", "-w", "7"],
        # Args ok
        ["sync-world", "-n", "--canary-wait-time", "30"],
        # Args ok
        ["sync-world", "-n", "-w", "92"],
        # Wrong way of specifying canary wait time
        ["sync-world", "-n", "-D", "canary_wait_time:30"],
    ],
    indirect=True,
)
def test_scap_sync_world_flags(cmd):
    # Testing if we are parsing -w propertly. argparse makes sure cw
    # is int or exists if -w flag is set, so no need to test for those.
    try:
        cmd.main(cmd.extra_arguments)
    except ValueError as ve:
        assert "canary_wait_time" in cmd.arguments.defines[0]
        assert "defined" in str(ve)


@pytest.mark.parametrize(
    "cmd,expected",
    [
        (["sync-world"], False),
        (["sync-world", "--scope", "*"], False),
        (["sync-world", "--scope", "pretrain"], True),
        (["sync-world", "--scope", "pretrain,train"], True),
        (["sync-world", "--k8s-only"], True),
        (["sync-file", "somefile"], False),
        (["sync-file", "somefile", "--scope", "pretrain"], True),
    ],
    indirect=["cmd"],
)
def test_scope_implies_k8s_only(cmd, expected):
    # A scoped deploy (--scope other than "*") applies only to Kubernetes
    # targets, so it must force a k8s-only sync.
    assert cmd._k8s_only_sync() is expected


def test_supervise_without_k8s_deployment():
    # When deploy_mw_container_image is False (e.g. a bare-metal-only sync on beta),
    # k8s_ops.k8s_deployments_config is never populated. _supervise must not touch
    # it; it should fall back to a single check over the (empty) k8s targets plus
    # the bare-metal hosts.
    from types import SimpleNamespace
    from scap.main import AbstractSync
    from scap.kubernetes import CANARIES

    class _K8sOpsStub:
        def get_stage_dep_configs(self, stage):
            return []

        def __getattr__(self, name):
            raise AssertionError(
                f"k8s_ops.{name} accessed when deploy_mw_container_image is False"
            )

    captured = {}

    def fake_run_logstash_checks(stage_label, checks):
        captured["checks"] = checks
        return True

    fake_self = SimpleNamespace(
        config={"deploy_mw_container_image": False},
        k8s_ops=_K8sOpsStub(),
        _logstash_checker=lambda dep_configs, baremetal: (
            "checker",
            dep_configs,
            baremetal,
        ),
        _run_logstash_checks=fake_run_logstash_checks,
    )

    result = AbstractSync._supervise(fake_self, CANARIES, "canary", 10, ["bm-canary"])

    assert result is True
    # One fallback check: empty k8s targets, bare-metal hosts, fallback threshold.
    assert captured["checks"] == [(("checker", [], ["bm-canary"]), 10)]


def _supervise_stub(commands, commands_ok, calls):
    from types import SimpleNamespace
    from scap.kubernetes import LogstashCheck

    dep_conf = SimpleNamespace(
        supervision_rules=[object()],  # non-None -> supervision path
        resolve_command_checks=lambda stage: commands,
        resolve_logstash_checks=lambda stage: [
            LogstashCheck(threshold=10, scope="train", stage="canaries")
        ],
        supervised_dep_configs=lambda scope, stage: [],
    )

    def fake_run_commands(stage_label, cmds):
        calls.append(("commands", cmds))
        return commands_ok

    def fake_run_logstash(stage_label, checks):
        calls.append(("logstash", checks))
        return True

    return SimpleNamespace(
        config={"deploy_mw_container_image": True},
        k8s_ops=SimpleNamespace(k8s_deployments_config=dep_conf),
        _run_command_checks=fake_run_commands,
        _logstash_checker=lambda dep_configs, baremetal: ("checker", dep_configs),
        _run_logstash_checks=fake_run_logstash,
    )


def test_supervise_runs_commands_before_logstash():
    from scap.main import AbstractSync
    from scap.kubernetes import CANARIES

    calls = []
    fake_self = _supervise_stub(["httpbb --hosts=x"], True, calls)

    result = AbstractSync._supervise(fake_self, CANARIES, "canary", 10, [])

    assert result is True
    # Commands run first, then the logstash checks.
    assert [kind for kind, _ in calls] == ["commands", "logstash"]
    assert calls[0][1] == ["httpbb --hosts=x"]


def test_supervise_failing_commands_skip_logstash():
    from scap.main import AbstractSync
    from scap.kubernetes import CANARIES

    calls = []
    fake_self = _supervise_stub(["httpbb --hosts=x"], False, calls)

    result = AbstractSync._supervise(fake_self, CANARIES, "canary", 10, [])

    assert result is False
    # Commands failed, so the logstash checks are never run.
    assert [kind for kind, _ in calls] == ["commands"]


def test_increment_stat(cmd, mocker):
    assert cmd._stats is None
    stats = mocker.patch.object(cmd, "_stats")

    # Case 1
    cmd.increment_stat("scap")
    stats.increment.assert_any_call("deploy.scap", 1)
    stats.increment.assert_any_call("deploy.all", 1)

    # Case 2
    mocker.resetall()
    cmd.increment_stat("sync-file", all_stat=False, value=5)
    stats.increment.assert_called_once_with("deploy.sync-file", 5)


api_canaries = ["pigeon.w.o", "crow.w.o", "cormorant.w.o", "flamingo.w.o"]
app_canaries = ["birdy.w.o", "hawk.w.o", "duck.w.o", "chick.w.o"]


def test__get_api_canary_list(cmd, mocker):
    tl = mocker.patch("scap.targets.get")
    tl.return_value.all = api_canaries
    assert cmd._get_api_canary_list() == api_canaries


def test__get_app_canary_list(cmd, mocker):
    tl = mocker.patch("scap.targets.get")
    tl.return_value.all = app_canaries
    assert cmd._get_app_canary_list() == app_canaries


def test__get_canary_list(cmd, mocker):
    canary_targets = mocker.patch("scap.targets.get")

    def side_effect_cb(label, config):
        tgt = mocker.MagicMock()
        if label == "dsh_api_canaries":
            all = api_canaries
        else:
            all = app_canaries
        tgt.all = all
        return tgt

    canary_targets.side_effect = side_effect_cb
    assert sorted(cmd._get_canary_list()) == sorted(app_canaries + api_canaries)


def test_exclude_wikiversions():
    sync_world_app = cli.Application.factory(["sync-world"])
    sync_world_app.setup()

    assert "--exclude-wikiversions.php" in sync_world_app._base_scap_pull_command()

    sync_wikiversions_app = cli.Application.factory(["sync-wikiversions"])
    sync_wikiversions_app.setup()

    assert (
        "--exclude-wikiversions.php"
        not in sync_wikiversions_app._base_scap_pull_command()
    )


def test_handle_exception(cmd, mocker):
    cmd.arguments = mocker.MagicMock()
    cmd.arguments.no_log_message = False

    # Mock the logger
    get_log = mocker.patch("logging.getLogger")
    announcer = get_log.return_value

    valueError = ValueError("test")

    assert cmd._handle_exception(valueError) == 1

    get_log.assert_called_with("scap.announce")
    assert cmd._announce_logger == announcer
    announcer.info.assert_called()


def test_init_history_includes_foss_violations_checkouts(cmd, mocker):
    cmd.config["stage_dir"] = "/srv/mediawiki-staging"
    cmd.config["foss_violations"] = [
        {"repo": "https://example/repo-a", "branch": "main", "path": "viol-a"},
        {"repo": "https://example/repo-b", "branch": "dev", "path": "viol-b"},
    ]

    mocker.patch.object(cmd, "scap_history_dbfile", return_value="/tmp/history.db")
    mocker.patch.object(cmd, "active_wikiversions", return_value=["1.45.0-wmf.1"])

    mocker.patch("scap.main.git.get_branch", return_value="branch")
    mocker.patch("scap.main.git.merge_base", return_value="commit")
    mocker.patch("scap.main.git.remote_get_url", return_value="repo")

    cmd._init_history()

    assert [checkout.directory for checkout in cmd.deployment_log_entry.checkouts] == [
        "/srv/mediawiki-staging",
        "/srv/mediawiki-staging/php-1.45.0-wmf.1",
        "/srv/mediawiki-staging/viol-a",
        "/srv/mediawiki-staging/viol-b",
    ]
