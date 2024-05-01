from scap import cli


def test_mulitple_uses_of_application_factory_with_plugins():
    cli.Application.factory(["backport", "--list"])
    cli.Application.factory(["clean", "auto"])
