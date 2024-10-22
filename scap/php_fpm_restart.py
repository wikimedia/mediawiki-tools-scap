# -*- coding: utf-8 -*-

from scap import cli, main


@cli.command(
    "php-fpm-restart",
    help="Restart php-fpm on all mediawiki targets",
    primary_deploy_server_only=True,
)
class PhpRpmRestartCmd(main.AbstractSync):
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        self.arguments.force = False

        self._assert_auth_sock()

        with self.lock_and_announce():
            self._restart_php()
