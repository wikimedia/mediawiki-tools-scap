# -*- coding: utf-8 -*-

from scap import cli, lock, main


@cli.command(
    "php-fpm-restart",
    help="Restart php-fpm on all mediawiki targets",
    affected_by_blocked_deployments=True,
)
class PhpRpmRestartCmd(main.AbstractSync):
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        self.arguments.force = False

        self._assert_auth_sock()

        with lock.Lock(
            self.get_lock_file(), name="php-fpm-restart", reason=self.arguments.message
        ):
            self.announce("Starting php-fpm-restarts")
            self._restart_php()
            self.announce("Finished php-fpm-restarts")
