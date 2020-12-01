import requests

from concurrent.futures import ThreadPoolExecutor

from scap import targets


class OpcacheManager(object):
    # Global timeout for all objects
    TIMEOUT = 5

    def __init__(self, admin_port):
        self.admin_port = admin_port
        self.threadpool = ThreadPoolExecutor(max_workers=10)

    def _invalidate_host(self, host, filename):
        url = "http://{hostname}:{port}/opcache-free".format(
            hostname=host, port=self.admin_port
        )
        if filename is not None:
            params = {"file": filename}
        else:
            params = {}
        try:
            result = requests.get(url, params=params, timeout=self.TIMEOUT)
            result.raise_for_status()
            return (True, None)
        except requests.exceptions.HTTPError as e:
            return (False, "Response returned error {}".format(str(e)))
        except requests.exceptions.Timeout:
            return (False, "A timeout happened before a response was received")
        except Exception as e:
            return (False, str(e))

    def invalidate(self, hosts, filename):
        """Invalidates files/directories (or all) opcache."""

        def invalidate_closure(host):
            return (host, self._invalidate_host(host, filename))

        results = self.threadpool.map(invalidate_closure, hosts)
        # Collect all failed results and return them to the caller
        return {host: result[1] for host, result in results if not result[0]}

    def invalidate_all(self, config, filename=None):
        target_groups = targets.DirectDshTargetList("mw_web_clusters", config)
        # Fallback if nothing was defined.
        if not target_groups.all:
            target_groups.primary_key = "dsh_targets"
            target_groups.deploy_groups = None
        failed = {}
        for _, group in target_groups.groups.items():
            failed.update(self.invalidate(group.targets, filename))
        return failed
