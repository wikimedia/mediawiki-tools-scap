# Import gevent.monkey and do tzhe monkey patching
from gevent import monkey; monkey.patch_all()  # noqa

import gevent
import urllib3

from scap import targets


class OpcacheManager(object):
    # Global timeout for all objects
    TIMEOUT = 5

    def __init__(self):
        self.http = urllib3.PoolManager({'retries': 1, 'timeout': 1})
        self.config = {}

    def _invalidate_host(self, url, filename):
        if filename is not None:
            params = {'file': filename}
        else:
            params = {}
        try:
            result = self.http.request('GET', url, fields=params)
            if result.status == 200:
                # If the return status is 200, the request succeeded
                return (True, None)
            else:
                return (False,
                        "Response returned status {}".format(result.status))
        except Exception as e:
            return (False, str(e))

    def invalidate(self, hosts, admin_port, filename):
        """Invalidates files/directories (or all) opcache."""
        failed = {}
        jobs = {}
        for host in hosts:
            try:
                url = "http://{hostname}:{port}/opcache-free".format(
                    hostname=host, port=admin_port)
                jobs[host] = gevent.spawn(self._invalidate_host, url, filename)
            except Exception as e:
                failed[host] = str(e)
        gevent.joinall(jobs.values(), self.TIMEOUT)
        for host, job in jobs.items():
            if job.successful():
                print(job.value)
                outcome, msg = job.value
                if not outcome:
                    failed[host] = msg
            else:
                failed[host] = \
                    "A timeout happened before a response was received"
        return failed

    def invalidate_all(self, admin_port, filename=None):
        # get all current hostgroups, or default to
        groups = self.config.get('mw_web_clusters',
                                 self.config['dsh-targets']).split(',')
        failed = {}
        for group in groups:
            hosts = targets.get(group.strip(), self.config).all
            failed.update(self.invalidate(hosts, admin_port, filename))
        return failed
