# -*- coding: utf-8 -*-
"""
    scap.targets
    ~~~~~~~~
    Classes and helpers for getting a list of hosts to do things to
"""
import collections
import os
import re
import string

from . import utils


def get(key, cfg, limit_hosts=None, extra_paths=[]):
    """
    Factory function to get a TargetList object to fetch a list of targets.

    Right now this is only flat files, mostly from dsh.

    :param key: str The primary configuration key to look for,
                    combined with the server_groups config
    :param cfg: dict Ordered dictionary of configuration
    :param limit_hosts: str A pattern to limit host names by. See
        limit_target_hosts for further information on the format
    :param extra_paths: list of extra paths to search for list files in
    """
    return DshTargetList(key, cfg, limit_hosts, extra_paths)


def limit_target_hosts(pattern, hosts):
    """
    Return a subset of hosts based on wildcards.

    If the pattern can specify a range of the format '[start:end]'.

    If the supplied pattern begins with ``~`` then it is treated as a
    regular expression.

    If the pattern begins with ``!`` then it is negated.

    :param pattern: str pattern to limit by
    :param hosts: list of hosts to search against
    """
    # Return early if there's no special pattern
    if pattern == '*' or pattern == 'all':
        return hosts

    # If pattern is a regex, handle that and return
    if pattern[0] == '~':
        regex = re.compile(pattern[1:])
        return [target for target in hosts if regex.match(target)]

    patterns = []
    rpattern = pattern

    # Handle replacements of anything like [*:*] in pattern
    while(0 <= rpattern.find('[') < rpattern.find(':') < rpattern.find(']')):
        head, nrange, tail = rpattern.replace(
            '[', '|', 1).replace(']', '|', 1).split('|')

        beg, end = nrange.split(':')
        zfill = len(end) if (len(beg) > 0 and beg.startswith('0')) else 0

        if (zfill != 0 and len(beg) != len(end)) or beg > end:
            raise ValueError("Host range incorrectly specified")

        try:
            asc = string.ascii_letters
            seq = asc[asc.index(beg):asc.index(end) + 1]
        except ValueError:  # numeric range
            seq = range(int(beg), int(end) + 1)

        patterns = [''.join([head, str(i).zfill(zfill), tail]) for i in seq]
        rpattern = rpattern[rpattern.find(']') + 1:]

    # If there weren't range replacements, make pattern an array
    if len(patterns) == 0:
        patterns = [pattern]

    targets = []
    for pattern in patterns:
        # remove any leading '!'
        test_pattern = pattern.lstrip('!')

        # change '.' to literal period
        test_pattern = test_pattern.replace('.', '\.')

        # convert '*' to match a-Z, 0-9, _, -, or .
        test_pattern = test_pattern.replace('*', '[\w\.-]*')

        # Add beginning and end marks
        test_pattern = '^{}$'.format(test_pattern)

        regex = re.compile(test_pattern)

        targets.extend([host for host in hosts if regex.match(host)])

    # handle regation of patterns by inverting
    if pattern.startswith('!'):
        targets = list(set(targets) ^ set(hosts))

    return targets


class TargetList():
    """An abstract list of targets (lists of hosts)."""

    def __init__(self, key, cfg, limit_hosts=None, extra_paths=[]):
        """
        Constructor for target lists.

        :param key: str The primary configuration key to look for,
                        combined with the server_groups config
        :param cfg: dict Ordered dictionary of configuration
        :param limit_hosts: str A pattern to limit host names by. See
            limit_target_hosts for further information on the format
        :param extra_paths: list of extra paths to search for list files in
        """
        self.primary_key = key
        self.config = cfg
        self.limit_hosts = limit_hosts
        self.extra_paths = extra_paths
        self.deploy_groups = {}

    def _get_group_size(self, group):
        key = '{}_group_size'.format(group)
        size = self.config.get(key, self.config.get('group_size', None))

        return int(size) if size is not None else None

    def _get_server_groups(self):
        """Get the server_groups from configuration."""
        server_groups = self.config.get('server_groups', None)

        if server_groups is None:
            server_groups = ['default']
        else:
            server_groups = server_groups.split(',')

        return server_groups

    def _get_targets_for_key(self, key):
        """
        Get a list of hosts for a given configuration key.

        All child classes must implement this!

        :param key: str Combined key generated from a group + primary key
        """
        raise NotImplementedError

    def get_deploy_groups(self):
        """Get the list of targets and groups to deploy to."""
        if self.deploy_groups:
            return self.deploy_groups

        groups = collections.OrderedDict()
        all_hosts = []
        server_groups = self._get_server_groups()

        for group in server_groups:
            group = group.strip()
            cfg_key = group + '_' + self.primary_key
            limit = False
            if group == 'default':
                limit = True
                cfg_key = self.primary_key

            key = self.config.get(cfg_key, None)

            if key is None:
                raise RuntimeError(
                    'Reading `{0}` file "{1}" failed'.format(cfg_key, key))

            targets = self._get_targets_for_key(key)

            if limit and self.limit_hosts is not None:
                targets = limit_target_hosts(self.limit_hosts, targets)

            targets = list(set(targets) - set(all_hosts))
            all_hosts += targets

            group_size = self._get_group_size(group)

            if group_size is None or group_size >= len(targets):
                groups[group] = targets
            else:
                for i in xrange(0, len(targets), group_size):
                    group_key = group + str((i / group_size) + 1)
                    groups[group_key] = targets[i:i + group_size]

        self.deploy_groups = {'all_targets': all_hosts,
                              'deploy_groups': groups}
        return self.deploy_groups

    @property
    def groups(self):
        return self.get_deploy_groups()['deploy_groups']

    @property
    def all(self):
        return self.get_deploy_groups()['all_targets']


class DshTargetList(TargetList):
    """Fetch from list of hostnames. Historically kept in /etc/dsh/group."""

    def _get_targets_for_key(self, filename):
        """
        Get a list of hosts for a given filename.

        :param filename: str Combined filename key generated from a
            group + primary key
        """
        search_path = self.extra_paths
        # Historical reasons :p
        search_path.append('/etc/dsh/group')
        hosts_file = None

        if os.path.isabs(filename):
            hosts_file = filename
        else:
            for path in search_path:
                candidate = os.path.join(path, filename)
                if os.path.exists(candidate):
                    hosts_file = os.path.abspath(candidate)
                    break

        utils.check_file_exists(hosts_file)

        try:
            with open(hosts_file) as f:
                return re.findall(r'^[\w\.\-]+', f.read(), re.MULTILINE)
        except IOError as e:
            raise IOError(e.errno, e.strerror, hosts_file)
