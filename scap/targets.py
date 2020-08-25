# -*- coding: utf-8 -*-
"""
    scap.targets
    ~~~~~~~~
    Classes and helpers for getting a list of hosts to do things to

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import absolute_import

import collections
import os
import re
import string

import scap.utils as utils


def get(key, cfg, limit_hosts=None, extra_paths=None):
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
    while 0 <= rpattern.find('[') < rpattern.find(':') < rpattern.find(']'):
        head, nrange, tail = rpattern.replace(
            '[', '|', 1).replace(']', '|', 1).split('|')

        beg, end = nrange.split(':')
        zfill = len(end) if (beg and beg.startswith('0')) else 0

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
    if not patterns:
        patterns = [pattern]

    targets = []
    for a_pattern in patterns:
        # remove any leading '!'
        test_pattern = a_pattern.lstrip('!')

        # change '.' to literal period
        test_pattern = test_pattern.replace('.', r'\.')

        # convert '*' to match a-Z, 0-9, _, -, or .
        test_pattern = test_pattern.replace('*', r'[\w\.-]*')

        # Add beginning and end marks
        test_pattern = '^{}$'.format(test_pattern)

        regex = re.compile(test_pattern)

        targets.extend([host for host in hosts if regex.match(host)])

    # handle regation of patterns by inverting
    if pattern.startswith('!'):
        targets = list(set(targets) ^ set(hosts))

    return targets


class TargetList(object):
    """An abstract list of targets (lists of hosts)."""

    def __init__(self, key, cfg, limit_hosts=None, extra_paths=None):
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

    def _get_failure_limit(self, group):
        key = '{}_failure_limit'.format(group)
        return self.config.get(key, self.config.get('failure_limit', None))

    def _get_group_size(self, group):
        key = '{}_group_size'.format(group)
        size = self.config.get(key, self.config.get('group_size', None))

        return int(size) if size is not None else None

    def _get_filenames_for_groups(self):
        """
        Given the values in config and the primary key,
        return a hash {group_name: file_name} for the dsh groups
        """
        groups = collections.OrderedDict()
        server_groups = self.config.get('server_groups')
        if server_groups is None:
            server_groups = ['default']
        else:
            server_groups = server_groups.split(',')

        for group in server_groups:
            group = group.strip()
            if group == 'default':
                cfg_key = self.primary_key
            else:
                cfg_key = group + '_' + self.primary_key
            try:
                groups[group] = self.config[cfg_key]
            except KeyError:
                raise RuntimeError(
                    'Could not find config setting `{0}`'.format(cfg_key)
                )
        return groups

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

        for group, filename in self._get_filenames_for_groups().items():
            targets = self._get_targets_for_key(filename)

            if self.limit_hosts is not None:
                targets = limit_target_hosts(self.limit_hosts, targets)

            targets = list(set(targets) - set(all_hosts))

            if targets:
                all_hosts += targets

                size = self._get_group_size(group)
                failure_limit = self._get_failure_limit(group)

                groups[group] = DeployGroup(group, targets,
                                            size=size,
                                            failure_limit=failure_limit)

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
        search_path = []

        if self.extra_paths is not None:
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


class DirectDshTargetList(DshTargetList):
    """
    Read host lists from dsh files, minus the indirection.

    Usage:
    # This will read the dsh files named 'path/{a,b,c}'
    d = DirectDshTargetList('label', {'label': 'a,b,c'}, extra_path=path)
    """

    def _get_filenames_for_groups(self):
        server_groups = self.config.get(self.primary_key, None)
        if server_groups is None:
            return {}
        else:
            return {k: k.strip() for k in server_groups.split(',')}


class DeployGroup(object):
    def __init__(self, name, targets, size=None, failure_limit=None):
        """
        :param str name: Name of group
        :param list targets: Group target hosts
        :param int size: Size of deploy subgroups
        :param int|str failure_limit: String percentage or int number of
            acceptable failures (e.g. '30%' or 3). Defaults to 1
        """
        self.name = name
        self.targets = targets
        self.size = len(targets) if size is None else size

        if len(self.targets) < 1 or self.size < 1:
            raise ValueError('a deploy group must have at least one target')

        if failure_limit is None:
            failure_limit = 1
        elif isinstance(failure_limit, str):
            # Convert percentage strings (e.g. '30%') to number of targets
            if failure_limit.endswith('%'):
                failure_limit = float(failure_limit[:-1]) / 100
                failure_limit *= self.original_size

            failure_limit = int(failure_limit)

        self.failure_limit = failure_limit
        self.excludes = []

    def __iter__(self):
        return iter(self.targets)

    def exclude(self, target):
        """
        Excludes the given target from future iterations of `subgroups`.
        """
        self.excludes.append(target)

    def subgroups(self):
        """
        Generates each deployable subgroup and label according to size.

        :yields (str, list): Each subgroup label and list of targets
        """

        label = self.name
        targets = [host for host in self.targets if host not in self.excludes]

        for i in range(0, len(targets), self.size):
            if len(targets) > self.size:
                label = self.name + str((i // self.size) + 1)

            yield label, targets[i:i + self.size]

    @property
    def original_size(self):
        return len(self.targets)
