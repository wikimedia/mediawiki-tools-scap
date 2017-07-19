# -*- coding: utf-8 -*-
"""
    scap.pooler
    ~~~~~~~~~~~
    Wrapper for our pooling/depooling actions, handled via conftool

"""

import re

from conftool.drivers import BackendError
from conftool.kvobject import KVObject
from conftool import configuration, node


class Pooler(object):
    """
    Wrapper to the conftool for our common (de)pooling operations
    """

    def __init__(self, config_file, servers):
        """
        Read the conftool configuration and load all the provided nodes

        :param config_file: The path to a config file for conftool
        :param servers: Which servers to work with
        """
        KVObject.setup(configuration.get(config_file))
        node_re = re.compile('({name})'.format(name='|'.join(servers)))
        self.nodes = [n for n in node.Node.query({'name': node_re})]

    def pool(self):
        """Pool services"""
        return self._set_state('yes')

    def depool(self):
        """Depool services"""
        return self._set_state('no')

    def inactive(self):
        """Set services to inactive"""
        return self._set_state('inactive')

    def _set_state(self, state):
        """Set a provided state for all the nodes"""
        results = {'success': [], 'failed': []}
        for machine in self.nodes:
            try:
                machine.update({'pooled': state})
                results['success'].append(machine.name)
            except BackendError:
                results['failed'].append(machine.name)

        return results
