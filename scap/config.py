# -*- coding: utf-8 -*-
"""
    scap.config
    ~~~~~~~~~~~
    Configuration management

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
import ConfigParser
import getpass
import os
import socket

from . import utils


DEFAULT_CONFIG = {
    'bin_dir': (str, '/srv/deployment/scap/scap/bin'),
    'canary_dashboard_url': (
        str,
        'https://logstash.wikimedia.org/goto/3888cca979647b9381a7739b0bdbc88e'
    ),
    'canary_threshold': (float, 10.0),
    'canary_service': (str, 'mediawiki'),
    'canary_wait_time': (int, 20),
    'deploy_dir': (str, '/srv/mediawiki'),
    'failure_limit': (str, '0%'),
    'fancy_progress': (bool, False),
    'stage_dir': (str, '/srv/mediawiki-staging'),
    'lock_file': (str, None),
    'log_json': (bool, False),
    'logstash_host': (str, 'logstash1001.eqiad.wmnet:9200'),
    'master_rsync': (str, 'localhost'),
    'statsd_host': (str, '127.0.0.1'),
    'statsd_port': (str, '2003'),
    'tcpircbot_host': (str, None),
    'tcpircbot_port': (str, '9200'),
    'udp2log_host': (str, None),
    'udp2log_port': (str, '8420'),
    'wmf_realm': (str, 'production'),
    'ssh_user': (str, getpass.getuser()),
    'datacenter': (str, 'eqiad'),
    'hhvm_pid_file': (str, '/run/hhvm/hhvm.pid'),
    'apache_pid_file': (str, '/var/run/apache2/apache2.pid'),
    'dsh_targets': (str, 'mediawiki-installation'),
    'dsh_masters': (str, 'scap-masters'),
    'dsh_proxies': (str, 'scap-proxies'),
    'group_size': (int, None),
    'git_deploy_dir': (str, '/srv/deployment'),
    'git_fat': (bool, False),
    'git_server': (str, 'tin.eqiad.wmnet'),
    'git_scheme': (str, 'http'),
    'git_submodules': (bool, False),
    'git_upstream_submodules': (bool, False),
    'config_deploy': (bool, False),
    'nrpe_dir': (str, '/etc/nagios/nrpe.d'),
    'service_timeout': (float, 120.0),
    'tags_to_keep': (int, 20),
    'perform_checks': (bool, True),
    'patch_path': (str, None),
    'service_reload': (bool, False),
}


def load(cfg_file=None, environment=None, overrides=None):
    """
    Load configuration.

    A configuration file consists of sections, led by a ``[section]`` header
    and followed by ``name: value`` entries. Lines beginning with ``'#'`` are
    ignored and may be used to provide comments.

    A configuration file can contain multiple sections. The configuration
    object is populated with values from the ``global`` section and additional
    sections based on the fully qualified domain name of the local host. For
    example, on the host ``tin.eqiad.wmnet`` the final value for a given
    setting would be the first value found in sections: ``tin.eqiad.wmnet``,
    ``eqiad.wmnet``, ``wmnet`` or ``global``. Sections not present in the
    configuration file will be ignored.

    Configuration values are loaded from a file specified by the ``-c`` or
    ``--conf`` command-line options or from the default locations with the
    following hierarchy, sorted by override priority:

    #. ``$(pwd)/scap/environments/<environment>/scap.cfg`` or
       ``$(pwd)/scap/scap.cfg`` (if no environment was specified)
    #. ``/etc/scap.cfg``

    For example, if a configuration parameter is set in
    ``$(pwd)/scap/scap.cfg`` and that same parameter is set in
    ``/etc/scap.cfg`` the value for that parameter set in
    ``$(pwd)/scap/scap.cfg`` will be used during execution.

    :param cfg_file: Alternate configuration file
    :param environment: the string path under which scap.cfg is found
    :param overrides: Dict of configuration values
    :returns: dict of configuration values
    """
    local_cfg = os.path.join(os.getcwd(), 'scap')

    parser = ConfigParser.SafeConfigParser()
    if cfg_file:
        try:
            cfg_file = open(cfg_file)
        except TypeError:
            # Assume that cfg_file is already an open file
            pass

        parser.readfp(cfg_file)
    else:
        parser.read([
            '/etc/scap.cfg',
            os.path.join(local_cfg, 'scap.cfg'),
            utils.get_env_specific_filename(
                os.path.join(local_cfg, 'scap.cfg'),
                environment
            )
        ])

    fqdn = socket.getfqdn().split('.')
    sections = ['global']
    sections += ['.'.join(fqdn[l:]) for l in range(0, len(fqdn))][::-1]

    config = {key: value for key, (_, value) in DEFAULT_CONFIG.iteritems()}

    for section in sections:
        if parser.has_section(section):
            # Do not interpolate items in the section.
            # Fixes crash on tin: 'int' object has no attribute 'find'
            for key, value in parser.items(section, True):
                config[key] = coerce_value(key, value)

    config = override_config(config, overrides)

    if not environment and config.get('environment', None):
            return load(cfg_file, config.get('environment'), overrides)

    config['environment'] = environment
    return config


def override_config(config, overrides=None):
    """Override values in a config with type-coerced values."""
    if overrides:
        for key, value in overrides.iteritems():
            config[key] = coerce_value(key, value)

    return config


def coerce_value(key, value):
    """Coerce the given value based on the default config type."""

    if key in DEFAULT_CONFIG:
        default_type, _ = DEFAULT_CONFIG[key]

        if (isinstance(value, default_type)):
            return value

        if default_type == bool:
            lower = value.lower()

            # Accept the same bool values accepted by ConfigParser
            if lower in ['1', 'yes', 'true', 'on']:
                return True
            elif lower in ['0', 'no', 'false', 'off']:
                return False
            else:
                msg = "invalid boolean value '{}'".format(value)
                raise ValueError(msg)

        else:
            return default_type(value)

    return value
