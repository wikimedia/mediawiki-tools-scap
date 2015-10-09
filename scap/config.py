# -*- coding: utf-8 -*-
"""
    scap.config
    ~~~~~~~~~~~
    Configuration management

"""
import ConfigParser
import getpass
import os
import socket

from . import utils


DEFAULT_CONFIG = {
    'bin_dir': '/srv/deployment/scap/scap/bin',
    'deploy_dir': '/srv/mediawiki',
    'stage_dir': '/srv/mediawiki-staging',
    'lock_file': '/var/lock/scap',
    'log_json': False,
    'master_rsync': 'localhost',
    'statsd_host': '127.0.0.1',
    'statsd_port': '2003',
    'tcpircbot_host': None,
    'tcpircbot_port': '9200',
    'udp2log_host': None,
    'udp2log_port': '8420',
    'wmf_realm': 'production',
    'ssh_user': getpass.getuser(),
    'datacenter': 'eqiad',
    'hhvm_pid_file': '/run/hhvm/hhvm.pid',
    'apache_pid_file': '/var/run/apache2/apache2.pid',
    'pybal_interface': 'lo:LVS',
    'dsh_targets': 'mediawiki-installation',
    'git_deploy_dir': '/srv/deployment',
    'git_repo_user': 'mwdeploy',
    'git_server': 'tin.eqiad.wmnet',
    'git_scheme': 'http',
    'git_submodules': False,
    'config_deploy': False,
    'nrpe_dir': '/etc/nagios/nrpe.d',
    'service_timeout': 120,
    'perform_checks': True,
}


def load(cfg_file=None, environment=None, overrides=None):
    """Load configuration.

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
    #. ``/srv/scap/scap.cfg``
    #. ``<path to scap python package>/../scap.cfg``

    For example, if a configuration parameter is set in ``/srv/scap/scap.cfg``
    and that same parameter is set in ``/etc/scap.cfg`` the value for that
    parameter set in ``/etc/scap.cfg`` will be used during execution.

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
            os.path.join(os.path.dirname(__file__), '..', 'scap.cfg'),
            '/srv/scap/scap.cfg',
            '/etc/scap.cfg',
            utils.get_env_specific_filename(
                os.path.join(local_cfg, 'scap.cfg'),
                environment
            )
        ])

    fqdn = socket.getfqdn().split('.')
    sections = ['global']
    sections += ['.'.join(fqdn[l:]) for l in range(0, len(fqdn))][::-1]

    config = {}
    config.update(DEFAULT_CONFIG)
    for section in sections:
        if parser.has_section(section):
            # Do not interpolate items in the section.
            # Fixes crash on tin: 'int' object has no attribute 'find'
            config.update(parser.items(section, True))

    if overrides:
        config.update(overrides)

    return config
