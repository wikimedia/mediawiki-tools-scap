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
import getpass
import os
import socket
from configparser import ConfigParser

import scap.utils as utils


DEFAULT_CONFIG = {
    "beta_only_config_files": (str, ""),
    "block_execution": (bool, False),
    "canary_dashboard_url": (str, "https://logstash.wikimedia.org"),
    "canary_threshold": (float, 10.0),
    "canary_service": (str, "mediawiki"),
    "canary_wait_time": (int, 20),
    "delay_messageblobstore_purge": (bool, False),
    "deploy_dir": (str, "/srv/mediawiki"),
    "failure_limit": (str, "0%"),
    "fancy_progress": (bool, False),
    "gerrit_url": (str, "https://gerrit.wikimedia.org/r/"),
    "gerrit_push_url": (str, "ssh://gerrit.wikimedia.org:29418/"),
    "gerrit_push_user": (str, None),
    "php_version": (str, "php"),
    "keyholder_key": (str, None),
    "stage_dir": (str, "/srv/mediawiki-staging"),
    "history_log": (str, "/srv/mediawiki-staging/scap/log/history.log"),
    "lock_file": (str, None),
    "log_json": (bool, False),
    "logstash_host": (str, "logstash1001.eqiad.wmnet:9200"),
    "mw_web_clusters": (str, "jobrunner,appserver,appserver_api,testserver"),
    "master_rsync": (str, "localhost"),
    "statsd_host": (str, "127.0.0.1"),
    "statsd_port": (str, "2003"),
    "tcpircbot_host": (str, None),
    "tcpircbot_port": (str, "9200"),
    "udp2log_host": (str, None),
    "udp2log_port": (str, "8420"),
    "wmf_realm": (str, "production"),
    "ssh_user": (str, getpass.getuser()),
    "datacenter": (str, "eqiad"),
    "dsh_testservers": (str, "testserver"),
    "dsh_targets": (str, "mediawiki-installation"),
    "dsh_masters": (str, "scap-masters"),
    "dsh_proxies": (str, "scap-proxies"),
    "group_size": (int, None),
    "git_deploy_dir": (str, "/srv/deployment"),
    "git_fat": (bool, False),
    "git_binary_manager": (str, None),
    "git_server": (str, "deploy1001.eqiad.wmnet"),
    "git_scheme": (str, "http"),
    "git_submodules": (bool, False),
    "git_upstream_submodules": (bool, False),
    "config_deploy": (bool, False),
    "mediawiki_canary_swagger_url": (str, "http://en.wikipedia.org"),
    "mediawiki_canary_swagger_spec_path": (str, "/spec.yaml"),
    "operations_mediawiki_config_branch": (str, "master"),
    "nrpe_dir": (str, "/etc/nagios/nrpe.d"),
    "require_valid_service": (bool, False),
    "scap3_mediawiki": (bool, False),
    "service_timeout": (float, 120.0),
    "tags_to_keep": (int, 20),
    "perform_checks": (bool, True),
    "patch_path": (str, "/srv/patches"),
    "php7_admin_port": (int, None),
    "php_fpm_opcache_threshold": (int, 100),
    "php_fpm_always_restart": (bool, False),
    "php_l10n": (bool, False),  # Feature flag for PHP l10n file generation
    "cache_revs": (int, 5),
    "use_syslog": (bool, False),
    "umask": (int, 0o002),
    "train_blockers_url": (str, "https://train-blockers.toolforge.org/api.php"),
    # HTTP/HTTPS proxy used only for requests that need it (such as accessing train_blockers_url)
    "web_proxy": (str, None),

    # Settings related to building and deploying mediawiki container image
    # xref AbstractSync._build_container_images() in main.py
    "build_mw_container_image": (bool, False),
    "full_image_build": (bool, False),
    "deploy_mw_container_image": (bool, False),
    "release_repo_dir": (str, None),
    "release_repo_update_cmd": (str, None),

    # This command will run with the current directory set to
    # {release_repo_dir}/make-container-image.  It will be passed some
    # additional parameters.
    "release_repo_build_and_push_images_cmd": (str, "make -f Makefile build-and-push-all-images"),

    "docker_registry": (str, "docker-registry.discovery.wmnet"),
    "mediawiki_image_name": (str, "restricted/mediawiki-multiversion"),
    "mediawiki_debug_image_name": (str, "restricted/mediawiki-multiversion-debug"),
    "webserver_image_name": (str, "restricted/mediawiki-webserver"),
    "mediawiki_image_extra_packages": (str, ""),
    # Path to a CA cert to inject into the image
    "mediawiki_image_extra_ca_cert": (str, None),

    "helmfile_services_dir": (str, "/srv/deployment-charts/helmfile.d/services"),
    "helmfile_mediawiki_release_dir": (str, "/etc/helmfile-defaults/mediawiki/release"),
    # Comma separated list of the clusters we will deploy to
    "k8s_clusters": (str, "eqiad, codfw"),
    "k8s_deployments_file": (str, "/etc/helmfile-defaults/mediawiki-deployments.yaml"),
    # End settings related to building and deploying mediawiki container image

    # Settings related to scap installation
    "install_ssh_user": (str, "scap"),
    "scap_source_dir": (str, "/srv/deployment/scap"),
    "scap_targets": (str, "scap_targets"),
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
    example, on the host ``deployXXXX.eqiad.wmnet`` the final value for a given
    setting would be the first value found in sections:
    ``deployXXXX.eqiad.wmnet``, ``eqiad.wmnet``, ``wmnet`` or ``global``.
    Sections not present in the configuration file will be ignored.

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
    local_cfg = os.path.join(os.getcwd(), "scap")

    parser = ConfigParser()
    if cfg_file:
        try:
            cfg_file = open(cfg_file)
        except TypeError:
            # Assume that cfg_file is already an open file
            pass

        if hasattr(parser, "read_file"):
            parser.read_file(cfg_file)
        else:
            parser.readfp(cfg_file)
    else:
        if environment and not os.path.exists(
            os.path.join(local_cfg, "environments", environment)
        ):
            raise RuntimeError("Environment {} does not exist!".format(environment))

        parser.read(
            [
                "/etc/scap.cfg",
                os.path.join(local_cfg, "scap.cfg"),
                utils.get_env_specific_filename(
                    os.path.join(local_cfg, "scap.cfg"), environment
                ),
            ]
        )

    fqdn = socket.getfqdn().rstrip('.').split(".")
    sections = ["global"]
    sections += [".".join(fqdn[x:]) for x in range(0, len(fqdn))][::-1]

    config = {key: value for key, (_, value) in DEFAULT_CONFIG.items()}

    for section in sections:
        if parser.has_section(section):
            # Do not interpolate items in the section.
            # Fixes crash on deployment server:
            #   'int' object has no attribute 'find'
            for key, value in parser.items(section, True):
                config[key] = coerce_value(key, value)

    config = override_config(config, overrides)

    if not environment and config.get("environment", None):
        return load(cfg_file, config.get("environment"), overrides)

    config["environment"] = environment
    if cfg_file:
        cfg_file.close()
    return config


def override_config(config, overrides=None):
    """Override values in a config with type-coerced values."""
    if overrides:
        for key, value in overrides.items():
            config[key] = coerce_value(key, value)

    return config


def coerce_value(key, value):
    """Coerce the given value based on the default config type."""

    if key in DEFAULT_CONFIG:
        default_type, _ = DEFAULT_CONFIG[key]

        if isinstance(value, default_type):
            return value

        if default_type == bool:
            lower = value.lower()

            # Accept the same bool values accepted by ConfigParser
            if lower in ["1", "yes", "true", "on"]:
                return True
            if lower in ["0", "no", "false", "off"]:
                return False
            msg = "invalid boolean value '{}'".format(value)
            raise ValueError(msg)

        else:
            return default_type(value)

    return value


def multi_value(str_value):
    """
    Given a string that's got commas, turn it into a list

    :param str_value: Random thing the user typed in config
    """
    comma_list = [x.strip() for x in str_value.split(",")]
    return comma_list
