# -*- coding: utf-8 -*-
#
# Copyright (c) 2014 Wikimedia Foundation and contributors
"""
  Heterogeneous deployment script
  Deploys MediaWiki code and configuration to a group of servers via rsync.

"""
from .scap import scap
from . import log

__version__ = '0.0.1'

__all__ = [
    'scap',
]

log.setup_loggers()
