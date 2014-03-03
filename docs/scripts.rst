#######
Scripts
#######

.. _scap:

scap
====
**scap** is the driver script for syncing the MediaWiki versions and
configuration files currently staged on ``tin.eqiad.wmnet`` to the rest of the
production cluster.

.. program-output:: ../bin/scap --help
.. seealso::
   * :func:`scap.scap`
   * :func:`scap.tasks.scap`


sync-common
===========
**sync-common** uses rsync to fetch MediaWiki code and configuration to the
local host. It is typically called automatically on hosts during the execution of scap_.

.. program-output:: ../bin/sync-common --help
.. seealso::
   * :func:`scap.sync_common`
   * :func:`scap.tasks.sync_common`


mwversionsinuse
===============
**mwversionsinuse** examines wikiversions.json to find the current active
MediaWiki versions.

.. program-output:: ../bin/mwversionsinuse --help
.. seealso::
   * :func:`scap.MWVersionsInUse`
   * :func:`scap.utils.wikiversions`
