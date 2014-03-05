#######
Scripts
#######

.. _scap:

scap
====
**scap** is the driver script for syncing the MediaWiki versions and
configuration files currently staged on the deploy server to the rest of the
cluster.

.. program-output:: ../bin/scap --help
.. seealso::
   * :func:`scap.Scap`
   * :func:`scap.tasks.scap`


sync-common
===========
**sync-common** uses rsync to fetch MediaWiki code and configuration to the
local host. It is typically called automatically on hosts during the execution of scap_.

.. program-output:: ../bin/sync-common --help
.. seealso::
   * :func:`scap.SyncCommon`
   * :func:`scap.tasks.sync_common`


sync-wikiversions
=================
**sync-wikiversions** compiles wikiversions.json into a CDB database and then
syncs both the JSON and CDB versions to the rest of the cluster.

.. program-output:: ../bin/sync-wikiversions --help
.. seealso::
   * :func:`scap.SyncWikiversions`
   * :func:`scap.tasks.compile_wikiversions_cdb`
   * :func:`scap.tasks.sync_wikiversions`


mwversionsinuse
===============
**mwversionsinuse** examines wikiversions.json to find the current active
MediaWiki versions.

.. program-output:: ../bin/mwversionsinuse --help
.. seealso::
   * :func:`scap.MWVersionsInUse`
   * :func:`scap.utils.wikiversions`
