#######
Scripts
#######

.. _scap:

deploy
======
The **deploy** command handles deployment of various wikimedia projects from
the deployment server to a staging or production environment.

* Tags the current revision in the current git directory
* Runs `git update-server-info` for the current git directory (as well
  as for any submodules)
* SSHs into each server in the `dsh_targets` file
* Runs git fetch in the `/srv/deployment/{repo}` directory of each target
  (running checkout if it does not exist)
* Checks out the tag created in step 1 on each of the target machines
* If a `service_name` is specified, the service is restarted
* If a `service_port` is specified, make sure that it is accepting
  connections, waiting up to `service_timeout` (120 seconds by default)

.. program-output:: ../bin/sync-common --help
.. seealso::
   * :class:`scap.Deploy`


scap
====
**scap** is the driver script for syncing the MediaWiki versions and
configuration files currently staged on the deploy server to the rest of the
cluster.

.. program-output:: ../bin/scap --help
.. seealso::
   * :func:`scap.Scap`
   * :func:`scap.tasks.check_php_syntax`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_common`
   * :func:`scap.tasks.sync_wikiversions`


sync-common
===========
**sync-common** uses rsync to fetch MediaWiki code and configuration to the
local host. It is typically called automatically on hosts during the execution of scap_.

.. program-output:: ../bin/sync-common --help
.. seealso::
   * :func:`scap.SyncCommon`
   * :func:`scap.tasks.sync_common`


sync-dblist
===========
**sync-dblist** synchronizes dblist files to the cluster.

.. program-output:: ../bin/sync-dblist --help
.. seealso::
   * :func:`scap.SyncDblist`


sync-dir
========
**sync-dir** synchronizes a directory from the staging directory to the
cluster.

.. program-output:: ../bin/sync-dir --help
.. seealso::
   * :func:`scap.SyncDir`


sync-docroot
============
**sync-docroot** synchronizes common/docroot and common/w to the cluster.

.. program-output:: ../bin/sync-docroot --help
.. seealso::
   * :func:`scap.SyncDocroot`


sync-file
=========
**sync-file** synchronizes a file from the staging directory to the cluster.

.. program-output:: ../bin/sync-file --help
.. seealso::
   * :func:`scap.SyncFile`


sync-wikiversions
=================
**sync-wikiversions** compiles wikiversions.json into a CDB database and then
syncs both the JSON and CDB versions to the rest of the cluster.

.. program-output:: ../bin/sync-wikiversions --help
.. seealso::
   * :func:`scap.SyncWikiversions`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_wikiversions`


mwversionsinuse
===============
**mwversionsinuse** examines wikiversions.json to find the current active
MediaWiki versions.

.. program-output:: ../bin/mwversionsinuse --help
.. seealso::
   * :func:`scap.MWVersionsInUse`


scap-purge-l10n-cache
=====================
**scap-purge-l10n-cache** deletes localization files (CDB and JSON) across the
cluster.

.. program-output:: ../bin/scap-purge-l10n-cache --help
.. seealso::
   * :func:`scap.PurgeL10nCache`
   * :func:`scap.tasks.purge_l10n_cache`


compile-wikiversions
====================
**compile-wikiversions** compiles wikiversions.json into wikiversions.cdb.

.. program-output:: ../bin/compile-wikiversions --help
.. seealso::
   * :func:`scap.CompileWikiversions`
   * :func:`scap.tasks.compile_wikiversions`


scap-rebuild-cdbs
=================
**scap-rebuild-cdbs** rebuilds localization cache CDB files from JSON files.

.. program-output:: ../bin/scap-rebuild-cdbs --help
.. seealso::
   * :func:`scap.RebuildCdbs`
   * :func:`scap.tasks.merge_cdb_updates`


mw-update-l10n
==============
**mw-update-l10n** generates localization cache files.

.. program-output:: ../bin/mw-update-l10n --help
.. seealso::
   * :func:`scap.UpdateL10n`
   * :func:`scap.tasks.update_localization_cache`
