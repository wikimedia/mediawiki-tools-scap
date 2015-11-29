#############
Sync Commands
#############

These commands, for the most part, use rsync to synchronize the state of the
mediawiki config and wmf/* code branches from the deploy server to the
production web servers. There are also commands to perform other related
house-keeping tasks as described in the individual command descriptions.

scap
----
:command:`scap` is the driver script for syncing the MediaWiki versions and
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
-----------
:command:`sync-common` uses rsync to fetch MediaWiki code and configuration to the
local host. It is typically called automatically on hosts during the execution of scap_.

.. program-output:: ../bin/sync-common --help
.. seealso::
   * :func:`scap.SyncCommon`
   * :func:`scap.tasks.sync_common`


sync-dir
-----------
:command:`sync-dir` synchronizes a directory from the staging directory to the
cluster.

.. program-output:: ../bin/sync-dir --help
.. seealso::
   * :func:`scap.SyncDir`


sync-docroot
------------
:command:`sync-docroot` synchronizes common/docroot and common/w to the cluster.

.. program-output:: ../bin/sync-docroot --help
.. seealso::
   * :func:`scap.SyncDocroot`


sync-file
---------
:command:`sync-file` synchronizes a file from the staging directory to the cluster.

.. program-output:: ../bin/sync-file --help
.. seealso::
   * :func:`scap.SyncFile`


sync-l10n
---------
:command:`sync-l10n` synchronizes the localization files for a given
MediaWiki version to the cluster and rebuilds the associated cache files.

.. program-output:: ../bin/sync-l10n --help
.. seealso::
   * :func:`scap.SyncL10n`

sync-wikiversions
-----------------
:command:`sync-wikiversions` compiles wikiversions.json into a CDB database and then
syncs both the JSON and CDB versions to the rest of the cluster.

.. program-output:: ../bin/sync-wikiversions --help
.. seealso::
   * :func:`scap.SyncWikiversions`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_wikiversions`


mwversionsinuse
---------------
:command:`mwversionsinuse` examines wikiversions.json to find the current active
MediaWiki versions.

.. program-output:: ../bin/mwversionsinuse --help
.. seealso::
   * :func:`scap.MWVersionsInUse`


scap-purge-l10n-cache
---------------------
:command:`scap-purge-l10n-cache` deletes localization files (CDB and JSON) across the
cluster.

.. program-output:: ../bin/scap-purge-l10n-cache --help
.. seealso::
   * :func:`scap.PurgeL10nCache`
   * :func:`scap.tasks.purge_l10n_cache`


compile-wikiversions
--------------------
:command:`compile-wikiversions` compiles wikiversions.json into wikiversions.php.

.. program-output:: ../bin/compile-wikiversions --help
.. seealso::
   * :func:`scap.CompileWikiversions`
   * :func:`scap.tasks.compile_wikiversions`


scap-rebuild-cdbs
-----------------
:command:`scap-rebuild-cdbs` rebuilds localization cache CDB files from JSON files.

.. program-output:: ../bin/scap-rebuild-cdbs --help
.. seealso::
   * :func:`scap.RebuildCdbs`
   * :func:`scap.tasks.merge_cdb_updates`


mw-update-l10n
--------------
:command:`mw-update-l10n` generates localization cache files.

.. program-output:: ../bin/mw-update-l10n --help
.. seealso::
   * :func:`scap.UpdateL10n`
   * :func:`scap.tasks.update_localization_cache`
