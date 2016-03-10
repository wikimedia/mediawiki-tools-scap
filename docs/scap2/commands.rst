#############
Sync Commands
#############

These commands, for the most part, use rsync to synchronize the state of the
mediawiki config and wmf/* code branches from the deploy server to the
production web servers. There are also commands to perform other related
house-keeping tasks as described in the individual command descriptions.

scap sync
---------
:command:`scap sync` is the driver script for syncing the MediaWiki versions and
configuration files currently staged on the deploy server to the rest of the
cluster.

.. program-output:: ../bin/scap sync --help
.. seealso::
   * :func:`scap.Scap`
   * :func:`scap.tasks.check_php_syntax`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_common`
   * :func:`scap.tasks.sync_wikiversions`


scap pull
---------
:command:`scap pull` uses rsync to fetch MediaWiki code and configuration to the
local host. It is typically called automatically on hosts during the execution of `scap sync`_.

.. program-output:: ../bin/scap pull --help
.. seealso::
   * :func:`scap.SyncCommon`
   * :func:`scap.tasks.sync_common`


scap sync-dir
-------------
:command:`scap sync-dir` synchronizes a directory from the staging directory to the
cluster.

.. program-output:: ../bin/scap sync-dir --help
.. seealso::
   * :func:`scap.SyncDir`


scap sync-file
--------------
:command:`scap sync-file` synchronizes a file from the staging directory to the cluster.

.. program-output:: ../bin/scap sync-file --help
.. seealso::
   * :func:`scap.SyncFile`


scap sync-l10n
--------------
:command:`scap sync-l10n` synchronizes the localization files for a given
MediaWiki version to the cluster and rebuilds the associated cache files.

.. program-output:: ../bin/scap sync-l10n --help
.. seealso::
   * :func:`scap.SyncL10n`

scap sync-wikiversions
----------------------
:command:`scap sync-wikiversions` compiles wikiversions.json into a CDB database and then
syncs both the JSON and CDB versions to the rest of the cluster.

.. program-output:: ../bin/scap sync-wikiversions --help
.. seealso::
   * :func:`scap.SyncWikiversions`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_wikiversions`


scap wikiversions-inuse
-----------------------
:command:`scap wikiversions-inuse` examines wikiversions.json to find the current active
MediaWiki versions.

.. program-output:: ../bin/scap wikiversions-inuse --help
.. seealso::
   * :func:`scap.MWVersionsInUse`


scap l10n-purge
---------------
:command:`scap l10n-purge` deletes localization files (CDB and JSON) across the
cluster.

.. program-output:: ../bin/scap l10n-purge --help
.. seealso::
   * :func:`scap.PurgeL10nCache`
   * :func:`scap.tasks.purge_l10n_cache`


scap wikiversions-compile
-------------------------
:command:`wikiversions-compile` compiles wikiversions.json into wikiversions.php.

.. program-output:: ../bin/scap wikiversions-compile --help
.. seealso::
   * :func:`scap.CompileWikiversions`
   * :func:`scap.tasks.compile_wikiversions`


scap cdb-rebuild
----------------
:command:`scap cdb-rebuild` rebuilds localization cache CDB files from JSON files.

.. program-output:: ../bin/scap cdb-rebuild --help
.. seealso::
   * :func:`scap.RebuildCdbs`
   * :func:`scap.tasks.merge_cdb_updates`


scap l10n-update
----------------
:command:`scap l10n-update` generates localization cache files.

.. program-output:: ../bin/scap l10n-update --help
.. seealso::
   * :func:`scap.UpdateL10n`
   * :func:`scap.tasks.update_localization_cache`


refreshCdbJsonFiles
-------------------
:command:`refreshCdbJsonFiles` Create JSON/MD5 files for all CDB files in a directory.

.. program-output:: ../bin/refreshCdbJsonFiles --help
.. seealso::
   * :func:`scap.refreshCdbJsonFiles`
   * :func:`scap.tasks.refresh_cdb_json_files`


