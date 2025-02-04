#############
Sync Commands
#############

These commands, for the most part, use rsync to synchronize the state of the
mediawiki config and wmf/* code branches from the deploy server to the
production web servers. There are also commands to perform other related
house-keeping tasks as described in the individual command descriptions.

scap sync-world
---------------
:command:`scap sync-world` is the driver script for syncing the MediaWiki versions and
configuration files currently staged on the deploy server to the rest of the
cluster.

.. program-output:: python3 ../bin/run-dev-scap sync-world --help
.. seealso::
   * :func:`scap.ScapWorld`
   * :func:`scap.tasks.check_valid_syntax`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_common`
   * :func:`scap.tasks.sync_wikiversions`


scap pull
---------
:command:`scap pull` uses rsync to fetch MediaWiki code and configuration to the
local host. It is typically called automatically on hosts during the execution of `scap sync-world`_.

.. program-output:: python3 ../bin/run-dev-scap pull --help
.. seealso::
   * :func:`scap.SyncCommon`
   * :func:`scap.tasks.sync_common`


scap sync-file
--------------
:command:`scap sync-file` synchronizes a file or directory from the staging
directory to the cluster.

.. program-output:: python3 ../bin/run-dev-scap sync-file --help
.. seealso::
   * :func:`scap.SyncFile`


scap sync-wikiversions
----------------------
:command:`scap sync-wikiversions` compiles wikiversions.json into a CDB database and then
syncs both the JSON and CDB versions to the rest of the cluster.

.. program-output:: python3 ../bin/run-dev-scap sync-wikiversions --help
.. seealso::
   * :func:`scap.SyncWikiversions`
   * :func:`scap.tasks.compile_wikiversions`
   * :func:`scap.tasks.sync_wikiversions`


##############
Misc. Commands
##############

scap wikiversions-inuse
-----------------------
:command:`scap wikiversions-inuse` examines wikiversions.json to find the current active
MediaWiki versions.

.. program-output:: python3 ../bin/run-dev-scap wikiversions-inuse --help
.. seealso::
   * :func:`scap.MWVersionsInUse`


scap wikiversions-compile
-------------------------
:command:`wikiversions-compile` compiles wikiversions.json into wikiversions.php.

.. program-output:: python3 ../bin/run-dev-scap wikiversions-compile --help
.. seealso::
   * :func:`scap.CompileWikiversions`
   * :func:`scap.tasks.compile_wikiversions`


scap cdb-rebuild
----------------
:command:`scap cdb-rebuild` rebuilds localization cache CDB files from JSON files.

.. program-output:: python3 ../bin/run-dev-scap cdb-rebuild --help
.. seealso::
   * :func:`scap.RebuildCdbs`
   * :func:`scap.tasks.merge_cdb_updates`


scap cdb-json-refresh
---------------------
:command:`refreshCdbJsonFiles` Create JSON/MD5 files for all CDB files in a directory.

.. program-output:: python3 ../bin/run-dev-scap cdb-json-refresh --help
.. seealso::
   * :func:`scap.refreshCdbJsonFiles`
   * :func:`scap.tasks.refresh_cdb_json_files`
