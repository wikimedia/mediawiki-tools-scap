
.. _configuration:

######################
Configuring a Git Repo
######################

Since version 3, *Scap* is now able to deploy any git-based repository from tin
to any number of hosts. This deployment can happen in serial or in parallel. All
that is necessary, aside from the configuration outlined here, is that the target
hosts are accessible via SSH by the ``deploy_user`` from the ``deploy_host``
(the machine from which you run Scap).

Scap configuration is loaded from several files via :func:`scap.config.load`
function in the :mod:`scap.config` module.

.. autofunction:: scap.config.load


Simple initial setup
~~~~~~~~~~~~~~~~~~~~

For a new repo setup, the main file that needs to be created is in the root of
the repository at ``scap/scap.cfg``. This new file should be made in
``ConfigParser`` format.

.. warning::
   These values **must be** set in ``scap/scap.cfg``:
    - ``git_repo``
    - ``dsh_targets``.

.. _available-configuration:

Available configuration variables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
+----------------------------+---------------------------+---------------------------------+
| Value                      | Default                   | Explanation                     |
+============================+===========================+=================================+
| ``git_server``             | ``tin.eqiad.wmnet``       | (*String*) Server from          |
|                            |                           | which code is fetched           |
+----------------------------+---------------------------+---------------------------------+
| ``ssh_user``               | your username             | (*String*) User as whom         |
|                            |                           | to ssh to target hosts and      |
|                            |                           | execute remote commands         |
+----------------------------+---------------------------+---------------------------------+
| ``git_repo``               | **NONE**                  | (*String*) Repo on              |
|                            |                           | ``git_server``                  |
|                            |                           |                                 |
+----------------------------+---------------------------+---------------------------------+
| ``git_deploy_dir``         | ``/srv/deployment``       | (*String*) Directory on         |
|                            |                           | targets in which                |
|                            |                           | your repo is placed             |
+----------------------------+---------------------------+---------------------------------+
| ``dsh_targets``            | ``mediawiki-installation``| (*String*) Path to list         |
|                            |                           | of deploy targets. If           |
|                            |                           | the path is not absolute,       |
|                            |                           | Scap looks for a file in        |
|                            |                           | ``/etc/dsh/group``. For         |
|                            |                           | an absolute path, it just       |
|                            |                           | uses the full path to           |
|                            |                           | the file.                       |
+----------------------------+---------------------------+---------------------------------+
| ``server_groups``          | **NONE**                  | (*String*) (*Optional*)         |
|                            |                           | If this option is defined,      |
|                            |                           | Scap will look deploy to        |
|                            |                           | these *groups* of servers       |
|                            |                           | in order.                       |
|                            |                           |                                 |
|                            |                           | For example:                    |
|                            |                           | ``server_groups: 'one, default``|
|                            |                           | will cause Scap to look in      |
|                            |                           | the ``scap.cfg`` file for       |
|                            |                           | both a ``one_dsh_targets``      |
|                            |                           | file and a ``dsh_targets``      |
|                            |                           | file (the `default`). A full    |
|                            |                           | deploy will run for hosts       |
|                            |                           | defined in ``one_dsh_targets``, |
|                            |                           | then a full deploy will run for |
|                            |                           | hosts defined in ``dsh_targets``|
|                            |                           | (any hosts defined in both      |
|                            |                           | will be deployed with the       |
|                            |                           | first group--``can``            |
|                            |                           | in this example)                |
+----------------------------+---------------------------+---------------------------------+
| ``group_size``             | **NONE**                  | (*Int*) (*Optional*)            |
| ``[group]_group_size``     |                           | If defined, Scap will split     |
|                            |                           | each server group into smaller  |
|                            |                           | subgroups containing no more    |
|                            |                           | than the given number of hosts. |
|                            |                           | A global and/or group specific  |
|                            |                           | configuration may be provided.  |
|                            |                           |                                 |
|                            |                           | This configuration can be used  |
|                            |                           | to achieve a more serial        |
|                            |                           | deployment within each server   |
|                            |                           | group.                          |
+----------------------------+---------------------------+---------------------------------+
| ``failure_limit``          | 1                         | (*Int* or *String*) (*Optional*)|
| ``[group]_failure_limit``  |                           | Number or percentage of group   |
|                            |                           | targets that are allowed to     |
|                            |                           | fail without triggering a       |
|                            |                           | rollback. Percentages should be |
|                            |                           | suffixed with '%' (e.g. 10%).   |
|                            |                           |                                 |
|                            |                           | A global and/or group specific  |
|                            |                           | configuration may be provided.  |
+----------------------------+---------------------------+---------------------------------+
| ``git_submodules``         | False                     | (*Boolean*) (*Optional*)        |
|                            |                           | Whether submodules need         |
|                            |                           | to be fetched and               |
|                            |                           | checked-out as part of          |
|                            |                           | the deploy on targets.          |
+----------------------------+---------------------------+---------------------------------+
| ``git_fat``                | False                     | (*Boolean*) (*Optional*)        |
|                            |                           | Whether binary files are managed|
|                            |                           | via git-fat and should be       |
|                            |                           | synced as part of the deploy on |
|                            |                           | targets.                        |
+----------------------------+---------------------------+---------------------------------+
| ``service_name``           | **NONE**                  | (*String*) (*Optional*)         |
|                            |                           | If a service name is            |
|                            |                           | defined, the service            |
|                            |                           | will be restarted on            |
|                            |                           | each target as part             |
|                            |                           | of the ``promote``              |
|                            |                           | stage.                          |
|                            |                           |                                 |
|                            |                           | May also be a comma-separated   |
|                            |                           | list of services to restart     |
|                            |                           | (e.g. ``service1, service2``).  |
|                            |                           |                                 |
|                            |                           | Each service may be reloaded    |
|                            |                           | instead of restarted by         |
|                            |                           | appending ``=reload`` to the    |
|                            |                           | service name (e.g.,             |
|                            |                           | ``service1, service2=reload``)  |
+----------------------------+---------------------------+---------------------------------+
| ``service_port``           | **NONE**                  | (*Int*) (*Optional*)            |
|                            |                           | If a port is defined,           |
|                            |                           | Scap will verify that           |
|                            |                           | the port is accepting TCP       |
|                            |                           | connections after the           |
|                            |                           | ``promote`` deploy stage.       |
|                            |                           | (Timeout defined by             |
|                            |                           | ``service_timeout``)            |
+----------------------------+---------------------------+---------------------------------+
| ``service_timeout``        | 120                       | (*Int*) (*Optional*) The        |
|                            |                           | amount of time to wait          |
|                            |                           | when checking a                 |
|                            |                           | ``service_port``                |
|                            |                           | for accepting TCP               |
|                            |                           | connections.                    |
+----------------------------+---------------------------+---------------------------------+
| ``tags_to_keep``           | 20                        | (*Int*) (*Optional*)            |
|                            |                           | Number of tags to keep in the   |
|                            |                           | deployment server repo. Git     |
|                            |                           | appears to max-out at 999.      |
|                            |                           | Scap thinks 20 tags on the      |
|                            |                           | deployment server is quite      |
|                            |                           | enough.                         |
+----------------------------+---------------------------+---------------------------------+
| ``batch_size``             | 80                        | (*Int*) (*Optional*)            |
| ``[stage]_batch_size``     |                           | Parallelism  of a stage of      |
|                            |                           | deployment Number of hosts to   |
|                            |                           | execute a particular            |
|                            |                           | deployment stage on             |
|                            |                           | simultaniously. This            |
|                            |                           | is configurable by              |
|                            |                           | stage by creating               |
|                            |                           | a config variable:              |
|                            |                           | ``[stage]_batch_size``          |
+----------------------------+---------------------------+---------------------------------+
| ``config_deploy``          | False                     | (*Boolean*) (*Optional*)        |
|                            |                           | if ``True``, the                |
|                            |                           | ``./scap/config-files.yaml``    |
|                            |                           | file will be parsed, any        |
|                            |                           | templates defined inside        |
|                            |                           | will be evaluated with jinja2   |
|                            |                           | and deployed.                   |
+----------------------------+---------------------------+---------------------------------+
| ``git_upstream_submodules``| False                     | (*Boolean*) If ``True``,        |
|                            |                           | submodules will **NOT** be      |
|                            |                           | fetched from                    |
|                            |                           | ``git_deploy_server``,          |
|                            |                           | but from the git server         |
|                            |                           | defined in ``.gitmodules``      |
+----------------------------+---------------------------+---------------------------------+
| ``nrpe_dir``               | ``/etc/nagios/nrpe.d``    | (*String*) Directory in         |
|                            |                           | which nrpe checks are           |
|                            |                           | stored                          |
+----------------------------+---------------------------+---------------------------------+
| ``perform_checks``         | True                      | (*Boolean*) If ``True``,        |
|                            |                           | checks defined in               |
|                            |                           | ``./scap/checks.yaml``          |
|                            |                           | will be performed after         |
|                            |                           | each-stage of checkout.         |
+----------------------------+---------------------------+---------------------------------+
