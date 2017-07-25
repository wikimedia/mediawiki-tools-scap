.. _commands:

###############
Deploy commands
###############

scap deploy
===========
The **deploy** command handles deployment of various wikimedia projects from
the deployment server to a staging or production environment.

* Tags the current revision in the current git directory
* Runs `git update-server-info` for the current git directory (as well
  as for any submodules)
* SSHs into each server in the `dsh_targets` file
* Runs git fetch in the `/srv/deployment/{repo}` directory of each target
  (running checkout if it does not exist)
* Checks out the tag created in step 1 on each of the target machines
* If a `service_name` is specified, the service is restarted. Multiple services
  may be specified by separating them with commas, e.g., ``service1,
  service2``. A service can be reloaded by appending ``=reload`` to the
  service's name, e.g., ``service1, service2 = reload``.
* If a `service_port` is specified, make sure that it is accepting
  connections, waiting up to `service_timeout` (120 seconds by default)

.. program-output:: ../bin/scap deploy --help
.. seealso::
   * :class:`scap.Deploy`

.. _deploy-log:

scap deploy-log
===============

The :command:`scap deploy-log` command provides powerful filters for the `scap deploy` logs.

The main deploy application sends all structured log output to a file under
:file:`scap/log/{git-tag}.log`. deploy-log is meant to run during or after a
deploy, potentially in a separate terminal. Log entries can be filtered on
one or more fields using a given free-form expression. By default
:command:`scap deploy-log` will periodically scan the scap/log directory for new
files and immediately begin tailing any newly discovered log file.

As an alternative to the default behavior, you can either specify the log file
to parse via the :option:`scap deploy-log --file` option or choose the newest log file by using
:option:`scap deploy-log --latest`; in this case, it will simply filter the entire file for
matching records and exit, rather than watching for more log files to be
created.


   The default behavior is convenient for monitoring an ongoing deployment from
   a separate terminal. Simply start :command:`scap deploy-log` in a separate
   terminal prior to running
   :command:`scap deploy`. Once your deployment starts, :command:`scap deploy-log` will
   discover the new log file and immediately begin displaying relevant log
   messages


Usage
-----

:command:`scap deploy-log` ``[--file <file>] [--latest] [-v] [expr]``

scap deploy-log ``[--file <file>] [--latest] [-v] [expr]``

.. program:: scap deploy-log

.. option:: -f <file>, --file <file>

   Used to explicitly specify the log file to be parsed. If no file is specified
   then :command:`deploy-log` will automatically open any newly created log
   files and immediately begin outputting any matching log messages.

.. option:: -l, --latest

   Parse and filter the latest log file, exiting once the entire file has been
   processed.

.. option:: -v, --verbose

   Produce verbose output

.. option:: expr

   Optional filter expression which is used to match log entries in <file>

Examples
--------

.. code-block:: bash

   # show verbose output:
   scap deploy-log -v
   # tail the most recent log file:
   scap deploy-log --latest
   # show log messages for the host named scap-target-01
   scap deploy-log 'host == scap-target-01'
   # show log messages matching a regex pattern:
   scap deploy-log 'msg ~ "some important (message|msg)"'
   # show WARNING messages for hosts whose name begins with "scap-target-"
   scap deploy-log 'levelno >= WARNING host == scap-target-*'

.. seealso::
   * :func:`scap.DeployLog`
   * :func:`scap.log.Filter`
   * :func:`scap.log.JSONFormatter`
