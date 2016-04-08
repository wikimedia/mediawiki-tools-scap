.. _commands:

###############
Deploy commands
###############

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

.. program-output:: ../bin/deploy --help
.. seealso::
   * :class:`scap.Deploy`

.. _deploy-log:

deploy-log
==========

The :command:`deploy-log` command provides powerful filters for the `deploy` logs.

The main deploy application sends all structured log output to a file under
:file:`scap/log/{git-tag}.log`. deploy-log is meant to run during or after a
deploy, potentially in a separate terminal. Log entries can be filtered on
one or more fields using a given free-form expression. By default
:command:`deploy-log` will periodically scan the scap/log directory for new
files and immediately begin tailing any newly discovered log file.

As an alternative to the default behavior, you can either specify the log file
to parse via the :option:`deploy-log --file` option or choose the newest log file by using
:option:`deploy-log --latest`; in this case, it will simply filter the entire file for
matching records and exit, rather than watching for more log files to be
created.


   The default behavior is convenient for monitoring an ongoing deployment from
   a separate terminal. Simply start :command:`deploy-log` in a separate
   terminal prior to running
   :command:`deploy`. Once your deployment starts, :command:`deploy-log` will
   discover the new log file and immediately begin displaying relevant log
   messages


Usage
-----

:command:`deploy-log` ``[--file <file>] [--latest] [-v] [expr]``


.. program:: deploy-log

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
   deploy-log -v
   # tail the most recent log file:
   deploy-log --latest
   # show log messages for the host named scap-target-01
   deploy-log 'host == scap-target-01'
   # show log messages matching a regex pattern:
   deploy-log 'msg ~ "some important (message|msg)"'
   # show WARNING messages for hosts whose name begins with "scap-target-"
   deploy-log 'levelno >= WARNING host == scap-target-*'

.. seealso::
   * :func:`scap.DeployLog`
   * :func:`scap.log.Filter`
   * :func:`scap.log.JSONFormatter`
