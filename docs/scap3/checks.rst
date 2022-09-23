##########################
Configuring Service Checks
##########################

Scap can perform service checks during a deployment in order to detect any
problems that might be caused by the new code (or configuration) and alert the
deployer early in the process and offering the option to roll back to the
previously deployed version.

Additionally, you can run any arbitrary command using checks which can thus be
used as a hook system in the :doc:`process flow <architecture>`.

The environment in which a check executes has the following environment
variables defined:

  * ``$SCAP_FINAL_PATH`` - A symbolic link created during the ``promote`` stage
    that points to the deployed code, constructed from the configured
    ``git_deploy_dir`` and the repo name, e.g. ``/srv/deployment/repo/name``.
    Note that this link will point to the previously deployed rev for checks
    that run prior to ``promote``.
  * ``$SCAP_REV_PATH`` - Path to the revision of the code being deployed, e.g.
    ``/srv/deployment/repo/name-cache/revs/abc123``. This is the most reliable
    value for scripts that need to operate on the revision being deployed
    regardless of its current state.
  * ``$SCAP_REVS_DIR`` - Directory that contains all deployed revisions, e.g.
    ``/srv/deployment/repo/name-cache/revs``.
  * ``$SCAP_CURRENT_REV_DIR`` - A symbolic link created during the ``promote``
    stage that points to the deployed code. Note this is an internal convention
    that scap uses to track the state of deployment and should only be used by
    scripts that need to mimic its internal logic. For a less esoteric
    alternative, use ``$SCAP_FINAL_PATH``.
  * ``$SCAP_DONE_REV_DIR`` - A symbolic link created during the ``finalize``
    stage that points to the deployed code. Note this is an internal convention
    that scap uses to track the state of deployment and should only be used by
    scripts that need to mimic its internal logic.

.. topic:: Logging and Monitoring

  Utilizing service checks allows you to get real time status information about
  your deployments with minimal effort and without ever leaving the deployment
  console session.

.. seealso::
  * :command:`scap deploy-log` - monitor output from scap deployment :term:`target` (s)

.. _nrpe:

NRPE Checks
===========

The `nrpe` check type is a simple wrapper around existing Icinga/NRPE checks
so that your deployments can utilize monitoring infrastructure that is already
in place with minimal effort.

By default, the check commands are loaded and registered from definitions in
`/etc/nagios/nrpe.d`. You must reference these checks in your checks.yaml in
order to tell scap which checks are relevant to the service you are deploying.

The directory `/etc/nagios/nrpe.d/` should contain config files (generated
by puppet) which specify the specific commands needed to run various nagios
check plugins. For example, the directory should look something like this:

/etc/nagios/nrpe.d:

.. code-block:: text

    ├── check_cassandra.cfg
    ├── check_check_dhclient.cfg
    ├── check_check_eth.cfg
    ├── check_check_salt_minion.cfg
    ├── check_disk_space.cfg
    ├── check_dpkg.cfg
    ├── check_endpoints_restbase.cfg
    ├── check_puppet_checkpuppetrun.cfg
    ├── check_raid.cfg
    └── check_root_disk_space.cfg

NRPE checks can be referenced in `checks.yaml` using `type: nrpe` and
`command: {check_name}`. The value of `{check_name}` must match the name
of a file in /etc/nagios/nrpe.d, but omitting the file extension.

Example checks.yml:

.. code-block:: yaml

    checks:
      service_endpoints:
        type: nrpe
        command: check_service_endpoints
        after: promote
        timeout: 60 # default is 30 seconds

.. seealso::

    * :class:`scap.checks.Check`
    * :class:`scap.nrpe.NRPECheck`

.. _script:

Script Checks
=============
The ``script`` check type allows users to run scripts before or after any
`check stages`_ of a deployment. This was in the past achieved through use of
the ``command`` check; however, this provides an easier means by which to
execute scripts that may change between revisions of a repository.

Script checks will only run executable files in the `scap/scripts` directory.

Script checks can be referenced in `checks.yaml` using `type: script` and
`command: [basename_of_executable_file]`. The value of
`[basename_of_executable_file]` will be executed by the ``ssh_user``.

In the example below, scap expects that in the repo being deployed there exists
a `scap/scripts/build_venv.sh` file that is executable by the ``ssh_user``.

Example checks.yml:

.. code-block:: yaml

    checks:
      build_venv:
        type: script
        after: promote
        command: build_venv.sh


Command Checks
==============

The ``command`` check type allows users to define shell commands to run before
or after each stage of deployment.

Command checks can be referenced in `checks.yaml` using `type: command` and
`command: {shell_command}`. The value of `{shell_command}` will be executed
by the ``ssh_user`` before or after the stage specified by ``stage:``.

Example checks.yml:

.. code-block:: yaml

    checks:
      mockbase_responds:
        type: command
        after: promote
        command: curl -Ss localhost:1134

..
 TODO: Logstash/Graphite Checks
 ==============================

 Not yet implemented.  Once this feature is complete you will be able to monitor
 either a logstash and/or a graphite metric to detect anomalies in the rate of
 key events related to the deployment. The canonical use case is to check for a
 jump in the error rate for a service after deploying a new version.

Check stages
============

#. ``restart_service`` - a service is restarted
#. ``config_deploy`` - templated configuration files are rendered
#. ``config_diff`` - compare each file to the deployed version, called during
   ``scap deploy --dry-run``.
#. ``fetch`` - target repository has been checked-out
#. ``finalize`` - final deployment cleanup
#. ``promote`` - make the new deployment active
#. ``rollback`` - target is rolled back to the last deployed revision

Not all of these stages are run for every deployment.  The basic stages that
you might want to write checks for are ``fetch`` and ``promote``.

The checks may be executed at any stage of the deployment, either before
running the stage or after it. The execution time is specified in the
``checks.yaml`` configuration file using the ``before`` or ``after`` option
followed by one of the stage names above.

Checks with ``before`` are executed before the stage starts (for example
``before: promote``). A failure of those checks will prevent the associated
stage from running.

Checks with ``after`` (or the deprecated ``stage``) are run after the stage
runs. A failure of those checks will cause the stage to be failling.
