##########################
Configuring Service Checks
##########################

Scap can perform service checks during a deployment in order to detect any
problems that might be caused by the new code (or configuration) and alert the
deployer early in the process and offering the option to roll back to the
previously deployed version.

.. topic:: Logging and Monitoring

  Utilizing service checks allows you to get real time status information about
  your deployments with minimal effort and without ever leaving the deployment
  console session.

.. seealso::
  * :ref:`deploy-log` - monitor output from scap deployment :term:`target` (s)

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
        stage: promote

.. seealso::

    * :class:`scap.checks.Check`
    * :class:`scap.nrpe.NRPECheck`

..
 TODO: Logstash/Graphite Checks
 ==============================

 Not yet implemented.  Once this feature is complete you will be able to monitor
 either a logstash and/or a graphite metric to detect anomalies in the rate of
 key events related to the deployment. The canonical use case is to check for a
 jump in the error rate for a service after deploying a new version.