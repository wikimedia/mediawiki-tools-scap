#####################
Repo Owner Quickstart
#####################

This documentation is for repository owners wishing to move deployments
from Trebuchet_ to Scap using the :ref:`commands`.

This quickstart assumes that:

#. You are deploying from tin_.
#. Your repository is already present on tin.
   If your repository is not on tin, the current best instructions are in
   `Add the new repo's configuration in puppet`_ on wikitech. These
   instructions are likely to change as Trebuchet is phased-out.
#. You have at least one target machine.
#. You have SSH access to the target machines from tin (more info in :ref:`ssh_access`).
#. The target machines have the puppet modules ``scap`` and ``scap::target``
   in their manifests.
#. The user that has SSH access has write permissions to the directory
   into which code is placed.

The ``scap`` directory
~~~~~~~~~~~~~~~~~~~~~~

The ``scap`` directory contains all the configuration Scap uses to deploy
your code. It must reside at the root of your repository's working directory
on tin. The ``scap`` directory can be, but does not have to be, a part of your git
repository (it may be a submodule, or a ``.gitignore``'d directory on tin).

A good starting point for a scap directory can be seen here::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── mockbase
    └── scap.cfg

.. _scap.cfg:

``scap.cfg``
------------

The ``scap/scap.cfg`` file needs, at a minimum, values for these keys:

#. ``git_repo``
#. ``dsh_targets``

The ``git_repo`` is the path on the ``git_server`` (``tin.eqiad.wmnet`` by
default) where your repository lives.

The ``dsh_targets`` file is the list of deployment targets for your repository.

An example of a sensible default ``scap/scap.cfg`` file is seen here::

    [global]
    # The repo "mockbase" will be fetched from tin (the git_server)
    git_repo: mockbase

    # code will be deployed to /srv/deployment/ on the target
    git_deploy_dir: /srv/deployment

    # connect to targets with user deploy-mockbase
    ssh_user: deploy-mockbase

    # The target list is in scap/mockbase
    dsh_targets: mockbase

    # There are submodules that need to be pulled from tin
    git_submodules: True

    # There is a service that needs to be restarted
    service_name: mockbase
    service_port: 1134

    [wmflabs]
    git_server: deployment-bastion.deployment-prep.eqiad.wmflabs

    [wmnet]
    git_server: tin.eqiad.wmnet

This file is in ConfigParser_ format.  A ``scap.cfg`` file consists of
sections led by a ``[section]`` header and followed by ``name: value``
entries.

Lines beginning with ``#`` are ignored and may be used to provide comments.

The configuration is read from the global section and additional sections
based on the fully qualified domain name of the local host.

For example, on the host ``tin.eqiad.wmnet``, the final value for a given
setting would be the first value found in sections:
``[tin.eqiad.wmnet]``, ``[eqiad.wmnet]``, ``[wmnet]`` or ``[global]``.
Sections not present in the configuration file will be ignored.

Targets
-------

In the example above, the targets would be taken from your local repository's
``scap/mockbase`` file. That is the file specified by ``dsh_targets``.

The ``dsh_targets`` file is a file with a list of hosts, one host per line.
Lines beginning with ``#`` are ignored and may be used to provide comments.
Blank lines may be used for clarification.

An example mockbase ``dsh_targets`` file might look like::

    # Primary mockbase pair
    mockbase01.eqiad.wmnet
    mockbase02.eqiad.wmnet

    # Secondary mockbase pair
    mockbase03.eqiad.wmnet
    mockbase04.eqiad.wmnet

Targets can also be grouped into separate target files and deployed in
phases. For instance, if I wanted to move the ``mockbase01`` and ``mockbase02``
hosts into a separate, canary deploy group, I would add the following lines
to my ``scap/scap.cfg`` file::

    server_groups: 'canary,default'
    canary_dsh_targets: mockbase-canaries

The full ``scap/scap.cfg`` file would now look like::

    [global]
    # Code will be fetched from tin:/srv/deployment/mockbase
    git_repo: mockbase

    # code will be deployed to /srv/deployment/mockbase on the target
    git_deploy_dir: /srv/deployment

    # connect to targets with user deploy-mockbase
    ssh_user: deploy-mockbase

    # Canary deploy targets first
    server_groups: canary, default

    # Two target lists
    canary_dsh_targets: mockbase-canaries
    dsh_targets: mockbase

    # There are submodules that need to be pulled from tin
    git_submodules: True

    # There is a service that needs to be restarted
    service_name: mockbase
    service_port: 1134

    [wmflabs]
    git_server: deployment-bastion.deployment-prep.eqiad.wmflabs

    [wmnet]
    git_server: tin.eqiad.wmnet

The ``server_groups`` config variable represents the order of group deployment.
In the example above, the ``canary`` group is deployed to before the ``default`` group.
Adding a server group necessitates adding a ``[group]_dsh_targets`` key
in ``scap/scap.cfg``—because I added a server group named ``canary`` in ``server_groups``,
I also need a ``canary_dsh_targets`` config variable that points to a new
target file. After adding the ``canary_dsh_targets`` file, my new ``scap``
directory looks like this::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── mockbase
    ├── mockbase-canaries
    └── scap.cfg

The ``scap/mockbase`` file looks like this::

    # Non-canary mock-base servers
    mockbase03.eqiad.wmnet
    mockbase04.eqiad.wmnet

And the ``scap/mockbase-canaries`` file looks like this::

    # Canary mockbase servers
    mockbase01.eqiad.wmnet
    mockbase02.eqiad.wmnet

Now when I run ``deploy``: code fetch, update, and service restart will happen
on ``mockbase01`` and ``mockbase02`` (from the ``scap/mockbase-canaries`` file)
before I am prompted to continue the deploy on the default targets
(from the ``scap/mockbase`` file).::

    deployer@tin:/srv/deployment/mockbase$ deploy
        00:05:22 Started deploy_mockbase
        Entering 'mockbase'
        00:05:22
        == CANARY ==
        :* mockbase01.eqiad.wmnet
        :* mockbase02.eqiad.wmnet
        deploy_mockbase_config_deploy: 100% (ok: 2; fail: 0; left: 0)
        deploy_mockbase_fetch: 100% (ok: 2; fail: 0; left: 0)
        deploy_mockbase_promote: 100% (ok: 2; fail: 0; left: 0)
        canary deploy successful. Continue? [y]: y
        00:05:35
        == DEFAULT ==
        :* mockbase03.eqiad.wmnet
        :* mockbase04.eqiad.wmnet
        deploy_mockbase_config_deploy: 100% (ok: 2; fail: 0; left: 0)
        deploy_mockbase_fetch: 100% (ok: 2; fail: 0; left: 0)
        deploy_mockbase_promote: 100% (ok: 2; fail: 0; left: 0)
        00:05:53 Finished deploy_mockbase (duration: 00m 31s)

Service Restarts and Checks
~~~~~~~~~~~~~~~~~~~~~~~~~~~

When you specify a ``service_name``, the service specified will be restarted
as part of the ``promote`` stage of deployment. The ``ssh_user`` must have
appropriate sudoers permissions to restart the service.

When you specify a ``service_port``, the port specified will be checked to
see if it is accepting connections. By default, the port check on each host
will timeout after 120 seconds. If a service takes a long time to begin
accepting connections, you may need to set the ``service_timeout`` value
to a number > 120.

Additional checks
-----------------

After any stage of deployment (the stages being: ``config_deploy`` when any
template files are built on targets, ``fetch`` when the git repository is fetched
to the target machines, and ``promote`` when the newly fetched code is swapped for
the currently live code and a service is restarted), additional, user-defined,
checks may be preformed.

User-defined checks are specified in the ``scap/checks.yaml`` file::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── checks.yaml
    ├── mockbase
    ├── mockbase-canaries
    └── scap.cfg


The ``checks.yaml`` file is a dictionary of named checks. An example check
for the mockbase repository is to ensure that a particular end-point gives
a valid response to an HTTP request on localhost::

    checks:
      mockbase_responds:
        type: command
        stage: promote
        command: curl -Ss localhost:1134

Now, after the ``service_name`` is restarted, and after the ``service_port`` is
checked, at the end of the ``promote`` stage, the ``mockbase_responds`` check
will run. If the exit status of the command is non-zero, the deployer will be
notified and deployment will fail.

In the example above, the user-defined check will happen for every service group.
If I wanted to only run this check for the ``canary`` deploy group, I would modify
``scap/checks.yaml`` to specifiy the ``group``::

    checks:
      mockbase_responds:
        type: command
        stage: promote
        group: canary
        command: curl -Ss localhost:1134

In addition to the ``command``-type checks, you can also run any :ref:`nrpe`
that are defined in ``/etc/nagios/nrpe.d``. For example, if, in addition to
cURLing a known end-point, you wanted to check disk-space at the end
of the fetch stage for all groups using the NRPE check at
``/etc/nagios/nrpe.d/check_disk_space.cfg``, you could modify
``scap/checks.yaml`` and specify an ``nrpe``-type check::

    checks:
      mockbase_responds:
        type: command
        stage: promote
        group: canary
        command: curl -Ss localhost:1134

      check_diskspace:
        type: nrpe
        stage: fetch
        command: check_disk_space

No additional ``scap/scap.cfg`` variables are required to run the checks in
``scap/checks.yaml``: if the file doesn't exist, no user-defined checks are run.

Config file deploy
~~~~~~~~~~~~~~~~~~

Scap supports target-local rendering of jinja2_ templated configuration files.
To render a file template on a target, place the template in the ``templates``
directory of your repository's ``scap`` directory. You will also need to
create a ``scap/config-files.yaml`` file to control rendered config templates::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── templates
    │   └── config.yaml.j2
    ├── checks.yaml
    ├── config-files.yaml
    ├── mockbase
    ├── mockbase-canaries
    └── scap.cfg

``scap/config-files.yaml`` is a list of configuration files keyed by
their final location and supporting two properties: ``template`` and
``remote_vars``.

As an example, let's add mockbase's configuration file to the
``scap/templates/config.yaml.j2`` file::

    ---
    info:
      name: mockbase

Now, let's configure Scap to deploy this file to ``/etc/mockbase/config.yaml``
by specifying the target and the template in the ``scap/config-files.yaml``
file::

    ---
    /etc/mockbase/config.yaml:
      template: config.yaml.j2

During the next ``deploy`` run, in the ``config_deploy`` phase, this template
will be fetched from tin (the ``git_server``) and symlinked to its final
location at ``/etc/mockbase/config.yaml``.

Config Template Variables
-------------------------

The jinja2_ template files inside the ``scap/templates`` directory are fully
jinja-syntax-capable. Variables and looping constructs are fully supported.

The master variable file for templates is called ``vars.yaml`` and is located
inside the ``scap`` directory::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── templates
    │   └── config.yaml.j2
    ├── checks.yaml
    ├── config-files.yaml
    ├── mockbase
    ├── mockbase-canaries
    ├── scap.cfg
    └── vars.yaml

Any variables specified in ``scap/vars.yaml`` will be used to render a template
before it is symlinked into place. For example, let's add the variables
``last_deployer`` and ``bar`` into our ``scap/templates/config.yaml.j2`` file::

    ---
    info:
      name: mockbase

    deployer: {{ last_deployer }}
    foo: {{ bar }}

And we'll add the corresponding values to the ``scap/vars.yaml`` file::

    last_deployer: Scappy, the scap pig
    foo: bar

After another ``deploy``, the final rendered file at ``/etc/mockbase/config.yaml``
will read::

    ---
    info:
      name: mockbase

    deployer: Scappy, the scap pig
    foo: bar

Remote Variable Files
---------------------

An additional source of variables for rendered templates is specified in the
``scap/config-files.yaml`` file using the ``remote_vars`` template property.
``remote_vars`` is a path on a target to a yaml file, the contents of which will
override the values specified in ``scap/vars.yaml``. For example, if I had
Puppet geneate a file for each host at ``/var/mockbase/dynamic-config.yaml``
with the contents::

    ---
    hostname: mockbase01
    ip_address: 10.10.10.1

I could then use these variables in any of my local ``scap/templates`` by
specifying the ``remote_vars`` property in the ``scap/config-files.yaml``
file::

    ---
    /etc/mockbase/config.yaml:
      template: config.yaml.j2
      remote_vars: /var/mockbase/dynamic-config.yaml

Then update my template to use those additional variables supplied by the
``remote_vars`` file::

    ---
    info:
      name: {{ hostname }}

    deployer: {{ last_deployer }}
    foo: {{ bar }}
    localhost_public_ip: {{ ip_address }}

The final rendered template at ``/etc/mockbase/config.yaml`` on ``mockbase01``
would read::

    ---
    info:
      name: mockbase01

    deployer: Scappy, the scap pig
    foo: bar
    localhost_public_ip: 10.10.10.1

Environments
~~~~~~~~~~~~

There are times when a repository may need a different configuration depending on
the environment into which it is deployed. Staging vs production vs beta
may all need different configurations. This is the use-case of the
``--environment`` flag and the ``environments`` directory.

Running a mockbase ``deploy`` with a different environment means that every
configuration file will first be searched-for under the ``scap/environments/[environment]``
directory before falling-back to the global configuration file.

For example, if the ``/etc/mockbase/config.yaml`` file needed to have an
additional ``beta: true`` parameter in its template file, I could override
the template in the ``beta`` environment::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── environments
    │   └── beta
    │       └── templates
    │           └── config.yaml.j2
    ├── templates
    │   └── config.yaml.j2
    ├── checks.yaml
    ├── config-files.yaml
    ├── mockbase
    ├── mockbase-canaries
    ├── scap.cfg
    └── vars.yaml

Inside the ``scap/environments/beta/templates/config.yaml.j2`` file, I would
simply have a template complete with the new beta boolean::

    ---
    info:
      name: {{ hostname }}

    deployer: {{ last_deployer }}
    foo: {{ bar }}
    localhost_public_ip: {{ ip_address }}
    beta: true

Combined-environments ``vars.yaml``
-----------------------------------

All extant files in an environment shadow their global counterparts with the
exception of ``vars.yaml``. Adding an environment-specific ``vars.yaml``
will override any variables set in both the global ``vars.yaml``
file and the environment-specific ``vars.yaml`` file, but will inherit any variable
values that aren't set in the environment-specific
``vars.yaml`` that are set in the global ``vars.yaml``.

For example, if I wanted to set the ``/etc/mockbase/config.yaml`` variable
``foo`` to the value ``baz`` in the ``beta`` environment, I could do so
by first creating an environment-specific ``vars.yaml``::

    deployer@tin:/srv/deployment/mockbase$ tree --dirsfirst scap
    scap
    ├── environments
    │   └── beta
    │       ├── templates
    │       │   └── config.yaml.j2
    │       └── vars.yaml
    ├── templates
    │   └── config.yaml.j2
    ├── checks.yaml
    ├── config-files.yaml
    ├── mockbase
    ├── mockbase-canaries
    ├── scap.cfg
    └── vars.yaml

Contents of ``scap/environments/beta/vars.yaml``::

    ---
    foo: baz

Final rendered content of ``/etc/mockbase/config.yaml`` after running
``deploy --environment beta``::

    ---
    info:
      name: mockbase01

    deployer: Scappy, the scap pig
    foo: baz
    localhost_public_ip: 10.10.10.1
    beta: true

.. _trebuchet: https://wikitech.wikimedia.org/wiki/Trebuchet
.. _tin: https://wikitech.wikimedia.org/wiki/Tin
.. _add the new repo's configuration in puppet: https://wikitech.wikimedia.org/wiki/Trebuchet#Add_the_new_repo.27s_configuration_to_puppet
.. _configparser: https://docs.python.org/2/library/configparser.html
.. _jinja2: http://jinja.pocoo.org/docs/dev/
