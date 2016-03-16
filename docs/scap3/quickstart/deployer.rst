#########################
Deployer Quickstart Guide
#########################

For a deployer, interaction with Scap is straight-forward.

The two most-typical commands are ``deploy`` and ``deploy-log``.

Below we demo deployment using a fake service called **Mockbase**
located at ``tin:/srv/deployment/mockbase/deploy``.

``deploy``
~~~~~~~~~~

The first step to deploying new code from your ``deployment_host`` (in most
cases this will be ``tin.eqiad.wmnet``) is to use ``git pull``, ``git checkout``,
``git cherry-pick``, and ``git commit`` to bring the repository on the
``deployment_host`` into the state that you want deployed to your targets.


.. code-block:: bash

    deployer@tin:/srv/deployment/mockbase/deploy$ echo "Add a README" > README
    deployer@tin:/srv/deployment/mockbase/deploy$ git add README
    deployer@tin:/srv/deployment/mockbase/deploy$ git commit -m 'Added a README'
    [master 70eb01e] Added a README
     1 file changed, 1 insertion(+)
      create mode 100644 README

Once the repository state is correct, use the deploy command to release the
finished code:

.. code-block:: bash

    deployer@tin:/srv/deployment/mockbase/deploy$ deploy
    20:46:12 Started deploy_mockbase/deploy
    Entering 'mockbase'
    20:46:12
    == DEFAULT ==
    :* scap-target-07
    :* scap-target-08
    :* scap-target-09
    :* scap-target-04
    :* scap-target-05
    :* scap-target-06
    :* scap-target-10
    :* scap-target-01
    :* scap-target-02
    :* scap-target-03
    deploy_mockbase/deploy_config_deploy: 100% (ok: 10; fail: 0; left: 0)
    deploy_mockbase/deploy_fetch: 100% (ok: 10; fail: 0; left: 0)
    deploy_mockbase/deploy_promote: 100% (ok: 10; fail: 0; left: 0)
    20:46:42 Finished deploy_mockbase/deploy (duration: 00m 29s)

The default output has several sections:

#. **Deploy group target list**
    Scap will list the targets of the deploy.
    Scap lists both the deploy target names and the deploy group.
    In the example output below, the deploy group is "DEFAULT" and the
    targets are listed beneath. (see `Configuring a Git Repo`_ for more
    info on targets). If you have more than one deploy group, you will see
    more than one of these sections.

   .. code-block:: bash

    == DEFAULT ==
    :* scap-target-07
    :* scap-target-08
    :* scap-target-09
    :* scap-target-04
    :* scap-target-05
    :* scap-target-06
    :* scap-target-10
    :* scap-target-01
    :* scap-target-02
    :* scap-target-03

#. **Deployment stages reporters**

   There are three stages to a deployment:

   #. ``config_deploy`` Where your configration files are deployed.
   #. ``fetch`` Where the code from your repository is fetched to targets.
   #. ``promote`` Where your currently running code is swapped for newly fetched code.

   Each of these stages is run sequentially and you can see the progress of
   these stages via the reporters:

   .. code-block:: bash

     deploy_mockbase/deploy_config_deploy: 100% (ok: 10; fail: 0; left: 0)
     deploy_mockbase/deploy_fetch: 100% (ok: 10; fail: 0; left: 0)
     deploy_mockbase/deploy_promote: 100% (ok: 10; fail: 0; left: 0)

The ``deploy`` command also accepts the ``--verbose`` argument which may be
useful in troubleshooting. The full options of the ``deploy`` command can
be found in the `deploy documentation`_.

``deploy-log``
~~~~~~~~~~~~~~

The ``deploy-log`` command is designed to be used in tandem with the ``deploy``
command. It can be run either during a ``deploy``, or it may be used to
inspect the log afterward.

Below is a gif that demonstrates a simple use of ``deploy-log``; running along-side
a ``deploy`` that targets only a single host (using the ``-l|--limit`` flag).

At first, in the right pane, the tail of the logfile from the previous
deployment is opened; however, after starting the ``deploy`` in the
left pane, a new logfile is tailed.

``deploy-log`` supports advanced filtering both during and post deployment. A
deployer can match a particular host, log message, or error level using one
of several comparison operators (`see \`\`deploy-log\`\` examples`_)

.. image:: /assets/images/deploy-log.gif

Deployment information
~~~~~~~~~~~~~~~~~~~~~~

A log of deployments is kept in several places. A full log of every deployment
is available via Logstash_. Also, a json record of the last deployment
is kept inside your repo's git directory on the ``deploy_host`` at
``.git/DEPLOY_HEAD``.

The ``.git/DEPLOY_HEAD`` file shows the date, user, and commit of the last
deployment from this repository. The deployed commit is also tagged locally
with this information via an annotated ``git tag``.

.. code-block:: bash

    deployer@tin:/srv/deployment/mockbase/deploy$ jq '.' < .git/DEPLOY_HEAD
    {
      "timestamp": "2015-11-17T22:05:53.277869",
      "user": "deployer",
      "tag": "scap/sync/2015-11-17/0014",
      "commit": "70eb01ed946c6bdc7d94b5b71abe81b9ea2e8d0c"
    }

.. _configuring a git repo: ../repo_config.html#available-configuration-variables
.. _deploy documentation: ../deploy_commands.html#deploy
.. _See ``deploy-log`` examples: ../deploy_commands.html#examples
.. _logstash: https://logstash.wikimedia.org
