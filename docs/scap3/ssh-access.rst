.. _ssh_access:

##########
SSH Access
##########

Before Scap can work, SSH access from tin to target machines will have to work.
It is noteworthy that ssh-agent forwarding is disabled for production servers,
so that is not an option. The current best-practice is to add a user to
targets specifically for deployment and then add that user's private key
to the Keyholder_ service running on tin.

Below is example puppet code for each of your targets that should:

#. Create a ``deploy-mockbase`` system user for your targets
#. Ensure ``deploy-mockbase`` is the owner of the target deploy directory
   (``/srv/deployment/mockbase``)
#. Ensure that the ``deploy-mockbase`` user's public key is correct on the
   target (stored in the file ``puppet://modules/mockbase/deploy-mockbase_rsa.pub``)

.. code-block:: puppet

    # == Class mockbase::deploy_target
    #
    # Ensures users and permissions are correct for deploying mockbase via scap3
    class mockbase::deploy_target(
        $user = 'deploy-mockbase',
    ) {

        user { $user:
            ensure => present,
            shell  => '/bin/bash',
            system => true,
        }

        ssh::userkey { $user:
            source => 'puppet:///modules/mockbase/deploy-test_rsa.pub',
        }

        file { '/srv/deployment/mockbase':
            ensure  =>  directory,
            owner   =>  $user,
            mode    => '0755',
            recurse => true,
        }

        file { '/srv/deployment/mockbase-cache':
            ensure  => directory,
            owner   => $user,
            mode    => '0755',
            recurse => true,
        }
    }

The following puppet code should be added in a separate class, ``role::deployment::mockbase``,
which should then be added to the ``role::deployment::server`` class that is run on tin via
``include role::deployment::mockbase``. The purpose of this code is to:

#. Adds the keyfile at ``puppet:///private/ssh/tin/deploy-mockbase_rsa`` to the keyholder
#. Adds a permissions file to the keyholder service, so that only members of the group
   ``deploy-mockbase`` can access the deploy-mockbase key.

.. code-block:: puppet

    # === Class role::deployment::mockbase
    #
    # Installs the keyholder agent for deploying mockbase
    #
    # ==== Parameters
    # [*keyholder_user*]
    #   file name of private key and generated filename of the keyholder permissions file
    # [*keyholder_group*]
    #   group on tin that has access to use the keyholder-proxy socket to
    #   login to targets
    # [*keyholder_fingerprint*]
    #   Fingerprint of the public half of the private key file
    class role::deployment::mockbase (
        $keyholder_user  = 'deploy-mockbase',
        $keyholder_group = 'deploy-mockbase',
        $key_fingerprint = '96:d3:76:32:0d:d1:c7:85:ef:2d:d1:34:c7:68:bf:87',
    ) {
        require ::keyholder
        require ::keyholder::monitoring

        keyholder::agent { $keyholder_user:
            trusted_group   => $keyholder_group,
            key_fingerprint => $key_fingerprint,
            key_file        => "${keyholder_user}_rsa"
        }
    }

Finally, you'll want to modify: ``modules/admin/data/data.yaml`` in the ``operations/puppet``
repo to create the ``deploy-mockbase`` group and add users to that group.

Once all that has been complete, you can test ssh access by setting your
``SSH_AUTH_SOCK=/run/keyholder/proxy.sock`` and attempting to ssh to a target
as the user you defined above:

.. code-block:: bash

    SSH_AUTH_SOCK=/run/keyholder/proxy.sock ssh -l deploy-mockbase mockbase-target-01.eqiad.wmnet

.. _keyholder: https://wikitech.wikimedia.org/wiki/Keyholder
