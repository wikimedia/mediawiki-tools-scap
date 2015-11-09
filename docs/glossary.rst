#################
Glossary of terms
#################

.. glossary::

  target
  deployment target
    The target(s) are servers/instances which will ultimately run whatever software you
    are deploying.

    .. seealso::
      * :ref:`architecture`

  deploy host
  deployment server
  deployment master
    The server where you will run scap. In the case of WMF deployments, `tin`
    and `mira` are currently the names of deployment hosts.

  deployment repository
  deployment project
    The git repository which contains the scap configuration for your
    application/service. This could be at the root of your application's
    source code repository or it could be a separate repository which
    includes your application code and configuration as submodules.

  scap directory
    The filesystem directory named scap, which lives directly inside the root
    of your deployment project repository. This is where you put scap.cfg and
    other scap related :ref:`config files <configuration>`.