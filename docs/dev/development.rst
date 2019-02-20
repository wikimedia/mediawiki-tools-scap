

Get the code
------------
Clone the repository with the following command:

    :command:`git clone https://gerrit.wikimedia.org/r/mediawiki/tools/scap`

Scap-Vagrant
------------
For testing scap deployments, we have created a vagrant development environment
called `scap-vagrant
<https://phabricator.wikimedia.org/source/scap-vagrant/>`_

Code Review
-----------

The scap team uses `Wikimedia's Gerrit install
<https://gerrit.wikimedia.org/>`_ for code review. To contribute to this
project you will need a Wikimedia Gerrit account, see the `guide
<https://www.mediawiki.org/wiki/Gerrit/Getting_started/>`_  on  mediawiki.org
for setup instructions

Testing
-------

Scap uses `tox
<https://tox.readthedocs.org/en/latest/>`_ to run unit tests and to generate
the documentation.

To run unit tests, lint, coverage and update documentation, simply run
:command:`tox` without any arguments.

Git pre-commit hook
-------------------

There is an example pre-commit hook<pre-commit.sh>`_ that can
run ``tox -e flake8`` and ``tox -e doc`` before allowing a commit to
proceed.

.. literalinclude:: pre-commit.sh
   :language: bash
   :linenos:

This can help you catch problems earlier, before preparing
to submit a change for review. If you would like to install this
hook in your repository, simply copy the pre-commit.sh into
your .git/hooks folder and make it executable.

Specifically, run the following from the root of your scap
repository:

.. code-block:: bash

  cp docs/pre-commit.sh .git/hooks/pre-commit
  chmod +x .git/hooks/pre-commit
