

Get the code
------------
Clone the repository with the following command:

    ``git clone https://gitlab.wikimedia.org/repos/releng/scap``

Development environment
-----------------------

For testing scap deployments, we have created a development
environment called `train-dev
<https://gitlab.wikimedia.org/repos/releng/train-dev>`_.  Please see
the train-dev documentation for information on how to set it up and
use it.

Code Review
-----------

The scap team uses `Wikimedia's GitLab
<https://gitlab.wikimedia.org/>`_ for code review. To contribute to this
project you will need a Wikimedia GitLab account, see the `guide
<https://www.mediawiki.org/wiki/GitLab/Workflows>`_  on  mediawiki.org
for setup instructions

Testing
-------

To run unit tests, lint, coverage and update documentation, simply run
``tox`` without any arguments.  For this to work you must have tox installed.

If you have Docker installed and you want to run tests without having
to install prerequisite packages on your system, you can run
``make test`` to run tests inside a properly-provisioned
container.

Git pre-commit hook
-------------------

There is an example `pre-commit hook <pre-commit.sh>`_ that can
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

  cp docs/dev/pre-commit.sh .git/hooks/pre-commit
  chmod +x .git/hooks/pre-commit
