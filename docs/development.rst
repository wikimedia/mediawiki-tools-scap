Development
===========

Get the code
------------
Clone the repository with the following command:

    :command:`git clone https://phabricator.wikimedia.org/diffusion/MSCA/scap.git`

Scap-Vagrant
------------
For testing scap deployments, we have created a vagrant development environment
called `scap-vagrant
<https://phabricator.wikimedia.org/diffusion/GSV/>`_

Code Review
-----------

The scap team uses `Phabricator
<https://phabricator.wikimedia.org/>`_ for code review. To contribute to this
project you should use Phabricator's code review tool, `arcanist
<https://secure.phabricator.com/book/phabricator/article/arcanist/>`_ to submit
your changes for review.

Read the `Arcanist Quick Start guide
<https://secure.phabricator.com/book/phabricator/article/arcanist_quick_start/>`_
to learn about installing arcanist.

Once you have installed arcanist, you can use the :command:`arc diff` command
to submit a revision based on the last commit in your working tree.

Other useful arc sub-commands:

* :command:`arc lint` - run lint (pep8) on the changes in your working tree.
* :command:`arc unit` - run unit tests on the changes in your working tree.

Testing
-------

Scap uses `tox
<https://tox.readthedocs.org/en/latest/>`_ to run unit tests and to generate
the documentation.

To run unit tests, lint, coverage and update documentation, simply run
:command:`tox` without any arguments.