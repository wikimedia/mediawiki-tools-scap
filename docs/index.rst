####
Scap
####

.. include:: ../README.rst

Background
==========

*Scap* is a tool that was originally designed to deploy MediaWiki code
on Wikimedia Foundation servers. In Summer/Fall 2015 functionality was
added to Scap to allow it to deploy any git-based repositories on `tin
<https://wikitech.wikimedia.org/wiki/Tin>`_.

What does "Scap" mean?
----------------------

In about 2004, in the course of moving the wiki document roots from
NFS to the local hard drives of the Apache servers, a number of shell
scripts were introduced, probably written by Brion, notably:

- sync-common, which pulled the shared code into the local server
- sync-common-all, which ran sync-common on all servers

Tim added a script which would push the MW source code in common/php
out to all servers, in addition to running a pre-deployment lint
check, in order to reduce the (then frequent) incidents of site
downtime due to PHP fatal errors. The natural name for this script,
following the previous convention, would have sync-common-php-all,
but this had two problems:

- It was too long
- Its acronym was not pronounceable

Tim noticed that if the last two components of the name were swapped,
a short and catchy acronym could be contrived. Thus, scap was born.

Indices and tables
==================

Table of contents
-----------------

.. toctree::
   :maxdepth: 3

   scap3/index
   scap2/index
   dev/index
   api
   glossary

Topic Index
-----------
* :ref:`genindex`
* :ref:`search`
