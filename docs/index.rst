####
scap
####
Wikimedia Foundation's MediaWiki deploy scripts.

*******
Scripts
*******

scap
====
``scap`` is the main MediaWiki deployment script. It is executed from
``tin.eqiad.wmnet`` to push the contents of ``/a/common`` out to the
production cluster.

.. program-output:: ../bin/scap --help
.. seealso::
   * :func:`scap.scap`
   * :func:`scap.tasks.scap`


sync-common
===========
``sync-common`` uses rsync to fetch MediaWiki code and configuration to the
local host.

.. program-output:: ../bin/sync-common --help
.. seealso::
   * :func:`scap.sync_common`
   * :func:`scap.tasks.sync_common`


***
API
***

.. automodule:: scap
   :members:
   :special-members:

.. automodule:: scap.log
   :members:
   :special-members:

.. automodule:: scap.main
   :members:
   :special-members:

.. automodule:: scap.ssh
   :members:
   :special-members:

.. automodule:: scap.tasks
   :members:
   :special-members:

.. automodule:: scap.utils
   :members:
   :special-members:


******************
Indices and tables
******************
* :ref:`genindex`
* :ref:`search`

