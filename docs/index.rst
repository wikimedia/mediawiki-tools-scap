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


scap-1
======
``scap-1`` (also known as ``sync-common``) uses rsync to fetch MediaWiki code
and configuration to the local host.

.. program-output:: ../bin/scap-1 --help
.. seealso::
   * :func:`scap.sync_common`
   * :func:`scap.tasks.sync_common`


***
API
***

.. automodule:: scap
   :members:

.. automodule:: scap.tasks
   :members:

.. automodule:: scap.log
   :members:

.. automodule:: scap.utils
   :members:


******************
Indices and tables
******************
* :ref:`genindex`
* :ref:`search`

