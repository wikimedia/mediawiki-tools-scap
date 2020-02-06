::

           ___ ____
         ⎛   ⎛ ,----
          \  //==--'
     _//| .·//==--'    ____________________________
    _OO≣=-  ︶ ᴹw ⎞_§ ______  ___\ ___\ ,\__ \/ __ \
   (∞)_, )  (     |  ______/__  \/ /__ / /_/ / /_/ /
     ¨--¨|| |- (  / _______\___/ \___/ \__^_/  .__/
         ««_/  «_/ jgs/bd808               /_/


**Scap** is the deployment script used by Wikimedia Foundation to publish
code and configuration on production web servers.


Running tests
=============

Scap come with a unit test suite implemented using pytest and invoked
using tox.

On Debian 10 (buster) you need the following packages installed to run
the test suite:

   build-essential locales-all git python2 python3 python-all-dev
   python3-all-dev tox php

After this, run the ``tox`` command on the command line to run the
tests. It uses the tox.ini file to know what to do. Edit that file to
drop any Python3 versions you don't have installed from the
``envlist`` line.

Alternatively, you can specify python environments against which to run the
test suite by selecting a custom envlist in ``tox`` via the ``-e`` option,
i.e., ``tox -e py27``.


Reporting Issues
================

You can report issues to the `#scap
<https://phabricator.wikimedia.org/maniphest/task/create/?projects=Scap>`_
project on phabricator.wikimedia.org


Credits
=======

ASCII art derived from original work by Joan Stark [#pig]_ and the `speed`
figlet font [#speedfont]_.

.. [#pig] http://www.oocities.com/spunk1111/farm.htm#pig
.. [#speedfont] http://www.jave.de/figlet/fonts/details/speed.html

License
=======

|    Copyright 2014-2017 Wikimedia Foundation & Contributors.
|
|    Scap is free software: you can redistribute it and/or modify
|    it under the terms of the GNU General Public License as published by
|    the Free Software Foundation, version 3.
|
|    Scap is distributed in the hope that it will be useful,
|    but WITHOUT ANY WARRANTY; without even the implied warranty of
|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
|    GNU General Public License for more details.
|
|    You should have received a copy of the GNU General Public License
|    along with this program.  If not, see <http://www.gnu.org/licenses/>.


Increment T184118
=================
Increment this number when you work around `#T184118
<https://phabricator.wikimedia.org/T184118>`_: 5
