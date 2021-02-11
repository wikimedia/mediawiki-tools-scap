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

Scap comes with an automated suite invoked via scripts/check. It
requires the various dependencies of Scap to be installed, via Debian
packages. They're not installed by the check script, and testing is
done directly on the host, not in virtual environments, to match what
happens in production. tox and pip are not used.

For a list of packages needed, see debian/control or
.pipeline/blubber.yaml, which should have matching lists of Debian
packages.

To generate a Docker container and run tests in that, run the
following command:

  blubber .pipeline/blubber.yaml test > Dockerfile.tests
  docker build -f Dockerfile.tests --iidfile id .
  docker run --rm "$(cat id)"

Building a .deb
===============

Run this command to build a scap.deb package under Docker, for your
own use (not for SRE to install on production hosts):

  ./build-deb-in-docker ~/tmp/scap-deb

The directory given as an argument must exist. The resulting package
files will be put into that directory.

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

|    Copyright 2014-2021 Wikimedia Foundation & Contributors.
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
