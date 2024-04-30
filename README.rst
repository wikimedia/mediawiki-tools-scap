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

Automated tests can be executed with `make test` or `tox -e test`.

For a list of Scap dependencies during testing, see `requirements.txt` and
`test-requirements.txt`.  Some Debian packages are also required to set up the
running environment, for instance to install Python or pip.

To ensure compatiblity with multiple Debian distributions, we use Docker images
defined using Blubber (see `.pipeline/blubber.yaml`). The image build installs
the runtime dependencies from Debian packages while tests dependencies defined
in `test-requirements.txt` are installed via `pip`. This lets us use ensure we
run with the same set of Python module provided by Debian while using more
recent versions of testing utilities than the one frozen by Debian
(ex: `pytest`) and ensure we use the same version of `flake8` regardless of the
Debian distribution version.

To generate the container images and run tests in each of them, we provide a
`Makefile`. To run all tests against all supported Debian distributions use
`make test`.

The `test` target builds the images using the `Blubberfile syntax
<https://wikitech.wikimedia.org/wiki/Blubber/User_Guide#Blubberfiles>` using
`Blubber buildkit <docker-registry.wikimedia.org/repos/releng/blubber/buildkit>`.
The target then invokes the defined entrypoint `tox` which runs the
tests in each images.

The Wikimedia CI builds those images in a similar way and run the same tests.

To build all images without running tests, use `make images`, useful when
amending the Blubber config or changing dependencies.

For each Debian distribution defined at the top of the `makefile`, rules let
you build and test against a single distribution: `make test-bullseye`. You
will probably want to use it when developing instead of testing against each of
the distribution. When your change is ready, run it against all distributions
to catch potential compatibility issues (`make test`).

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

|    Copyright 2014-2022 Wikimedia Foundation & Contributors.
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
