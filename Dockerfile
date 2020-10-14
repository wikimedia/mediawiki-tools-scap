# Base stage: A build environment based on Debian stretch, with all
# the Scap build-dependencies and build tools installed. Keeping it as
# a separate stage makes it nicely cacheable.
FROM docker-registry.wikimedia.org/wikimedia-stretch:latest AS deps
MAINTAINER Release Engineering releng@lists.wikimedia.org
RUN apt-get update
RUN apt-get install -y \
  build-essential \
  lintian \
  debhelper \
  dh-python \
  python-all \
  python-setuptools \
  python-concurrent.futures \
  python-jinja2 \
  python-pygments \
  python-yaml \
  python-requests \
  git \
  bash-completion \
  python-six \
  python-configparser \
  python-psutil \
  tox \
  flake8 

# Build stage: This builds on the base stage. The source tree and
# place where build results are stored get bind mounted into the
# container as /build and /build/src.
FROM deps
WORKDIR /build/src
ENTRYPOINT ["./build-deb-in-environment", "/build"]
