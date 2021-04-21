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
  python3 \
  python3-jinja2 \
  python3-pygments \
  python3-pytest \
  python3-pytest-mock \
  python3-requests \
  python3-sphinx \
  python3-sphinxcontrib.actdiag \
  python3-sphinxcontrib.blockdiag \
  python3-sphinxcontrib.programoutput \
  python3-yaml \
  flake8 \
  git \
  bash-completion \
  sudo
RUN apt-get purge -y python

# Build stage: This builds on the base stage. The source tree and
# place where build results are stored get bind mounted into the
# container as /build and /build/src.
FROM deps
WORKDIR /build/src
ENTRYPOINT ["./build-deb-in-environment", "/build"]
