FROM debian:11 AS base

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y procps python3-venv

ENV PATH=${PATH}:/workspace/scap-venv/bin

COPY scap.cfg /etc/scap.cfg

ARG UID
ARG GID
RUN groupadd -g ${GID} -o deployment
RUN useradd -m -s /bin/bash -u ${UID} -g deployment deployer01

USER deployer01

#########
# SETUP #
#########

FROM base AS setup
COPY entrypoint.setup /
ENTRYPOINT ["/entrypoint.setup"]

##########
# DEPLOY #
##########

FROM base AS deploy
COPY entrypoint.deploy /
ENTRYPOINT ["/entrypoint.deploy"]
