#!/bin/bash

# This script can be used to install Scap locally from Python wheels available via docker images. It is meant for the
# following use cases:
#    * Bootstrap/stage a scap installation on a deploy server
#    * Install scap on a target host (wheels must be available locally on the target)
#    * Allow a RelEng operator to create their own installation in their home
#
# The script can be used to install versions >=4.42.0 of scap

set -eu -o pipefail

BASE_SCAP_IMAGE_REPO=docker-registry.wikimedia.org/repos/releng/scap
# When changing the supported distros, also update the authoritative list in install_world.py
SUPPORTED_DISTROS="buster bullseye"

function usage {
  cat <<HERE

   Usage: $0 [-u|--user <USERNAME>] [--on-deploy [-t|--tag <TAG>] [-d|--distros <D1,D2...>]]

   Uses Python wheels to install scap in a Python3 venv in the user's HOME at $HOME/scap. It has two modes of operation:
     * Install target host (DEFAULT): Wheels for the host distro must be already available in the user's HOME
     * Install deployment server:     Wheels will be downloaded prior to installation

   Optional arguments:
     -u, --user <USERNAME>       The user to install Scap for. If not passed, the env var USER is taken instead
     --on-deploy                 Signals this is a deployment host. Pull wheels from a distribution image before installing
       -t, --tag <TAG>           Install wheels for <TAG>. Default is 'latest' (honored only if --on-deploy specified)
       -d, --distros <D1,D2...>  Override supported distro codenames (honored only if --on-deploy specified)

   Note the user running this script needs to have permissions to:
     * sudo as <USERNAME> (unless the user is already <USERNAME>)
     * Call the docker client
HERE
}

function log {
  echo -e "INFO: $1"
}

function fail {
  echo -e "ERROR: $1. Aborting"
  exit 1
}

function verify_distro {
  local DISTRO
  DISTRO=$(lsb_release -cs)

  if ! echo "$SUPPORTED_DISTROS" | grep -q "$DISTRO"; then
    fail "System's distribution \"$DISTRO\" not supported by Scap"
  fi
}

function parseArgs {
  ON_DEPLOY_SERVER=
  INSTALL_USER=
  TAG=latest

  for arg in "$@"; do
    shift
    case "$arg" in
      --user)
        set -- "$@" '-u'
        ;;
      --on-deploy)
        ON_DEPLOY_SERVER=y
        ;;
      --tag)
        set -- "$@" '-t'
        ;;
      --distros)
        set -- "$@" '-d'
        ;;
      -h)
        usage
        exit 1
        ;;
      *)
        set -- "$@" "$arg"
        ;;
    esac
  done

  while getopts 'u:t:d:' opt; do
    case "$opt" in
      u)
        INSTALL_USER=$OPTARG
        ;;
      t)
        if [ "$ON_DEPLOY_SERVER" != y ]; then
          usage
          exit 1
        fi
        TAG=$OPTARG
        ;;
      d)
        if [ "$ON_DEPLOY_SERVER" != y ]; then
          usage
          exit 1
        fi
        SUPPORTED_DISTROS=${OPTARG//,/ }
        ;;
      *)
        usage
        exit 1
        ;;
    esac
  done

  if [ -z "$INSTALL_USER" ]; then
    if [ -z "${USER:-}" ]; then
      fail "No user specified and USER var env is not set"
    fi
    INSTALL_USER=$USER
  fi

  if [ "$ON_DEPLOY_SERVER" != y ]; then
    TAG=n
  fi
}

function verify_user {
  if ! id "$INSTALL_USER" &>/dev/null; then
    fail "Unknown user \"$INSTALL_USER\""
  fi

  if [ "$INSTALL_USER" != "$(id -un)" ]; then
    if ! sudo -n -u "$INSTALL_USER" id &>/dev/null; then
      fail "Cannot sudo to user \"$INSTALL_USER\""
    fi
  fi
}

function verify_local_wheels_available {
  local DIST_DIR
  DIST_DIR=$BASE_DIST_DIR/$(lsb_release -cs)

  if [ ! -d "$DIST_DIR" ]; then
    fail "Scap distribution dir \"$DIST_DIR\" is missing. Maybe this is a deploy server? Please check usage"
  fi
}

function get_scap_distribution {
  local DISTRO=$1
  local IMAGE=${BASE_SCAP_IMAGE_REPO}/$DISTRO:$TAG
  local DIST_DIR=$BASE_DIST_DIR/$DISTRO

  docker pull "$IMAGE" >/dev/null

  if [ -d "$DIST_DIR" ]; then
    $AS_USER rm -rf "$DIST_DIR"
  fi
  $AS_USER mkdir -p "$DIST_DIR"

  local CONT_ID
  CONT_ID=$(docker create "$IMAGE")
  trap 'docker rm "$CONT_ID" >/dev/null' EXIT

  local TEMP_WHEELS
  TEMP_WHEELS=$(mktemp --tmpdir -d scap-wheels.XXX)
  chmod 'go=rx' "$TEMP_WHEELS"
  # Target user may not have permissions to run docker
  docker cp "$CONT_ID":/wheels "$TEMP_WHEELS"
  $AS_USER cp -r "$TEMP_WHEELS"/wheels/* "$DIST_DIR"

  rm -rf "$TEMP_WHEELS"
  docker rm "$CONT_ID" >/dev/null
  trap - EXIT

  log "Scap distribution successfully extracted at $DIST_DIR"
}

function install_scap_venv_for_user {
  function install_venv {
    $AS_USER python3 -m venv "$SCAP_VENV_DIR"
    $AS_USER "$SCAP_VENV_DIR"/bin/pip install --no-deps "$DIST_DIR"/*.whl
    return $?
  }

  local DISTRO
  DISTRO=$(lsb_release -cs)
  local DIST_DIR=$BASE_DIST_DIR/$DISTRO
  local SCAP_VENV_DIR=${USER_HOME}/scap
  local OLD_SCAP_VENV_DIR=

  if [ -d "$SCAP_VENV_DIR" ]; then
    OLD_SCAP_VENV_DIR=$($AS_USER mktemp --tmpdir -d scap.XXX)
    $AS_USER mv "$SCAP_VENV_DIR" "$OLD_SCAP_VENV_DIR"
    trap '[ -d "$SCAP_VENV_DIR" ] && $AS_USER rm -rf "$SCAP_VENV_DIR";'\
'$AS_USER mv "$OLD_SCAP_VENV_DIR"/scap "$SCAP_VENV_DIR";'\
'$AS_USER rmdir "$OLD_SCAP_VENV_DIR";'\
'echo -e "\nInstallation canceled. Restoring previous Scap version"' EXIT
  fi

  # If any of the python commands fails, we retry once. Palliative solution for T337394.
  # Note `set -e` has no effect inside of a function when its exit status is checked in an if statement, so
  # `install_venv` handles its status explicitly
  if ! install_venv; then
    log 'Retrying creation of virtual environment'
    install_venv
  fi

  # At this point installation has succeeded and we don't need to restore the old env anymore
  trap - EXIT
  if [ -d "$OLD_SCAP_VENV_DIR" ]; then
    $AS_USER rm -rf "$OLD_SCAP_VENV_DIR"
  fi

  echo
  if [ "$TAG" = n ]; then
    log "Scap from local $DISTRO wheels successfully installed at $SCAP_VENV_DIR"
  elif [ "$TAG" = latest ]; then
    log "Latest Scap for $DISTRO successfully installed at $SCAP_VENV_DIR"
  else
    log "Scap \"$TAG\" for $DISTRO successfully installed at $SCAP_VENV_DIR"
  fi
}

function install_scap {
  AS_USER=
  if [ "$INSTALL_USER" != "$(id -un)" ]; then
    AS_USER="sudo -su $INSTALL_USER"
  fi

  if [ "$ON_DEPLOY_SERVER" = y ]; then
    for DISTRO in $SUPPORTED_DISTROS; do
      get_scap_distribution "$DISTRO"
    done
  fi

  install_scap_venv_for_user
}

parseArgs "$@"
verify_distro
verify_user

USER_HOME=$(eval echo "~$INSTALL_USER")
BASE_DIST_DIR=$USER_HOME/scap-wheels
if [ "$ON_DEPLOY_SERVER" != y ]; then
  BASE_DIST_DIR=$USER_HOME
  verify_local_wheels_available
fi
install_scap
