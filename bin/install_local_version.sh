#!/bin/bash

# This script can be used to install Scap locally from Python wheels available via docker images. It is meant for the
# following use cases:
#    * Bootstrap/fix/stage a scap installation on a deploy server
#    * Install scap on a target host (wheels must be available locally on the target)
#    * Allow a RelEng operator to create their own installation in their home

set -eu -o pipefail

REGISTRY=docker-registry.wikimedia.org
SCAP_REPO=repos/releng/scap
BASE_SCAP_IMAGE_REPO=$REGISTRY/$SCAP_REPO
# When changing the supported distros, also update the authoritative list in install_world.py
SUPPORTED_DISTROS="bullseye bookworm"

function usage {
  cat <<HERE

   Usage: $0
     [-u|--user <USERNAME>]
     [--on-primary -t|--tag <TAG> [-s|--skip-install] [-d|--distros <D1,D2...>]]
     [--on-secondary -t|--tag <TAG>]

   Uses Python wheels to install scap in a Python3 venv in the user's HOME at HOME/scap. It has three modes of operation:
     * Install target host (DEFAULT):       Wheels for the host distro must be already available in the user's HOME
     * Install primary deployment server:   Wheels for supported distros and <TAG> will be downloaded prior to installation
     * Install secondary deployment server: Wheels for the host distro and <TAG> must be available at HOME/scap-wheels

   Options:
     -u, --user <USERNAME>       The user to install Scap for. If not passed, the env var USER is taken instead
     --on-primary                Signals this is a primary deployment host. Pull wheels from a distribution image before installing
       -t, --tag <TAG>           Install wheels for <TAG>. Must be a numeric version tag, 'latest' not accepted
       -s, --skip-install        Download the wheels, if missing, for <TAG> but do not install Scap
       -d, --distros <D1,D2...>  Override supported distro codenames
     --on-secondary              Signals this is a secondary deployment host. Mutually exclusive with '--on-primary'
       -t, --tag <TAG>           Install wheels for <TAG>. Must be a numeric version tag, 'latest' not accepted

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
  if ! echo "$SUPPORTED_DISTROS" | grep -q "$HOST_DISTRO"; then
    fail "System's distribution \"$HOST_DISTRO\" not supported by Scap"
  fi
}

function on_master {
  [ "$ON_PRIMARY" = y ] || [ "$ON_SECONDARY" = y ]
}

function parseArgs {
  ON_PRIMARY=
  ON_SECONDARY=
  INSTALL_USER=
  TAG=
  SKIP_PRIMARY_INSTALL=

  for arg in "$@"; do
    shift
    case "$arg" in
      --user)
        set -- "$@" '-u'
        ;;
      --on-primary)
        [ "$ON_SECONDARY" = y ] && fail "--on-primary and --on-secondary options are mutually exclusive"
        ON_PRIMARY=y
        ;;
      --on-secondary)
        [ "$ON_PRIMARY" = y ] && fail "--on-primary and --on-secondary options are mutually exclusive"
        ON_SECONDARY=y
        ;;
      --tag)
        set -- "$@" '-t'
        ;;
      --skip-install)
        set -- "$@" '-s'
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

  while getopts 'u:t:d:s' opt; do
    case "$opt" in
      u)
        INSTALL_USER=$OPTARG
        ;;
      t)
        if ! on_master; then
          usage
          exit 1
        fi
        TAG=$OPTARG
        ;;
      s)
        if [ "$ON_PRIMARY" != y ]; then
          usage
          exit 1
        fi
        SKIP_PRIMARY_INSTALL=y
        ;;
      d)
        if [ "$ON_PRIMARY" != y ]; then
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

  if on_master; then
    if [ -z "$TAG" ]; then
      echo -e "\n   Please specify --tag <TAG>"
      usage
      exit 1
    fi
  fi
}

function get_dist_dir_path_master {
  local DISTRO=$1
  echo "$BASE_DIST_DIR/$DISTRO/$TAG"
}

function get_dist_dir_path_target {
  local DISTRO=$1
  echo "$BASE_DIST_DIR/$DISTRO"
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

function verify_version_tag {
  TAGS_URL="https://$REGISTRY/v2/$SCAP_REPO/$HOST_DISTRO/tags/list"
  if ! curl -sSf "$TAGS_URL" | jq -r '.tags[]' | grep -v latest | grep -qw "$TAG"; then
    fail "Tag \"$TAG\" is not valid. Please specify an existing numeric version tag, e.g.: 4.131.0"
  fi
}

function verify_local_wheels_available {
  local DIST_DIR
  DIST_DIR=$($GET_DIST_DIR_PATH "$HOST_DISTRO")

  if [ ! -d "$DIST_DIR" ]; then
    fail "Scap distribution dir \"$DIST_DIR\" is missing. Maybe this is a primary deploy server? Please check usage"
  fi
}

function get_scap_version_distribution {
  local DISTRO=$1
  local IMAGE=${BASE_SCAP_IMAGE_REPO}/$DISTRO:$TAG
  local DIST_DIR=
  DIST_DIR=$(get_dist_dir_path_master "$DISTRO")

  if compgen -G "$DIST_DIR/*.whl" >/dev/null; then
    log "Scap version \"$TAG\" for distribution \"$DISTRO\" already exists locally. Nothing to retrieve"
    return
  fi

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

  log "Scap version \"$TAG\" for distribution \"$DISTRO\" successfully extracted at $DIST_DIR"
}

function get_distributions {
  for DISTRO in $SUPPORTED_DISTROS; do
    get_scap_version_distribution "$DISTRO"
  done
}

function clean_up_distributions {
  # Keep the 3 newest version dirs for each distro (by directory creation date, not version)
  for DISTRO in $SUPPORTED_DISTROS; do
    # shellcheck disable=SC2012
    for OLD_VERSION in $(ls --time=birth "$BASE_DIST_DIR/$DISTRO" | tail +4); do
      $AS_USER rm -rf "$BASE_DIST_DIR/$DISTRO/$OLD_VERSION"
      log "Deleted old wheels at \"$BASE_DIST_DIR/$DISTRO/$OLD_VERSION\""
    done
  done
}

function install_scap {
  function install_venv {
    $AS_USER python3 -m venv "$SCAP_VENV_DIR"
    # Upgrade pip first using the included wheel.
    $AS_USER "$SCAP_VENV_DIR"/bin/pip install --upgrade --no-deps "$DIST_DIR"/pip-*.whl
    # Then install everything else
    $AS_USER "$SCAP_VENV_DIR"/bin/pip install --no-deps "$DIST_DIR"/*.whl
    return $?
  }

  local DIST_DIR=
  DIST_DIR=$($GET_DIST_DIR_PATH "$HOST_DISTRO")
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
  if on_master; then
    log "Scap \"$TAG\" for \"$HOST_DISTRO\" successfully installed at $SCAP_VENV_DIR"
  else
    log "Scap for \"$HOST_DISTRO\" successfully installed at $SCAP_VENV_DIR"
  fi
}

parseArgs "$@"

HOST_DISTRO=$(lsb_release -cs)
AS_USER=
if [ "$INSTALL_USER" != "$(id -un)" ]; then
  AS_USER="sudo -su $INSTALL_USER"
fi
USER_HOME=$(eval echo "~$INSTALL_USER")
if on_master; then
  BASE_DIST_DIR=$USER_HOME/scap-wheels
  GET_DIST_DIR_PATH=get_dist_dir_path_master
else
  BASE_DIST_DIR=$USER_HOME
  GET_DIST_DIR_PATH=get_dist_dir_path_target
fi

verify_distro
verify_user
if on_master; then
  verify_version_tag
fi
if [ "$ON_PRIMARY" != y ]; then
  verify_local_wheels_available
fi
if [ "$ON_PRIMARY" = y ]; then
  get_distributions
  clean_up_distributions
  if [ "$SKIP_PRIMARY_INSTALL" = y ]; then
    log "Distributions downloaded. Skipping local installation on primary as requested"
    exit 0
  fi
fi
install_scap
