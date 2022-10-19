#!/bin/bash

# This script can be used to create a local, self-contained, fresh installation of scap from a local
# copy of the source code; or to update an already existing installation. It is meant for the
# following use cases:
#    * Bootstrap/stage a scap installation on a deploy server
#    * Allow a RelEng operator to create their own installation in their home
#
# The script can be used to install versions >=4.9.3 of scap

set -eu -o pipefail

function usage {
  cat <<HERE

   Usage: $0 [-u <USERNAME>] [-t <TAG>] <SCAP_SOURCE_DIR>

   Uses a scap git repository at <SCAP_SOURCE_DIR> to recreate a self-contained scap Python3
   venv in the user's HOME at $HOME/scap. If <USERNAME> is not passed, the environment variable USER
   is taken instead. An optional git <TAG> can be passed to specify the code revision to use when
   creating the venv.

   Note the user running this script needs to have permissions:
     * To read <SCAP_SOURCE_DIR>
     * To sudo as <USERNAME> (unless the user is already <USERNAME>)
     * If a tag is specified, permissions to write <SCAP_SOURCE_DIR> and to access the git remote
     pointed to by <SCAP_SOURCE_DIR>

HERE
}

function log {
  echo -e "INFO: $1"
}

function fail {
  echo -e "ERROR: $1. Aborting"
  exit 1
}

function git_scap {
  git -C "$SCAP_SOURCE_DIR" "$@"
}

function parseArgs {
  INSTALL_USER=
  TAG=

  while getopts 'u:t:' opt; do
    case "$opt" in
      u)
        INSTALL_USER=$OPTARG
        ;;
      t)
        TAG=$OPTARG
        ;;
      *)
        usage
        exit 1
        ;;
    esac
  done
  shift $((OPTIND - 1))

  (($# < 1)) && usage && exit 1

  SCAP_SOURCE_DIR=$1

  if [ -z "$INSTALL_USER" ]; then
    if [ -z "${USER:-}" ]; then
      fail "No user specified and USER var env is not set"
    fi
    INSTALL_USER=$USER
  fi
}

function verify_source_dir {
 if ! git_scap config remote.origin.url 2>/dev/null | grep -q mediawiki/tools/scap; then
   fail "Specified source path \"$SCAP_SOURCE_DIR\" does not seem to be a scap repository"
 fi
}

function verify_user {
  if ! id "$INSTALL_USER" &>/dev/null; then
    fail "Unknown user \"$INSTALL_USER\""
  fi
}

function verify_tag {
  if [ -n "$TAG" ]; then
    git_scap fetch --prune
    if ! git_scap rev-parse tags/"$TAG" &>/dev/null; then
     fail "Specified tag \"$TAG\" is not recognized"
    fi
  fi
}

function verify_args {
  verify_source_dir
  verify_user
  verify_tag
}

function check_out_tag {
  git_scap -c advice.detachedHead=false checkout "$TAG"
  # The trap ensures the script restores the git repo to the original checkout it found before
  # running. The call generates several lines of output, so we use the grep filter to trim it down
  # to the most informative of those
  trap "git_scap checkout - 2>&1 | grep -i switched" EXIT
  log "Tag \"$TAG\" checked out"
}

function create_scap_venv_for_user {
  local USER_HOME
  USER_HOME=$(eval echo "~$INSTALL_USER")

  # Directory to hold a copy of the previous version while we create a new
  # virtual environment with the version to be deployed.
  local OLD_VENV_DIR
  # Virtualenv directory for the scap deployment
  local VENV_DIR=${USER_HOME}/scap
  local AS_USER=
  local http_proxy=
  local https_proxy=

  if dnsdomainname | grep -q wmnet; then
    local SUBDOMAIN
    SUBDOMAIN=$(dnsdomainname | cut -d. -f1)
    export http_proxy="http://webproxy.${SUBDOMAIN}.wmnet:8080"
    export https_proxy=$http_proxy
  fi

  if [ ! "$INSTALL_USER" = "$(id -un)" ]; then
    if sudo -n -u "$INSTALL_USER" id &>/dev/null; then
      AS_USER="sudo -su $INSTALL_USER --preserve-env=http_proxy,https_proxy"
    else
      fail "Could not sudo to user \"$INSTALL_USER\""
    fi
  fi

  OLD_VENV_DIR=$($AS_USER mktemp -d "${USER_HOME}/.scap-venv.XXXXXX")

  # Signal we will want to restore the old venv in case of installation failure
  if [ -e "$VENV_DIR" ]; then
    $AS_USER mv "$VENV_DIR" "$OLD_VENV_DIR"
    trap '$AS_USER rm -fR "$VENV_DIR"; [ -e "$OLD_VENV_DIR" ] && $AS_USER mv "$OLD_VENV_DIR" "$VENV_DIR"' ERR
  fi

  $AS_USER python3 -m venv --clear "$VENV_DIR"
  $AS_USER "${VENV_DIR}"/bin/pip3 install wheel==0.37.1
  # --no-deps prevents pip from processing any requirements in SCAP_SOURCE_DIR
  # in conjunction with -r requirements.txt, none of the requirements transitive
  # dependencies will be installed
  $AS_USER "${VENV_DIR}"/bin/pip3 install --no-deps -r requirements.txt "$SCAP_SOURCE_DIR"

  # Since we have successfully installed we no more need to restore the old env
  trap - ERR
  $AS_USER rm -fR "$OLD_VENV_DIR"

  if [ -n "$TAG" ]; then
    log "Scap \"$TAG\" successfully installed at \"$VENV_DIR\""
  else
    log "Scap successfully installed at \"$VENV_DIR\""
  fi
}

function install_scap {
  [ -n "$TAG" ] && check_out_tag
  create_scap_venv_for_user
}

parseArgs "$@"
verify_args
install_scap
