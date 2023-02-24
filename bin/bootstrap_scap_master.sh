#!/bin/bash

# This script primes a deployment server/master with a Scap installation. Latest Scap release available will be used

set -eu -o pipefail

if (($# < 2)); then
    echo "Usage: $0 scap_user scap_source_path"
    exit 1
fi

SCAP_USER="$1"
SCAP_SOURCE_PATH="$2"

cd "$SCAP_SOURCE_PATH"
git fetch
LATEST_TAG=$(git tag --sort -taggerdate | head -1)
git -c advice.detachedHead=false checkout "$LATEST_TAG"
trap "git checkout - 2>&1 | grep -i switched" EXIT
bin/install_local_version.sh -u "$SCAP_USER" --on-deploy -t "$LATEST_TAG"
