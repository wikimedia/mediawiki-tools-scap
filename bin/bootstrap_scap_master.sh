#!/bin/bash

# This script primes a deployment server/master with a Scap installation. The version will be the latest available tag
# in the code repository. Note that the script needs to run as a user with permissions to fetch from the Scap repo

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
bin/install_local_version.sh -u "$SCAP_USER" -t "$LATEST_TAG" .