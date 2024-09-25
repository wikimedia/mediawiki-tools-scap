#!/bin/bash

set -eu -o pipefail

WHEELS_DIR=wheels

REQS=
for req in "$@"; do
  REQS="$REQS -r $req"
done

# shellcheck disable=SC2086
# Create wheels for dependencies
pip3 wheel -w $WHEELS_DIR $REQS
# Then create wheel for scap and put everything together
python3 setup.py bdist_wheel
mv ./dist/* $WHEELS_DIR
