#!/bin/bash

set -eu -o pipefail

WHEELS_DIR=wheels

function announce {
    echo
    echo "** $* **"
    echo
}
    

# Make sure we're using up-to-date pip, whose wheel will be included
# for use on installation targets.
announce Upgrading pip
pip3 install --upgrade pip

announce Installing build package
pip3 install build

announce Creating wheels from requirements.txt
# Create wheels for dependencies
pip3 wheel -w $WHEELS_DIR \
     pip \
     -r requirements.txt

announce Building the Scap wheel
# Then create wheel for scap and put everything together
python3 -m build --wheel
mv ./dist/* $WHEELS_DIR
