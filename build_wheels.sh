#!/bin/bash

set -eu -o pipefail

WHEELS_DIR=wheels

# Make sure we're using up-to-date pip, whose wheel will be included
# for use on installation targets.
pip3 install --upgrade pip

# Create wheels for dependencies
pip3 wheel -w $WHEELS_DIR \
     pip \
     -r requirements.txt
# Then create wheel for scap and put everything together
python3 setup.py bdist_wheel
mv ./dist/* $WHEELS_DIR
