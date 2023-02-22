#!/bin/bash

set -eu -o pipefail

WHEELS_DIR=wheels

# Create wheels for dependencies
pip3 wheel -r requirements.txt -w $WHEELS_DIR
# Then create wheel for scap and put everything together
python3 setup.py bdist_wheel
mv ./dist/* $WHEELS_DIR
