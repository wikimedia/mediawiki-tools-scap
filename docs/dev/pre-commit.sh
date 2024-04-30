#!/bin/bash
# example pre-commit hook for git which runs `tox -e lint` and `tox -e doc`
# before each commit, aborting the commit if there any error.


if ! lint=$(tox -e lint 2>&1); then
    echo "$lint"
    exit 1
fi

# Remove the remaining code if you don't want to rebuild docs on each commit
if ! tox=$(tox -e doc 2>&1); then
    echo "$tox"
    exit 1
fi
