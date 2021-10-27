#!/bin/bash
# example pre-commit hook for git which runs `flake8` and `tox -e doc`
# before each commit, aborting the commit if there is a lint error.


if ! lint=$(tox -e flake8 2>&1); then
    echo "$lint"
    exit 1
fi

# Remove the remaining code if you don't want to rebuild docs on each commit
if ! tox=$(tox -e doc 2>&1); then
    echo "$tox"
    exit 1
fi
