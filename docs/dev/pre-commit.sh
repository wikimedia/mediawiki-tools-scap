#!/bin/bash
# example pre-commit hook for git which runs `flake8` and `tox -e doc`
# before each commit, aborting the commit if there is a lint error.


lint=`tox -e flake8 2>&1`
if [ $? != 0 ]; then
    echo $lint
    exit 1
fi

# Remove the remaining code if you don't want to rebuild docs on each commit
tox=`tox -e doc 2>&1`
if [ $? != 0 ]; then
    echo $tox
    exit $?
fi
