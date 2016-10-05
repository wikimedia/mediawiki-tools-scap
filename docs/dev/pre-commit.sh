#!/bin/bash
# example pre-commit hook for git which runs `arc lint` and `tox -e doc`
# before each commit, aborting the commit if there is a lint error.


lint=`arc lint`
if [ $? != 0 ]; then
    arc lint
    exit 1
fi

# Remove the remaining code if you don't want to rebuild docs on each commit
tox=`tox -e doc 2>&1`
if [ $? != 0 ]; then
    echo $tox
    exit $?
fi
