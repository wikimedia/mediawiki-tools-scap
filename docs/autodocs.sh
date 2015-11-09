#!/bin/bash
# run sphinx-autobuild

sphinx-autobuild docs/ docs/_build/html/ -p 8001 -H 0.0.0.0
