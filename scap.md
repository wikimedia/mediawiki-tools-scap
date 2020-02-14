---
title: Scap for deploying at WMF
author: WMF Release Engineering
date: work in progress
bindings: scap.yaml
functions: scap.python
...

Introduction
=============================================================================

Scap is a tool for deploying MediaWiki, MediaWiki extensions, and
supporting software, to servers at the Wikimedia Foundation (WMF).
It's meant to be used by the WMF release engineering and SRE teams,
and trusted other parties.

This document describes some of the acceptance criteria for Scap, and
how they can be verified automatically.

Note that this document describes version 2 of Scap, not version 3,
which is a whole different codebase. At least for now, this may
change later.


Scap documentation
=============================================================================

This document doesn't try to document Scap itself. Instead, see the
following sources.

* The docs directory in the source tree.
  - online: <https://gerrit.wikimedia.org/r/plugins/gitiles/mediawiki/tools/scap/+/master/docs/>
  - formatted: <https://doc.wikimedia.org/mw-tools-scap/>
* Wikitech:
  - version 2: <https://wikitech.wikimedia.org/wiki/Scap>
  - version 3: <https://wikitech.wikimedia.org/wiki/Scap3>


Acceptance criteria
=============================================================================

This chapter documents the acceptance criteria for the software, and
how they're verified automatically, in the form of scenarios.


scap version works
-----------------------------------------------------------------------------

This is a smoke test: if "scap version" runs, outputs something that
looks like a version number, and exits with a zero code, we're
satisfied.

~~~scenario
given a built scap
when I run scap version
then the exit code is 0
then the output matches ^\d+(\.\d+)+(-\S+)?$
~~~
