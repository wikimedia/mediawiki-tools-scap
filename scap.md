# Introduction

Scap is a tool for deploying MediaWiki, MediaWiki extensions, and
supporting software, to servers at the Wikimedia Foundation (WMF).
It's meant to be used by the WMF release engineering and SRE teams,
and trusted other parties.

This document describes some of the acceptance criteria for Scap, and
how they can be verified automatically.

Note that this document describes version 2 of Scap, not version 3,
which is a whole different code base. At least for now, this may
change later.


## Scap documentation

This document doesn't try to document Scap itself. Instead, see the
following sources.

* The docs directory in the source tree.
  - online: <https://gerrit.wikimedia.org/r/plugins/gitiles/mediawiki/tools/scap/+/master/docs/>
  - formatted: <https://doc.wikimedia.org/mw-tools-scap/>
* Wikitech:
  - version 2: <https://wikitech.wikimedia.org/wiki/Scap>
  - version 3: <https://wikitech.wikimedia.org/wiki/Scap3>


# scap version

This is a smoke test: if `scap version` runs, outputs something that
looks like a version number, and exits with a zero code, we're
satisfied. If this doesn't work, something is really badly wrong with
Scap or the test setup.

~~~scenario
given a built scap
when I run scap version
then the exit code is 0
then the output matches ^\d+(\.\d+)+(-\S+)?$
~~~


# scap sync without --canary-wait-time works

The `scap sync` command takes an optional `--canary-wait-time` option.
Make sure it works without the option or rather fails in the right way.

~~~scenario
given a built scap
when I run scap sync
then the exit code is 1
then the output matches scap failed: RuntimeError sync requires SSH agent forwarding
~~~



<!-- document metadata at end to not confuse Emacs syntax highlighting -->

---
title: Scap for deploying at WMF
author: WMF Release Engineering
bindings: scap.yaml
functions: scap.python
documentclass: report
classes:
- php
- patch
- scenario-disabled
...
