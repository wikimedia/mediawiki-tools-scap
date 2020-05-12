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
then the output matches pattern ^\d+(\.\d+)+(-\S+)?$
~~~


# Applying security patches for train

[Train]: https://wikitech.wikimedia.org/wiki/Heterogeneous_deployment/Train_deploys#Apply_security_patches

[Train][] is the weekly manual deployment process at the Wikimedia
Foundation. WMF is in the process of automating more of it. Scap has a
plugin that adds the `apply-patches` subcommand that implements the
application of security patches as part of the train deployment
process. This section explains the process as done manually, and then
how the plugin's behaviour is verified.

## The old manual process

This all needs to be run on the deployment server, as that's where the
security patches live. Security patches are sensitive and embargoed,
and can't be made public.

Source code for MediaWiki and its extensions is checked out as part of
the earlier "branch cut" part of the train process. The tree looks
like:

```
/srv/mediawiki-staging/php-1.35.0-wmf.30
`-- extensions
    `-- Goat
```

Similarly, the patches live under the `/srv/patches` directory, and
are put there before the train starts. For example:

```
/srv/patches/1.35.0-wmf.30
+-- core
|   +-- 01-T123.patch
|   `-- 02-T456.patch
`-- extensions
    `-- Goat
        +-- 01-T222.patch
        `-- 02-T333.patch
```

Patch files are named using an ordering prefix and a Phabricator task
identifier. The filename ends in `.patch`. Any other files can be
ignored.

The patches under `core` are applied to MediaWiki core, and those
under `extensions` are applied to the relevant extension. Thus:

* `/srv/patches/1.35.0-wmf.30/core/01-T123.patch` is applied in
  `/srv/mediawiki-staging/php-1.35.0-wmf.30`
* `/srv/patches/1.35.0-wmf.30/extensions/Goat/01-T222.patch` is applied in
  `/srv/mediawiki-staging/php-1.35.0-wmf.30/extensions/Goat`

Patches are applied, in the order of the filename ordering prefixes.
For example:

```{.sh .numberLines}
$ cd /srv/mediawiki-staging/php-1.35.0-wmf.30
$ git apply --check --3way /srv/patches/1.35.0-wmf.30/core/01-T123.patch
$ git am --3way /srv/patches/1.35.0-wmf.30/core/01-T123.patch
```

The `git apply --check` command verifies that the patch can be
applied. The `git am` command actually applies the patch. If either
command fails, the train should stop until the problem can be fixed.

## Verifying the patching process

Scap has two subcommands for the patching process:

* `scap list-patches` lists all the patches for a given train
* `scap apply-patches` actually applies the patches, if they can all
  be applied

We verify the patching process by creating dummy git repositories for
MW core and extensions, and dummy patches, and then verifying that the
subcommands work.

### Data files for testing

These files are used to construct the dummy git repositories and patch
sets. Their contents doesn't really matter, except for the following:

* `code.php` is the unpatched file
* `patch` applies to the unpatched file
* `conflict` doesn't apply to the unpatched file due to a conflict

```{.file #code.php .php .numberLines}
<?php
/* Pretend this is some PHP code. */
```

```{.file #patch .patch .numberLines}
From 640e813fdba8ea7ae27b088e36574889c293d758 Mon Sep 17 00:00:00 2001
From: Lars Wirzenius <lwirzenius@wikimedia.org>
Date: Tue, 17 Mar 2020 14:45:11 +0200
Subject: [PATCH] ok

---
 code.php | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)

diff --git a/code.php b/code.php
index fe577c7..acc4d52 100644
--- a/code.php
+++ b/code.php
@@ -1,2 +1,2 @@
 <?php
-/* Pretend this is some PHP code. */
+/* This is some updated PHP code */
-- 
2.20.1
```

```{.file #conflict .patch .numberLines}
From 640e813fdba8ea7ae27b088e36574889c293d758 Mon Sep 17 00:00:00 2001
From: Lars Wirzenius <lwirzenius@wikimedia.org>
Date: Tue, 17 Mar 2020 14:45:11 +0200
Subject: [PATCH] conflict

---
 code.php | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)

diff --git a/code.php b/code.php
index fe577c7..acc4d52 100644
--- a/code.php
+++ b/code.php
@@ -1,2 +1,2 @@
 <?php
-/* This line doesn't exist in the unpatched file. */
+/* This is some updated PHP code */
-- 
2.20.1
```

## There are no patches to list

This is the simplest case: there are no patches at all. In this
scenario, we only check `list-patches`

~~~scenario
given an empty git repository at patches
and empty directory patches/wmf.30
when I run scap list-patches -Dpatch_path:patches --train wmf.30
then the exit code is 0
and the output is empty
~~~

## There are some patches to list

In this case, there are some patches, and they all apply cleanly. We
check for the core and extension case. In this scenario, we only check
`list-patches`

~~~scenario
given an empty git repository at patches
and file wmf.30/core/01-T123.patch from patch committed in patches
and file wmf.30/core/02-T123.patch from patch committed in patches
and file wmf.30/extensions/fooext/01-T789.patch from patch committed in patches
when I run scap list-patches -Dpatch_path:patches --train wmf.30
then the exit code is 0
and the output matches first core/01-T123.patch and later core/02-T123.patch
and the output matches pattern extensions/fooext/01-T789.patch
and the output does not match T456
and the output does not match T999
~~~

## There are badly named patch files when listing

In this case there's a file in the patches tree that is badly named.
We should get an error message, but no actual patches.

~~~scenario
given an empty git repository at patches
and file 1.35.0-wmf.30/core/01-T123.patch from patch committed in patches
and file 1.35.0-wmf.30/core/02-T456.patch.failed from patch committed in patches
and file 1.35.0-wmf.30/core/03-T789.patch.failed from patch committed in patches
when I run scap list-patches -Dpatch_path:patches --train 1.35.0-wmf.30
then the exit code is 1
and the output does not match T123
and stderr contains "Bad filename for patch:.*T456"
and stderr contains "Bad filename for patch:.*T789"
~~~

## There are no patches to apply

If there are no patches, the core and extension repositories should
not be changed when apply-patches is run.

~~~scenario
given an empty git repository at staging/php-wmf.30
and empty directory patches/wmf.30
and file code.php committed in staging/php-wmf.30
and an empty git repository at staging/php-wmf.30/extensions/fooext
and file code.php committed in staging/php-wmf.30/extensions/fooext
and an empty git repository at patches
~~~

The rest of the scenario applies patches and verifies nothing is
changed, since there weren't any patches.

~~~ scenario
when I run scap apply-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 0
then repository staging/php-wmf.30 is not changed
then repository staging/php-wmf.30/extensions/fooext is not changed

when I run scap test-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 0
~~~


## Apply a good patch for core

If there's a patch for core, it gets applied, but extensions are not
touched.

~~~scenario
given an empty git repository at staging/php-wmf.30
and file code.php committed in staging/php-wmf.30
and an empty git repository at staging/php-wmf.30/extensions/fooext
and file code.php committed in staging/php-wmf.30/extensions/fooext
and an empty git repository at patches
and file wmf.30/core/01-T123.patch from patch committed in patches

when I run scap test-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 0

when I run scap apply-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 0
and the staging/php-wmf.30 checkout is clean and committed
then the change in staging/php-wmf.30 matches patch
then repository staging/php-wmf.30/extensions/fooext is not changed
~~~


## Apply a good patch for an extension

If there's a patch for an extension, it gets applied, but core is not
touched.

~~~scenario
given an empty git repository at staging/php-wmf.30
and file code.php committed in staging/php-wmf.30
and an empty git repository at staging/php-wmf.30/extensions/fooext
and file code.php committed in staging/php-wmf.30/extensions/fooext
and an empty git repository at patches
and file wmf.30/extensions/fooext/01-T123.patch from patch committed in patches

when I run scap test-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 0

when I run scap apply-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 0
and repository staging/php-wmf.30 is not changed
and the staging/php-wmf.30/extensions/fooext checkout is clean and committed
and the change in staging/php-wmf.30/extensions/fooext matches patch
~~~


## There is also badly named file among patches

If there's a good patch, as above, but also an extra file, nothing is
applied. We use a file named `.failed` since there's an old plugin for
scap that renames patches it can't apply, and we'd like those to be
cleaned up before deployment, so that a known security hole isn't
accidentally deployed without its patch.

~~~scenario
given an empty git repository at staging/php-wmf.30
and file code.php committed in staging/php-wmf.30
and an empty git repository at patches
and file wmf.30/core/01-T123.patch from patch committed in patches
and file wmf.30/core/01-T456.patch.failed from patch committed in patches

when I run scap test-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 1
then stderr contains "Unknown file in patches"

when I run scap apply-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 1
then stderr contains "Unknown file in patches"
then repository staging/php-wmf.30 is not changed
~~~


## There is a bad patch to apply

In this case, all patches are named OK, but one of them fails to
apply correctly. Nothing should be applied.

~~~scenario
given an empty git repository at staging/php-wmf.30
and file code.php committed in staging/php-wmf.30
and an empty git repository at staging/php-wmf.30/extensions/fooext
and file code.php committed in staging/php-wmf.30/extensions/fooext
and an empty git repository at patches
and file wmf.30/extensions/fooext/01-T123.patch from conflict committed in patches

when I run scap test-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 70
then stderr contains "patch does not apply"

when I run scap apply-patches -Dstage_dir:staging -Dpatch_path:patches --train wmf.30
then the exit code is 70
then stderr contains "patch does not apply"
and repository staging/php-wmf.30 is not changed
and repository staging/php-wmf.30/extensions/fooext is not changed
~~~


# scap sync fails

The `scap sync` command is being renamed to `scap sync-world`. The old
command now gives an error.

~~~scenario
given a built scap
when I run scap sync
then the exit code is 1
then stderr contains "scap sync-world"
~~~



# scap sync-world without --canary-wait-time works

The `scap sync` command takes an optional `--canary-wait-time` option.
Make sure it works without the option or rather fails in the right way.

~~~scenario
given a built scap
when I run scap sync-world
then the exit code is 1
then stderr contains "scap failed: RuntimeError sync-world requires SSH agent forwarding"
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
