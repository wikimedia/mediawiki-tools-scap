*Scap* is a tool for deploying MediaWiki, MediaWiki extensions, and
and trusted other parties. In other words, Scap updates the software
that runs Wikipedia.
how they can be verified automatically. As the document is written
many years after the software, it doesn't capture all the acceptance
criteria. New criteria are mostly added as the software is changed, to
cover the changed part.
which is a whole different code base.
# Acceptance criteria

## Expected subcommands exist
then I can run scap apply-patches --help
then I can run scap cdb-json-refresh --help
then I can run scap cdb-rebuild --help
then I can run scap clean --help
then I can run scap deploy --help
then I can run scap deploy-local --help
then I can run scap deploy-log --help
then I can run scap deploy-mediawiki --help
then I can run scap fortune --help
then I can run scap lock --help
then I can run scap patch --help
then I can run scap prep --help
then I can run scap pull --help
then I can run scap pull-master --help
then I can run scap say --help
then I can run scap security-check --help
then I can run scap sync --help
then I can run scap sync-canary --help
then I can run scap sync-dir --help
then I can run scap sync-file --help
then I can run scap sync-l10n --help
then I can run scap sync-wikiversions --help
then I can run scap sync-world --help
then I can run scap test-progress --help
then I can run scap update-interwiki-cache --help
then I can run scap update-wikiversions --help
then I can run scap version --help
then I can run scap wikiversions-compile --help
then I can run scap wikiversions-inuse --help
then I can run scap wmf-beta-autoupdate --help
## scap version
then the output matches pattern "^\d+(\.\d+)+(-\S+)?$"
## Applying security patches for train
Foundation. Scap has an `apply-patches` subcommand that applies
security patches as part of the train deployment process. This section
explains the process as done manually, and then how the plugin's
behaviour is verified.
### The old manual process
earlier "scap prep" part of the train process. The tree looks like:
### Applying security patches automatically
Scap has a subcommand, `scap apply-patches`, which attempts to apply
the patches. The command can be run many times, and it reports what
the outcome of applying each patch is. Any problems need to be fixed
by the person using Scap, as the software does not try to fix any
problems automatically.
This scenario constructs several git repositories, for MediaWiki core,
an extension, and a skin, all of which consist of
a file `file.php`, which contains several lines of text. There are
also patches for each repository: two good ones that change different
parts of the file, and one that expects a different content in the
file. See below for actual file contents.
First we set up the repository that pretends to be MediaWiki core,
and the patches for it.
~~~scenario
given an empty git repository at php-wmf.30
given file file.patch from file.patch
when I invoke, in php-wmf.30, git am ../file.patch
then the exit code is 0
then git repository php-wmf.30 is clean
given file patches/wmf.30/core/01-first.patch from good-patch-1
given file patches/wmf.30/core/02-second.patch from bad-patch
given file patches/wmf.30/core/03-third.patch from good-patch-2
~~~
Note that we populate the repository by applying a patch. This is so
that the version history matches what the patches have.
We then do the same for a mock extension: repository for the extension
and patches for it.
given an empty git repository at php-wmf.30/extensions/Foo
when I invoke, in php-wmf.30/extensions/Foo, git am ../../../file.patch
then git repository php-wmf.30/extensions/Foo is clean
given file patches/wmf.30/extensions/Foo/01-first.patch from good-patch-1
given file patches/wmf.30/extensions/Foo/02-second.patch from bad-patch
given file patches/wmf.30/extensions/Foo/03-third.patch from good-patch-2
~~~
We do the same for a mock skin: repository for the extension and
patches for it.
given an empty git repository at php-wmf.30/skins/Bar
when I invoke, in php-wmf.30/skins/Bar, git am ../../../file.patch
then git repository php-wmf.30/skins/Bar is clean
given file patches/wmf.30/skins/Bar/01-first.patch from good-patch-1
given file patches/wmf.30/skins/Bar/02-second.patch from bad-patch
given file patches/wmf.30/skins/Bar/03-third.patch from good-patch-2
~~~
Then we run `scap apply-patches`. This fails, because one of the
patches is bad, but we verify that the output is as expected and that
the repositories are left in the expected state.
given an installed scap
when I run scap apply-patches -Dstage_dir:. -Dpatch_path:patches --train wmf.30
then the output matches pattern "^\[APPLIED\] /.*/patches/wmf.30/core/01-first.patch"
then the output matches pattern "^\[FAILED\] /.*/patches/wmf.30/core/02-second.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/core/03-third.patch"
then the output matches pattern "^\[APPLIED\] /.*/patches/wmf.30/extensions/Foo/01-first.patch"
then the output matches pattern "^\[FAILED\] /.*/patches/wmf.30/extensions/Foo/02-second.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/extensions/Foo/03-third.patch"
then the output matches pattern "^\[APPLIED\] /.*/patches/wmf.30/skins/Bar/01-first.patch"
then the output matches pattern "^\[FAILED\] /.*/patches/wmf.30/skins/Bar/02-second.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/skins/Bar/03-third.patch"
then git repository php-wmf.30 is dirty
then git repository php-wmf.30/extensions/Foo is dirty
then git repository php-wmf.30/skins/Bar is dirty
If we run `scap apply-patches` at this point, it won't do anything,
and will tell us there is a problem. Being able to re-run the command
is useful: it tells you what patches have problems.
when I run scap apply-patches -Dstage_dir:. -Dpatch_path:patches --train wmf.30
then the exit code is 1
then the output matches pattern "^\[FAILED\] /.*/patches/wmf.30/core/01-first.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/core/02-second.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/core/03-third.patch"
then the output matches pattern "^\[FAILED\] /.*/patches/wmf.30/extensions/Foo/01-first.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/extensions/Foo/02-second.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/extensions/Foo/03-third.patch"
then the output matches pattern "^\[FAILED\] /.*/patches/wmf.30/skins/Bar/01-first.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/skins/Bar/02-second.patch"
then the output matches pattern "^\[SKIPPED\] /.*/patches/wmf.30/skins/Bar/03-third.patch"
then git repository php-wmf.30 is dirty
then git repository php-wmf.30/extensions/Foo is dirty
then git repository php-wmf.30/skins/Bar is dirty
If we fix the problem, by terminating the in-progress `git am` in each
troubled git repository, and removing the offending patch, things can
progress. Note that in real life, it would be better to modify the
offending patch to keep the security fix, and not just blindly drop
it. However, that is hard to do in automated tests, so we just remove
it.
~~~scenario
when I run, in php-wmf.30, git am --abort
when I run, in php-wmf.30/extensions/Foo, git am --abort
when I run, in php-wmf.30/skins/Bar, git am --abort
when I remove patches/wmf.30/core/02-second.patch
when I remove patches/wmf.30/extensions/Foo/02-second.patch
when I remove patches/wmf.30/skins/Bar/02-second.patch
then git repository php-wmf.30 is clean
then git repository php-wmf.30/extensions/Foo is clean
then git repository php-wmf.30/skins/Bar is clean
We can now run `scap apply-patches` again and it succeeds.
~~~scenario
when I run scap apply-patches -Dstage_dir:. -Dpatch_path:patches --train wmf.30
then the exit code is 0
then the output matches pattern "\[OK\] /.*/patches/wmf.30/core/01-first.patch"
then the output matches pattern "\[APPLIED\] /.*/patches/wmf.30/core/03-third.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/extensions/Foo/01-first.patch"
then the output matches pattern "\[APPLIED\] /.*/patches/wmf.30/extensions/Foo/03-third.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/skins/Bar/01-first.patch"
then the output matches pattern "\[APPLIED\] /.*/patches/wmf.30/skins/Bar/03-third.patch"
then git repository php-wmf.30 is clean
then git repository php-wmf.30/extensions/Foo is clean
then git repository php-wmf.30/skins/Bar is clean
~~~
If we run `scap apply-patches` yet again, after all patches are
already applied, it's a benign operation.
when I run scap apply-patches -Dstage_dir:. -Dpatch_path:patches --train wmf.30
then the exit code is 0
then the output matches pattern "\[OK\] /.*/patches/wmf.30/core/01-first.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/core/03-third.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/extensions/Foo/01-first.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/extensions/Foo/03-third.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/skins/Bar/01-first.patch"
then the output matches pattern "\[OK\] /.*/patches/wmf.30/skins/Bar/03-third.patch"
then git repository php-wmf.30 is clean
then git repository php-wmf.30/extensions/Foo is clean
then git repository php-wmf.30/skins/Bar is clean
All is well. The train can continue securely.
~~~{#file.patch .file}
From 3b67b77a40d48ef3b60bde6911e4b3eb581d011b Mon Sep 17 00:00:00 2001
From: Lars Wirzenius <lwirzenius@wikimedia.org>
Date: Wed, 24 Mar 2021 15:41:30 +0200
Subject: [PATCH] file.php
---
 file.php | 9 +++++++++
 1 file changed, 9 insertions(+)
 create mode 100644 file.php

diff --git a/file.php b/file.php
new file mode 100644
index 0000000..8675b0b
--- /dev/null
+++ b/file.php
@@ -0,0 +1,9 @@
+This is line 1
+This is line 2
+This is line 3
+This is line 4
+This is line 5
+This is line 6
+This is line 7
+This is line 8
+This is line 9
-- 
2.30.2
~~~{#good-patch-1 .file .patch}
From 053ebb53a0a1fb1660f9e210984c589c696da474 Mon Sep 17 00:00:00 2001
From: Lars Wirzenius <lwirzenius@wikimedia.org>
Date: Wed, 24 Mar 2021 15:42:21 +0200
Subject: [PATCH] p1
---
 file.php | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)
diff --git a/file.php b/file.php
index 8675b0b..367ef35 100644
--- a/file.php
+++ b/file.php
@@ -1,4 +1,4 @@
-This is line 1
+This is modified line 1
 This is line 2
 This is line 3
 This is line 4
-- 
2.30.2
~~~
~~~{#good-patch-2 .file .patch}
From e417a4bcd9e2229828c144cbf815f25b189341b2 Mon Sep 17 00:00:00 2001
From: Lars Wirzenius <lwirzenius@wikimedia.org>
Date: Wed, 24 Mar 2021 15:43:00 +0200
Subject: [PATCH] p2

---
 file.php | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)

diff --git a/file.php b/file.php
index 8675b0b..699a993 100644
--- a/file.php
+++ b/file.php
@@ -6,4 +6,4 @@ This is line 5
 This is line 6
 This is line 7
 This is line 8
-This is line 9
+This is modified line 9
-- 
2.30.2
~~~{#bad-patch .file .patch}
From 4e140c14ea23d56e27e8a4fee86ba05c1197125d Mon Sep 17 00:00:00 2001
From: Lars Wirzenius <lwirzenius@wikimedia.org>
Date: Mon, 15 Mar 2021 16:32:10 +0200
Subject: [PATCH 2/2] line 5 to 100
---
 file.php | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)
diff --git a/file.php b/file.php
index 04eddf6..41b6c63 100644
--- a/file.php
+++ b/file.php
@@ -2,7 +2,7 @@ This is line 1
 This is line 2
 This is line 3
 This is line 4
-This is line 42
+This is line 100
 This is line 6
 This is line 7
 This is line 8
-- 
2.30.2
```
## scap sync fails
## scap sync-world without --canary-wait-time works
The `scap sync-world` command takes an optional `--canary-wait-time` option.

title: Scap
subtitle: Tool for deploying MediaWiki at WMF
bindings: 
- subplot/scap.yaml
functions: 
- subplot/scap.py
- subplot/vendored/runcmd.py