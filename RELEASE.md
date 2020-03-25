# Release procedure for Scap

This is a detailed release procedure for the Scap software. It's meant
to be followed by a member of the Release Engineering team.

* Land all changes in master, test them locally.
* Pick version number.
* Test Debian package build locally.
* Test changes on the beta cluster.
* Tag the release in git.
* Ask SRE to build and install a new Debian package


## Land all changes in master, test them locally

Before making a release, make sure all the changes needed for the
release have landed on the master branch. Then verify that the local
test suite works.

```sh
$ scripts/check
...
Everything seems OK.
$ 
```

## Pick version number

Pick a new version number, called VERS in these instructions; a
version number might be, for example `1.2.3` Replace every instance of
that in examples below.


## Test Debian package build locally

Sometimes there have been changes that break the Debian package build.
Before making CI build a package for the beta cluster, make sure
builds work, if only so that it's easier to debug build problems
locally rather than via CI. You should probably do this step in a
jessie environment (chroot will do) to be compatible with CI.

```sh
$ git clean -fdx
$ git reset --hard
$ rm -f ../scap_*
$ DEBEMAIL='YOU@wikimedia.org' DEBFULLNAME='YOU' \
  gbp dch --distribution unstable --urgency low --new-version VERS-1
$ git commit -m "Bump Debian package to VERS-1" debian/changelog
```

At this point, edit `debian/changelog` to summarize changes since the
previous release. Then build the package:

```sh
$ gbp buildpackage -us -uc --git-ignore-new
```

If this fails, fix and repeat until it works. Remember to commit any
changes.


## Test changes on the beta cluster

Local tests aren't always enough, especially as Scap's automated test
suite is still rather simplistic. Thus, we test Scap on the beta
cluster to have some confidence the release doesn't break anything too
badly.

The steps for testing on the beta cluster are:

* build a Debian package for installing to the beta cluster
* use it manually to try things
* if things break, roll back

You MUST have the following access:

* ssh to **deployment-deploy01.deployment-prep.eqiad.wmflabs** (called
  "deploy01" below)
* ssh to **deployment-cumin02.deployment-prep.eqiad.wmflabs** (called
  "cumin02" below); also you must have sudo there
* login on <https://integration.wikimedia.org/ci> and the right to
  trigger jobs

To build the Debian package for the beta cluster:

* have Gerrit merge changes to Scap; this triggers a Jenkins job to
  build a Debian package for beta

  * <https://integration.wikimedia.org/ci/job/scap-beta-deb/>

* if scap-beta-deb finishes successfully, it triggers the
  beta-publish-deb job, which publishes the newly built Debian package

  * this is currently broken due to permissions problems
  * run the following commands on deploy01, if that is the case:

    * sudo aptly repo -remove-files=false -force-replace=true add jessie-deployment-prep /srv/deployment/debs/
    * sudo aptly repo -remove-files=false -force-replace=true add trusty-deployment-prep /srv/deployment/debs/
    * sudo aptly repo -remove-files=true -force-replace=true add stretch-deployment-prep /srv/deployment/debs/
    * sudo aptly publish --skip-signing update jessie-deployment-prep
    * sudo aptly publish --skip-signing update trusty-deployment-prep
    * sudo aptly publish --skip-signing update stretch-deployment-prep

    * stretch goal: fix the problems

* install new package on all beta hosts with Scap already installed

    * use `apt-get update && apt-get policy scap` on
      deployment-cumin02.deployment-prep.eqiad.wmflabs to find version
      of new package and fix it in the command line below
    * `sudo cumin 'O{project:deployment-prep}' 'command -v scap &&
      apt-get install -y --allow-downgrades scap=VERSION || echo "no
      scap"`

* run the following Jenkins job (click "Build now") or wait for it to
  run automatically (runs every ten minutes):

    * <https://integration.wikimedia.org/ci/job/beta-scap-eqiad/>
    * if it fails, see the console output to see what the problem is;
      it might look like the following, for example:

~~~
18:24:32 + /usr/bin/scap sync 'beta-scap-eqiad (build #293131)'
18:24:33 16:24:33 Unhandled error:
18:24:33 Traceback (most recent call last):
18:24:33   File "/usr/lib/python2.7/dist-packages/scap/cli.py", line 341, in run
18:24:33     exit_status = app.main(app.extra_arguments)
18:24:33   File "/usr/lib/python2.7/dist-packages/scap/main.py", line 646, in main
18:24:33     raise ValueError('Canary must be between 20 and 90 seconds')
18:24:33 ValueError: Canary must be between 20 and 90 seconds
18:24:33 16:24:33 scap failed: ValueError Canary must be between 20 and 90 seconds (duration: 00m 00s)
~~~

    * in this case, it's a bug in scap; to revert back to the
      previous, working version, run the cumin command above with the
      previous version number for the package

* manually run a deployment from deployment-deploy01

    * cd /srv/deployment/integration/slave-scripts
    * make a dummy change to README and commit it
    * run: scap deploy -v 'testing scap3'
    * run: scap deploy-log
    * output from 'scap deploy' might mention a host like
      deployment-mediawiki-07.deployment-prep.eqiad.wmflabs; log in
      there and check that
      /srv/deployment/integration/slave-scripts-cache has a current
      symlinked to a refs/... directory and that
      /srv/deployment/integration/slave-scripts is a symlink to the
      same revs directory


## Tag the release in git

This is the formal part of the release procedure. These steps all
happen in a clone of the Scap git repository. The steps are done in
the **release branch** and need to be merge in master to that branch.

* Make sure the workspace is clean and has no uncommitted changes or
  unwanted files: `git status`
* Edit `scap/version.py`, set `__version__ = 'VERS'`
* Commit: `git commit -m "Bump version to VERS" scap/version.py`
* Tag version: `git tag --sign VERS`
* Tag Debian version (debian/changelog is already committed):

```sh
gbp buildpackage --git-tag-only
git push --tags origin release
```

We keep the version number in the master branch ahead of a release so
update it. Below, `NEW` is a version number later than VERS.

* Edit `scap/version.py`, set `__version__ = 'NEW-dev'`
* Bump `debian/changelog`: `DEBEMAIL='YOU@wikimedia.org'
  DEBFULLNAME='YOU' gbp dch --distribution unstable --urgency low
  --new-version NEW-1`
* Commit: `git commit -m 'Debian: Bump development version to NEW'
  debian/changelog scap/version.py `
* Push: `git push origin master`


## Ask SRE to build and install a new Debian package

File an issue in Phabricator:

* title: Deploy Scap version VERS
* tagged: scap serviceops release-engineering-team-todo

Mention the debian/VERS tag for the release. Include the
debian/changelog entry for the new version, and any other versions
since the version currently on production.

SRE can follow instructions at
<https://wikitech.wikimedia.org/wiki/Scap3#Production_Upgrade> for
building the package.
