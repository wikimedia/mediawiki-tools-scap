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

To do this in Docker:

~~~
$ ~/go/bin/blubber .pipeline/blubber.yaml test > Dockerfile
$ docker build . # note build id
$ docker run --rm 5fea442ac172 # Fix the image id based on above
~~~


## Pick version number

Pick a new version number, called VERS in these instructions; a
version number might be, for example `1.2.3` Replace every instance of
`VERS` in examples below with the version number. If the example says
`VERS-1` that would result in `1.2.3-1`.


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
```

At this point, edit `debian/changelog` to summarize changes since the
previous release. Then build the package.

```sh
$ gbp buildpackage -us -uc --git-ignore-new
```

If this fails, fix whatever is broken, and repeat until it works.
Finally, commit any changes.

```sh
$ git commit -m "Bump Debian package to VERS-1" debian/changelog
$ git status
```

Push the changes directly to Gerrit. This is a rare exception to the
rule that all changes should be reviewed, and hopefully Scap will be
less of a SPOF in the future so that this exception can cease to
exist.

```sh
$ git push origin HEAD:master
```


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
* ssh to **deployment-cumin.deployment-prep.eqiad.wmflabs**; also you must have sudo there
* login on <https://integration.wikimedia.org/ci> and the right to
  trigger jobs

To build the Debian package for the beta cluster:

* go to the scap-beta-deb Jenkins project
  (<https://integration.wikimedia.org/ci/job/scap-beta-deb/>), and
  trigger it manually by clicking "Build with Parameters", entering
  the following parameters:

  * ZUUL_URL: https://gerrit.wikimedia.org/r
  * ZUUL_PROJECT: mediawiki/tools/scap.git
  * ZUUL_REF: master
  * ZUUL_VOTING: 0

* if scap-beta-deb finishes successfully, it triggers the
  beta-publish-deb job, which publishes the newly built Debian package

* install new package on all beta hosts with Scap already installed

    * on the #wikimedia-operations IRC channel, say you're testing
      Scap: `!log testing upcoming Scap release on beta`
    * ssh deployment-cumin.deployment-prep.eqiad.wmflabs
    * run: `sudo LC_ALL=C apt-get update && apt-cache policy scap`
    * find version of new package and fix it in the command line below
    * run: `sudo cumin 'O{project:deployment-prep}' 'if command -v scap; then apt-get update && apt-get install -y --allow-downgrades scap=VERSION; else echo "no scap"; fi'

* run the following Jenkins job (click "Build now") or wait for it to
  run automatically (runs every ten minutes):

    * <https://integration.wikimedia.org/ci/job/beta-scap-sync-world/>
    * if it fails (and it wasn't failing before), it's probably a new bug in scap; to revert back to the
      previous, working version, run the cumin command above with the
      previous version number for the package

* manually run a deployment from deployment-deploy01

    * `cd /srv/deployment/integration/slave-scripts`
    * make a dummy change to README and commit it
    * run: `scap version && dpkg -l scap`
    * check the version numbers are correct
    * run: `scap deploy -v 'testing scap3'`
    * run: `scap deploy-log`
    * kill that with control-C if it looks good
    * output from 'scap deploy' might mention a host like
      deployment-mediawiki-07.deployment-prep.eqiad.wmflabs; log in
      there and check that
      /srv/deployment/integration/slave-scripts-cache has a current
      symlinked to a refs/... directory and that
      /srv/deployment/integration/slave-scripts is a symlink to the
      same revs directory: 
      `ls -l /srv/deployment/integration/slave-scripts-cache /srv/deployment/integration/slave-scripts`


## Tag the release in git

This is the formal part of the release procedure. These steps all
happen in a clone of the Scap git repository. The steps are done in
the **release branch** and need to be merge in master to that branch.

* Make sure the workspace is clean and has no uncommitted changes or
  unwanted files: `git status`
* run: `git checkout release`
* run: `git merge master`
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

* Run: `git checkout master`
* Edit `scap/version.py`, set `__version__ = 'NEW-dev'`
* Bump `debian/changelog`: `DEBEMAIL='YOU@wikimedia.org'
  DEBFULLNAME='YOU' gbp dch --distribution unstable --urgency low
  --new-version NEW-1`
* Commit: `git commit -m 'Debian: Bump development version to NEW'
  debian/changelog scap/version.py `
* Push: `git push origin master`


## Ask SRE to build and install a new Debian package

File an issue in Phabricator:

* title: Deploy Scap version VERS-1
* description: The release branch of scap has tags for VERS-1. Please
  build the package and deploy it. Instructions at
  https://wikitech.wikimedia.org/wiki/Scap3#Production_Upgrade

  Thank you.
* tagged: scap serviceops release-engineering-team-todo


Mention the debian/VERS tag for the release. Include the
debian/changelog entry for the new version, and any other versions
since the version currently on production.

SRE can follow instructions at
<https://wikitech.wikimedia.org/wiki/Scap3#Production_Upgrade> for
building the package.
