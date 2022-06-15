# Release procedure for Scap

This is a detailed release procedure for the Scap software. It's meant
to be followed by a member of the Release Engineering team.

This document assumes that the codebase has already undergone manual
and automated testing.

## Test latest Scap code using the Beta Cluster

NOTE: _Production is currently using scap self-installation (scap-over-scap) to update scap versions,
but the Debian package is still used in the beta cluster. The plan is to eventually phase out the
Debian package and use scap-o-scap in beta too._

Before you make a new release you should verify that the latest Scap
code works in beta cluster.

Whenever a Scap commit is merged, the
[beta-build-scap-deb](https://integration.wikimedia.org/ci/view/Beta/job/beta-build-scap-deb/)
Jenkins job builds a new deb file and triggers the
[beta-publish-deb](https://integration.wikimedia.org/ci/view/Beta/job/beta-publish-deb/)
to publish it.  These debs are not automatically installed on beta
cluster hosts.  To install the latest deb on beta hosts, perform the following steps:

(For these steps to work you must have ssh access to and sudo
privileges on **deployment-cumin.deployment-prep.eqiad.wmflabs**)

1. Run `release-scripts/scaps-installed-in-beta`.  It will print a
list of the versions of Scap that are installed on beta hosts.  Save
this information in case a rollback is needed.  Ideally there will
just be one version reported but sometimes (read: frequently)
situations arise on beta cluster that result in mismatches.

1. Run `release-scripts/update-scap-in-beta`.  This will retrieve the
latest available version of Scap, prompt you for confirmation, and (if
you answer `y` or `yes`) install it on beta hosts that already have
Scap installed.

### Test the Scap deployment

* Run the
  [beta-scap-sync-world Jenkins job](https://integration.wikimedia.org/ci/job/beta-scap-sync-world/)
  (click "Build now") or wait for it to run automatically
  (runs every ten minutes)

* If it fails (and it wasn't failing before), it's probably a new bug
  in scap.  Revert back to the previous working version by running
  `release-scripts/update-scap-in-beta OLDVERSION`, replacing
  OLDVERSION with the version saved in earlier steps.

* Manually run a deployment from deployment-deploy03 *(FIXME: This needs to be replaced by pre-release automated testing)*

    * `ssh deployment-deploy03.deployment-prep.eqiad1.wikimedia.cloud`
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

### Let the release marinate in beta cluster for a while

Let the new scap deb operate for some amount of time in beta cluster.
How long is up to you.  When you are satisfied, move on with the remaining steps.

## Pick version number

Pick a new version number, called $VERS in these instructions.  A
version number might be, for example `1.2.3`.  Replace every instance
of `$VERS` in examples below with the version number.  Run
```sh
VERS=1.2.3 # Replace with new version
```
in your shell to make the following commands simpler.

## Update scap/version.py and debian/changelog

Run `release-scripts/prepare-scap-release $VERS`.  This will
automatically update and commit `scap/version.py` and
`debian/changelog`, prompting you for confirmation along the way.

At this point you are welcome to further modify `debian/changelog` and
update the commit.

## Perform a local deb build (optional)

* Run `./build-deb-in-docker /tmp/scap-build` to verify that the deb
  file can be created.  This step requires Docker.

## Push changes to Gerrit for review

```sh
$ git push origin HEAD:refs/for/master
```

Someone will need to +2 the change to merge it.

After the merge, run `git pull --rebase`

## Tag the release in git

This is the formal part of the release procedure. These steps all
happen in a clone of the Scap git repository.

* Make sure the workspace is clean and has no uncommitted changes or
  unwanted files: `git status`
* Use `git log` to locate the version-changing commit that you merged earlier
  in this process.
* Tag version: `git tag --sign -m "Release $VERS" $VERS HEAD`
* Push the tag to Gerrit: `git push --tags origin $VERS`

## Deploy the new version in production

* Make sure you select a time window where no deployments are happening
* On the main deployment server (**deploy1002.eqiad.wmnet** at the time of writing), run
`scap install-world`. This will select the latest available version tag for installation and
prompt you for confirmation.
* If you need to roll back, you can specify a particular version tag with e.g.
`scap install-world --version 4.9.3`
* Profit!
