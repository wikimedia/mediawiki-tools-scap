# Release procedure for Scap

This is a detailed release procedure for the Scap software. It's meant
to be followed by a member of the Release Engineering team.

This document assumes that the codebase has already undergone manual
and automated testing.

## Pick version number

Pick a new version number, called VERS in these instructions.  A
version number might be, for example `1.2.3`.  Replace every instance
of `VERS` in examples below with the version number. If the example
says `VERS-1` that would result in `1.2.3-1`.

* Edit `scap/version.py` and set the new version.

## Update debian changelog

Install the `devscripts` package if not already installed (for the `dch` command).

* Run `dch -v VERS-1`.  This will spawn an editor with a copy of
  debian/changelog with a new changelog entry.  Populate the entry
  with a list of changes from the prior release.  `debian/changelog`
  will be updated when you exit the editor.

* Run `DEBEMAIL=youraccount@wikimedia.org dch -r --distribution
  unstable`.  This will spawn an editor.  Just save and exit.

* Commit the changed files using git, using commit message summary
  `Release VERS-1`.

## Perform a local deb build (optional)

* Run `./build-deb-in-docker /tmp/scap-build` to verify that the deb
  file can be created.  This step requires Docker.

## Push changes to Gerrit for review

```sh
$ git push origin HEAD:refs/for/master
```

Someone will need to +2 the change to merge it.

## Test deb package on the beta cluster

You MUST have the following access:

* ssh to **deployment-deploy01.deployment-prep.eqiad.wmflabs**
* ssh to **deployment-cumin.deployment-prep.eqiad.wmflabs**; also you must have sudo there
* login on <https://integration.wikimedia.org/ci> and the right to
  trigger jobs

Steps:

* When the version-bumping commit has been merged, the
  `beta-build-scap-deb` Jenkins job will be triggered.  Wait for the job
  to finish.  When it finishes it will trigger the `beta-publish-deb`
  job.  That job finishes within a few seconds.  Everything is now set
  up for deploying the new deb to beta.

* Install new package on all beta hosts with Scap already installed

    * `ssh deployment-cumin.deployment-prep.eqiad.wmflabs`
    * run: `sudo cumin --no-progress --force 'O{project:deployment-prep}' 'if command -v scap >/dev/null; then dpkg -l scap; fi'` to determine the version(s) of scap that are currently deployed in beta cluster.  Ideally there will just be one version but there have been cases where not all hosts carried the same version.  In that case you must select one of the versions to use as the "old" version in case you need to roll back.
    * run: `sudo apt-get update`
    * run: `VERSION=$(sudo LC_ALL=C apt-cache policy scap | grep 'Candidate:' | awk '{print $2}')` to determine the latest version of scap that has been published by the beta-publish-deb job.
    * run: `echo $VERSION` to verify that the selected version looks ok.  It should be something like `4.0.0-1+0~20210914200047.52~1.gbp65cf53`.
    * on the #wikimedia-operations IRC channel, say you're testing
      Scap: `!log testing upcoming Scap release on beta`
    * run: `sudo cumin -p90 'O{project:deployment-prep}' "if command -v scap; then apt-get update && apt-get install -y --allow-downgrades scap=$VERSION; else echo no scap; fi"`
    * Unfortunately there are often a small number of hosts in the
      beta cluster that are broken (e.g., full filesystem) and might
      fail to install the new scap deb.  You will have to wade through
      the cumin output to figure out what went wrong.

* Run the following Jenkins job (click "Build now") or wait for it to
  run automatically (runs every ten minutes):

    * <https://integration.wikimedia.org/ci/job/beta-scap-sync-world/>
    * If it fails (and it wasn't failing before), it's probably a new bug in scap; to revert back to the
      previous, working version, run the cumin command above with the
      previous version number for the package

* Manually run a deployment from deployment-deploy01

    * `ssh deployment-deploy01.deployment-prep.eqiad.wmflabs`
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

## Let the release marinate in beta cluster for a while

Let the new scap deb operate for some amount of time in beta cluster.
How long is up to you.  

## Tag the release in git

This is the formal part of the release procedure. These steps all
happen in a clone of the Scap git repository.

* Make sure the workspace is clean and has no uncommitted changes or
  unwanted files: `git status`
* Use `git log` to locate the version-changing commit that you merged earlier
  in this process.  
* Tag version: `git tag --sign -m "Release VERS" VERS <commit>`
* Push the tag to Gerrit: `git push --tags origin VERS`

## Ask SRE to build and install a new Debian package

File a task in Phabricator:

* title: Deploy Scap version VERS
* description:
```
Dear SRE team,
Please build and deploy the version 4.0.0 of scap using the instructions in https://wikitech.wikimedia.org/wiki/Scap/Release
<other details that may be relevant>
Thanks in advance!
```
* tagged: scap serviceops release-engineering-team
