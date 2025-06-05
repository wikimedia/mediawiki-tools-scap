# Release procedure for Scap

This is the release procedure for the Scap software. It is meant
to be followed by a member of the Release Engineering team.

This document assumes that the codebase has already undergone manual
and automated testing.

## Production Release

Run `release-scripts/perform-release`.  It will:

* Ensure that your checkout is up-to-date and clean.
* Prompt you for for the new version number (with a reasonable default).
* Update scap/version.py and changelog and create a commit with those changes.
* Push the commit to Gitlab, approve it, and wait for it to merge.
* Tag and push the commit.
* Tell you how to deploy the new release.

That's it!

### Production deployment

* Make sure you select a time window where no deployments are happening
* On the main deployment server (**deployment.eqiad.wmnet** at the time of writing), run
`scap install-world`. This will select the latest available version tag for installation and
prompt you for confirmation.
* If you need to roll back, you can specify a particular version tag with e.g.
`scap install-world --version 4.42.0`

## Beta Release

(For the following steps to work you must have ssh access to and sudo privileges on 
**deployment-cumin-3.deployment-prep.eqiad.wmflabs** and **deployment-deploy03.deployment-prep.
eqiad1.wikimedia.cloud**)

1. Run `release-scripts/scaps-installed-in-beta`.  It will print a
list of the versions of Scap that are installed on beta hosts.  Save
this information in case a rollback is needed.  Ideally there will
be one version installed on all hosts. If there is more than one scap version
save the most installed scap version for rollback.

Example output:

```
$ ./release-scripts/scaps-installed-in-beta
11 hosts will be targeted:
deployment-deploy04.deployment-prep.eqiad1.wikimedia.cloud,deployment-echostore02.deployment-prep.eqiad1.wikimedia.cloud,deployment-jobrunner05.deployment-prep.eqiad1.wikimedia.cloud,deployment-mediawiki[13-14].deployment-prep.eqiad1.wikimedia.cloud,deployment-mwmaint03.deployment-prep.eqiad1.wikimedia.cloud,deployment-restbase05.deployment-prep.eqiad1.wikimedia.cloud,deployment-sessionstore06.deployment-prep.eqiad1.wikimedia.cloud,deployment-snapshot05.deployment-prep.eqiad1.wikimedia.cloud,deployment-webperf[21-22].deployment-prep.eqiad1.wikimedia.cloud
FORCE mode enabled, continuing without confirmation
100.0% (11/11) success ratio (>= 100.0% threshold) for command: 'sudo -u scap scap version'.
100.0% (11/11) success ratio (>= 100.0% threshold) of nodes successfully executed all commands.
4.182.0
```

2. Run `release-scripts/update-scap-in-beta`. This script will list the latest
tagged releases of Scap and asked you to pick which version to deploy. Once you've
selected a version to deploy the script will install that version of Scap on all
beta scap hosts.

Example output:

```
No scap version specified!

Pick a scap version to deploy:
     1  4.182.0
     2  4.181.0
     3  4.180.0
     4  4.179.1
     5  4.179.0
Enter the number of the version above (empty or 0 to cancel) → 1

    Selected version '4.182.0'

Use this log message in #wikimedia-releng IRC:

    !log Upgrading scap to 4.182.0 in beta cluster

Press enter to deploy (Ctrl-C to cancel)

<...lots of output while scap installs...>

17:06:48 Installation of scap version "4.182.0" completed for 11 hosts
```

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
