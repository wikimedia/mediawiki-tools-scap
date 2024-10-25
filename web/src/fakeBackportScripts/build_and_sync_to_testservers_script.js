const build_and_sync_to_testservers_script = [
  {
    "type": "line",
    "line": "\u001b[32m18:27:20 Voting on 1 change(s)\u001b[0m\n",
    "gap": 0.001260519027709961
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:20 Change 7 was already merged\u001b[0m\n",
    "gap": 0.0010666847229003906
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:20 Waiting for changes to be merged. This may take some time if there are long running tests.\u001b[0m\n",
    "gap": 0.001123189926147461
  },
  {
    "type": "status",
    "status": "Waiting for changes to be merged. This may take some time if there are long running tests.",
    "gap": 0.013793230056762695
  },
  {
    "type": "line",
    "line": "18:27:20 awaiting-backport-merges:   0% (ok: 0; fail: 0; left: 1)               \n",
    "gap": 0.001384735107421875
  },
  {
    "type": "line",
    "line": "18:27:20 awaiting-backport-merges: 100% (ok: 1; fail: 0; left: 0)               \n",
    "gap": 0.005759000778198242
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:20 All changes have been merged\u001b[0m\n",
    "gap": 0.0007853507995605469
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:20 Collecting commits to deploy...\u001b[0m\n",
    "gap": 0.0008425712585449219
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:20 Fetching new changes...\u001b[0m\n",
    "gap": 0.0006256103515625
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:21 Collecting commit for Ie76942a4cc70203ffe9061e1ff1aee9a97482cbc...\u001b[0m\n",
    "gap": 0.5854892730712891
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:21 Collecting merge commit for Ie76942a4cc70203ffe9061e1ff1aee9a97482cbc if it exists...\u001b[0m\n",
    "gap": 0.21859979629516602
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:21 No merge commit found.\u001b[0m\n",
    "gap": 0.004035472869873047
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:21 Started scap prep auto\u001b[0m\n",
    "gap": 0.6250369548797607
  },
  {
    "type": "status",
    "status": "Started scap prep auto",
    "gap": 0.008733034133911133
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:21 Update http://gerrit.traindev:8080/operations/mediawiki-config (train-dev branch) in /srv/mediawiki-staging\u001b[0m\n",
    "gap": 0.0003502368927001953
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:22 http://gerrit.traindev:8080/operations/mediawiki-config checked out at commit a2b6faae462733b03afb0c2aa59540a1fb4ccab1\u001b[0m\n",
    "gap": 0.04734086990356445
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:22 Update http://gerrit.traindev:8080/mediawiki/core (wmf/1.43.0-wmf.19 branch) in /srv/mediawiki-staging/php-1.43.0-wmf.19\u001b[0m\n",
    "gap": 0.2905282974243164
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:22 http://gerrit.traindev:8080/mediawiki/core checked out at commit dc7a1d9138b4b5490fb31f3dedeb5787ae952975\u001b[0m\n",
    "gap": 0.23610162734985352
  },
  {
    "type": "line",
    "line": "Applying patch /srv/patches/1.43.0-wmf.19/core/01-T999999.patch in /srv/mediawiki-staging/php-1.43.0-wmf.19\n",
    "gap": 22.675719499588013
  },
  {
    "type": "line",
    "line": "Applying patch /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999997.patch in /srv/mediawiki-staging/php-1.43.0-wmf.19\n",
    "gap": 0.00026297569274902344
  },
  {
    "type": "line",
    "line": "Applying patch /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999998.patch in /srv/mediawiki-staging/php-1.43.0-wmf.19\n",
    "gap": 0.00021266937255859375
  },
  {
    "type": "line",
    "line": "[APPLIED] /srv/patches/1.43.0-wmf.19/core/01-T999999.patch\n",
    "gap": 0.00023865699768066406
  },
  {
    "type": "line",
    "line": "[APPLIED] /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999997.patch\n",
    "gap": 0.00021028518676757812
  },
  {
    "type": "line",
    "line": "[APPLIED] /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999998.patch\n",
    "gap": 0.0001685619354248047
  },
  {
    "type": "line",
    "line": "\u001b[0m\u001b[32m18:27:45 MediaWiki wmf/1.43.0-wmf.19 successfully checked out.\u001b[0m\n",
    "gap": 0.07678341865539551
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:45 Update http://gerrit.traindev:8080/mediawiki/core (wmf/1.43.0-wmf.20 branch) in /srv/mediawiki-staging/php-1.43.0-wmf.20\u001b[0m\n",
    "gap": 0.0027992725372314453
  },
  {
    "type": "line",
    "line": "\u001b[32m18:27:45 http://gerrit.traindev:8080/mediawiki/core checked out at commit 86aa317f7e2d68ca8290963ca9bb95f25f705144\u001b[0m\n",
    "gap": 0.2340257167816162
  },
  {
    "type": "line",
    "line": "Applying patch /srv/patches/1.43.0-wmf.20/core/01-T999999.patch in /srv/mediawiki-staging/php-1.43.0-wmf.20\n",
    "gap": 22.095717430114746
  },
  {
    "type": "line",
    "line": "Applying patch /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999997.patch in /srv/mediawiki-staging/php-1.43.0-wmf.20\n",
    "gap": 0.00024771690368652344
  },
  {
    "type": "line",
    "line": "Applying patch /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999998.patch in /srv/mediawiki-staging/php-1.43.0-wmf.20\n",
    "gap": 0.00026702880859375
  },
  {
    "type": "line",
    "line": "[APPLIED] /srv/patches/1.43.0-wmf.20/core/01-T999999.patch\n",
    "gap": 0.00025653839111328125
  },
  {
    "type": "line",
    "line": "[APPLIED] /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999997.patch\n",
    "gap": 0.0002117156982421875
  },
  {
    "type": "line",
    "line": "[APPLIED] /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999998.patch\n",
    "gap": 0.0002410411834716797
  },
  {
    "type": "line",
    "line": "\u001b[0m\u001b[32m18:28:07 MediaWiki wmf/1.43.0-wmf.20 successfully checked out.\u001b[0m\n",
    "gap": 0.0759425163269043
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:07 Finished scap prep auto (duration: 00m 45s)\u001b[0m\n",
    "gap": 0.0004208087921142578
  },
  {
    "type": "status",
    "status": "Finished scap prep auto (duration: 00m 45s)",
    "gap": 0.006723880767822266
  },
  {
    "type": "line",
    "line": "\u001b[0m\u0007           \u001b[0m\u001b[36m___\u001b[0m \u001b[0m\u001b[36m____\u001b[0m\n",
    "gap": 0.681755781173706
  },
  {
    "type": "line",
    "line": "         \u001b[0m\u001b[36m⎛   ⎛ ,----\u001b[0m\n",
    "gap": 0.00027298927307128906
  },
  {
    "type": "line",
    "line": "          \u001b[0m\u001b[36m\\  //==--'\u001b[0m\n",
    "gap": 0.00025391578674316406
  },
  {
    "type": "line",
    "line": "     \u001b[0m\u001b[1;35m_//|,.·\u001b[0m\u001b[36m//==--'\u001b[0m    \u001b[0m\u001b[37m______\u001b[0m\u001b[32m____\u001b[0m\u001b[37m_\u001b[0m\u001b[32m____\u001b[0m\u001b[37m___\u001b[0m\u001b[32m____\u001b[0m\u001b[37m__\u001b[0m\u001b[32m____\u001b[0m\n",
    "gap": 0.00025010108947753906
  },
  {
    "type": "line",
    "line": "    \u001b[0m\u001b[1;35m_\u001b[0m\u001b[33mOO≣=-\u001b[0m\u001b[1;35m  \u001b[0m\u001b[36m︶\u001b[0m\u001b[1;35m \u001b[0mᴹw\u001b[0m\u001b[1;35m ⎞_§\u001b[0m \u001b[0m\u001b[37m______\u001b[0m\u001b[32m  ___\\ ___\\ ,\\__ \\/ __ \\\u001b[0m\n",
    "gap": 0.00017380714416503906
  },
  {
    "type": "line",
    "line": "   \u001b[0m\u001b[1;35m(\u001b[0m\u001b[1;35m∞\u001b[0m\u001b[1;35m)\u001b[0m\u001b[1;35m_,\u001b[0m\u001b[1;35m )  (     |\u001b[0m  \u001b[0m\u001b[37m______\u001b[0m\u001b[32m/__  \\/ /__ / /_/ / /_/ /\u001b[0m\n",
    "gap": 0.00021147727966308594
  },
  {
    "type": "line",
    "line": "     \u001b[0m\u001b[1;35m¨--¨|| |- (  /\u001b[0m \u001b[0m\u001b[37m______\u001b[0m\u001b[32m\\____/ \\___/ \\__^_/  .__/\u001b[0m\n",
    "gap": 0.00018644332885742188
  },
  {
    "type": "line",
    "line": "         \u001b[0m\u001b[34m««\u001b[0m\u001b[1;35m_/\u001b[0m  \u001b[0m\u001b[34m«\u001b[0m\u001b[1;35m_/\u001b[0m \u001b[0m\u001b[34mjgs/bd808\u001b[0m                \u001b[0m\u001b[32m/_/\u001b[0m\n",
    "gap": 0.00020766258239746094
  },
  {
    "type": "line",
    "line": "\n",
    "gap": 0.0002200603485107422
  },
  {
    "type": "status",
    "status": "Started scap sync-world: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]]",
    "gap": 0.00820159912109375
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:08 Started scap sync-world: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]]\u001b[0m\n",
    "gap": 0.0006375312805175781
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:08 Started cache_git_info\u001b[0m\n",
    "gap": 0.20441389083862305
  },
  {
    "type": "status",
    "status": "Started cache_git_info",
    "gap": 0.00780177116394043
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:09 Finished cache_git_info (duration: 00m 01s)\u001b[0m\n",
    "gap": 1.347463846206665
  },
  {
    "type": "status",
    "status": "Finished cache_git_info (duration: 00m 01s)",
    "gap": 0.007032155990600586
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:09 Started l10n-update\u001b[0m\n",
    "gap": 0.00033354759216308594
  },
  {
    "type": "status",
    "status": "Started l10n-update",
    "gap": 0.0065460205078125
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:11 Updating ExtensionMessages-1.43.0-wmf.19.php\u001b[0m\n",
    "gap": 1.0394713878631592
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:15 Updating LocalisationCache for 1.43.0-wmf.19 using 10 thread(s)\u001b[0m\n",
    "gap": 4.152384281158447
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:15 Running rebuildLocalisationCache.php\u001b[0m\n",
    "gap": 0.00042891502380371094
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:16 0 languages rebuilt out of 1\u001b[0m\n",
    "gap": 1.0000550746917725
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:16 Use --force to rebuild the caches which are still fresh.\u001b[0m\n",
    "gap": 0.000331878662109375
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:17 Generating JSON versions and md5 files (as www-data)\u001b[0m\n",
    "gap": 1.2220566272735596
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:19 Updating ExtensionMessages-1.43.0-wmf.20.php\u001b[0m\n",
    "gap": 1.7966334819793701
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:23 Updating LocalisationCache for 1.43.0-wmf.20 using 10 thread(s)\u001b[0m\n",
    "gap": 4.338477611541748
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:23 Running rebuildLocalisationCache.php\u001b[0m\n",
    "gap": 0.00033020973205566406
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:24 0 languages rebuilt out of 1\u001b[0m\n",
    "gap": 1.0521116256713867
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:24 Use --force to rebuild the caches which are still fresh.\u001b[0m\n",
    "gap": 0.000396728515625
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:25 Generating JSON versions and md5 files (as www-data)\u001b[0m\n",
    "gap": 1.232424259185791
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:26 Finished l10n-update (duration: 00m 16s)\u001b[0m\n",
    "gap": 0.7347652912139893
  },
  {
    "type": "status",
    "status": "Finished l10n-update (duration: 00m 16s)",
    "gap": 0.006293058395385742
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:26 Checking for new runtime errors locally\u001b[0m\n",
    "gap": 0.00042724609375
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:29 Started build-and-push-container-images\u001b[0m\n",
    "gap": 2.4383292198181152
  },
  {
    "type": "status",
    "status": "Started build-and-push-container-images",
    "gap": 0.005961418151855469
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:29 K8s images build/push output redirected to /home/debian/scap-image-build-and-push-log\u001b[0m\n",
    "gap": 0.0003910064697265625
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:42 Finished build-and-push-container-images (duration: 00m 13s)\u001b[0m\n",
    "gap": 13.05268931388855
  },
  {
    "type": "status",
    "status": "Finished build-and-push-container-images (duration: 00m 13s)",
    "gap": 0.009584665298461914
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:42 Started sync-masters\u001b[0m\n",
    "gap": 0.08082365989685059
  },
  {
    "type": "status",
    "status": "Started sync-masters",
    "gap": 0.006597757339477539
  },
  {
    "type": "line",
    "line": "18:28:42 sync-masters:   0% (ok: 0; fail: 0; left: 1)                           \n",
    "gap": 0.0005276203155517578
  },
  {
    "type": "line",
    "line": "18:28:46 sync-masters: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)             \n",
    "gap": 4.820453643798828
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:46 Finished sync-masters (duration: 00m 04s)\u001b[0m\n",
    "gap": 0.0002613067626953125
  },
  {
    "type": "status",
    "status": "Finished sync-masters (duration: 00m 04s)",
    "gap": 0.023672819137573242
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:47 Started sync-testservers-k8s\u001b[0m\n",
    "gap": 0.03644299507141113
  },
  {
    "type": "status",
    "status": "Started sync-testservers-k8s",
    "gap": 0.005957603454589844
  },
  {
    "type": "status",
    "status": "Started Running helmfile -e traindev --selector name=pinkunicorn write-values --output-file-template /tmp/tmpcpnc1tcf in /srv/deployment-charts/helmfile.d/services/mw-debug",
    "gap": 0.0055768489837646484
  },
  {
    "type": "status",
    "status": "Finished Running helmfile -e traindev --selector name=pinkunicorn write-values --output-file-template /tmp/tmpcpnc1tcf in /srv/deployment-charts/helmfile.d/services/mw-debug (duration: 00m 00s)",
    "gap": 0.06012272834777832
  },
  {
    "type": "line",
    "line": "18:28:47 K8s deployment progress:   0% (ok: 0; fail: 0; left: 1)                \n",
    "gap": 0.08627533912658691
  },
  {
    "type": "status",
    "status": "Started Running helmfile -e traindev --selector name=pinkunicorn apply in /srv/deployment-charts/helmfile.d/services/mw-debug",
    "gap": 0.22066378593444824
  },
  {
    "type": "status",
    "status": "Finished Running helmfile -e traindev --selector name=pinkunicorn apply in /srv/deployment-charts/helmfile.d/services/mw-debug (duration: 00m 06s)",
    "gap": 6.973317861557007
  },
  {
    "type": "line",
    "line": "18:28:54 K8s deployment progress: 100% (ok: 1; fail: 0; left: 0)                \n",
    "gap": 0.03833436965942383
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:54 Finished sync-testservers-k8s (duration: 00m 07s)\u001b[0m\n",
    "gap": 0.0003132820129394531
  },
  {
    "type": "status",
    "status": "Finished sync-testservers-k8s (duration: 00m 07s)",
    "gap": 0.005827188491821289
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:54 Started sync-testservers\u001b[0m\n",
    "gap": 0.0003571510314941406
  },
  {
    "type": "status",
    "status": "Started sync-testservers",
    "gap": 0.005716562271118164
  },
  {
    "type": "line",
    "line": "18:28:54 sync-testservers:   0% (ok: 0; fail: 0; left: 1)                       \n",
    "gap": 0.0003478527069091797
  },
  {
    "type": "line",
    "line": "18:28:58 sync-testservers: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)         \n",
    "gap": 3.8904640674591064
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:58 Per-host sync duration: average 3.9s, median 3.9s\u001b[0m\n",
    "gap": 0.00028014183044433594
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:58 rsync transfer: average 361,996 bytes/host, total 361,996 bytes\u001b[0m\n",
    "gap": 0.0013318061828613281
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:58 Started scap-cdb-rebuild\u001b[0m\n",
    "gap": 0.0005278587341308594
  },
  {
    "type": "status",
    "status": "Started scap-cdb-rebuild",
    "gap": 0.006343841552734375
  },
  {
    "type": "line",
    "line": "18:28:58 scap-cdb-rebuild:   0% (ok: 0; fail: 0; left: 1)                       \n",
    "gap": 0.0003151893615722656
  },
  {
    "type": "line",
    "line": "18:28:59 scap-cdb-rebuild: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)         \n",
    "gap": 1.0280497074127197
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:59 Finished scap-cdb-rebuild (duration: 00m 01s)\u001b[0m\n",
    "gap": 0.0002918243408203125
  },
  {
    "type": "status",
    "status": "Finished scap-cdb-rebuild (duration: 00m 01s)",
    "gap": 0.006124258041381836
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:59 Started sync_wikiversions\u001b[0m\n",
    "gap": 0.00031280517578125
  },
  {
    "type": "status",
    "status": "Started sync_wikiversions",
    "gap": 0.005811929702758789
  },
  {
    "type": "line",
    "line": "18:28:59 sync_wikiversions:   0% (ok: 0; fail: 0; left: 1)                      \n",
    "gap": 0.0003345012664794922
  },
  {
    "type": "line",
    "line": "18:28:59 sync_wikiversions: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)        \n",
    "gap": 0.37673401832580566
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:59 Finished sync_wikiversions (duration: 00m 00s)\u001b[0m\n",
    "gap": 0.001041412353515625
  },
  {
    "type": "status",
    "status": "Finished sync_wikiversions (duration: 00m 00s)",
    "gap": 0.011378288269042969
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:59 Started php-fpm-restarts\u001b[0m\n",
    "gap": 0.0015377998352050781
  },
  {
    "type": "status",
    "status": "Started php-fpm-restarts",
    "gap": 0.010534524917602539
  },
  {
    "type": "line",
    "line": "\u001b[32m18:28:59 Running '/usr/local/sbin/check-and-restart-php php7.4-fpm 9223372036854775807' on 1 host(s)\u001b[0m\n",
    "gap": 0.0011799335479736328
  },
  {
    "type": "line",
    "line": "18:28:59 php-fpm-restart:   0% (ok: 0; fail: 0; left: 0)                        \n",
    "gap": 0.0009481906890869141
  },
  {
    "type": "line",
    "line": "18:29:00 php-fpm-restart: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)          \n",
    "gap": 0.26976823806762695
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:00 Finished php-fpm-restarts (duration: 00m 00s)\u001b[0m\n",
    "gap": 0.00028324127197265625
  },
  {
    "type": "status",
    "status": "Finished php-fpm-restarts (duration: 00m 00s)",
    "gap": 0.005459308624267578
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:00 Finished sync-testservers (duration: 00m 05s)\u001b[0m\n",
    "gap": 0.0004017353057861328
  },
  {
    "type": "status",
    "status": "Finished sync-testservers (duration: 00m 05s)",
    "gap": 0.0053865909576416016
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:00 Started check-testservers\u001b[0m\n",
    "gap": 0.00039505958557128906
  },
  {
    "type": "status",
    "status": "Started check-testservers",
    "gap": 0.005415439605712891
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:00 Executing check 'check_testservers_baremetal'\u001b[0m\n",
    "gap": 0.00040221214294433594
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:00 Executing check 'check_testservers_k8s'\u001b[0m\n",
    "gap": 0.0002601146697998047
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:02 Finished check-testservers (duration: 00m 02s)\u001b[0m\n",
    "gap": 2.402876138687134
  },
  {
    "type": "status",
    "status": "Finished check-testservers (duration: 00m 02s)",
    "gap": 0.006013154983520508
  },
  {
    "type": "status",
    "status": "trainbranchbot, debian: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]] synced to the testservers (https://wikitech.wikimedia.org/wiki/Mwdebug)",
    "gap": 0.005757570266723633
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:02 trainbranchbot, debian: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]] synced to the testservers (https://wikitech.wikimedia.org/wiki/Mwdebug)\u001b[0m\n",
    "gap": 0.0003821849822998047
  },
  {
    "type": "line",
    "line": "Changes synced to the testservers. (see https://wikitech.wikimedia.org/wiki/Mwdebug)\nPlease do any necessary checks before continuing.\nContinue with sync? [y/N]: ",
    "gap": 0.000335693359375
  },
  {
    "type": "interaction",
    "subtype": "choices",
    "prompt": "Changes synced to the testservers. (see https://wikitech.wikimedia.org/wiki/Mwdebug)\nPlease do any necessary checks before continuing.\nContinue with sync?",
    "choices": {
      "Yes": "y",
      "No": "n"
    },
    "default": "n",
    "gap": 3.337860107421875e-06
  },
]
export default build_and_sync_to_testservers_script
