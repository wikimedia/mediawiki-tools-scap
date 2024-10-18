const fakeJobOutput = [
    {
        "timestamp": 1729791035.5430288,
        "line": "\u001b[32m17:30:35 Checking whether requested changes are in a branch deployed to production and their dependencies valid...\u001b[0m\n"
    },
    {
        "timestamp": 1729791035.7576773,
        "line": "\u001b[32m17:30:35 Change '7' validated for backport\u001b[0m\n"
    },
    {
        "timestamp": 1729791035.7584708,
        "line": "The following changes are scheduled for backport:\n┌───┬─────────────────────────────┬───────────┬─────────────────────────┐\n│ # │           Project           │   Branch  │ Subject                 │\n├───┼─────────────────────────────┼───────────┼─────────────────────────┤\n│ 7 │ operations/mediawiki-config │ train-dev │ group1 to 1.43.0-wmf.20 │\n└───┴─────────────────────────────┴───────────┴─────────────────────────┘\nBackport the changes? [y/N]: "
    },
    {
        "timestamp": 1729791039.8563013,
        "line": "\u001b[32m17:30:39 Voting on 1 change(s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791039.8573868,
        "line": "\u001b[32m17:30:39 Change 7 was already merged\u001b[0m\n"
    },
    {
        "timestamp": 1729791039.858447,
        "line": "\u001b[32m17:30:39 Waiting for changes to be merged. This may take some time if there are long running tests.\u001b[0m\n"
    },
    {
        "timestamp": 1729791039.8740413,
        "line": "17:30:39 awaiting-backport-merges:   0% (ok: 0; fail: 0; left: 1)               \n"
    },
    {
        "timestamp": 1729791039.8866174,
        "line": "17:30:39 awaiting-backport-merges: 100% (ok: 1; fail: 0; left: 0)               \n"
    },
    {
        "timestamp": 1729791039.8873088,
        "line": "\u001b[32m17:30:39 All changes have been merged\u001b[0m\n"
    },
    {
        "timestamp": 1729791039.8880413,
        "line": "\u001b[32m17:30:39 Collecting commits to deploy...\u001b[0m\n"
    },
    {
        "timestamp": 1729791039.888685,
        "line": "\u001b[32m17:30:39 Fetching new changes...\u001b[0m\n"
    },
    {
        "timestamp": 1729791040.0294192,
        "line": "\u001b[32m17:30:40 Collecting commit for Ie76942a4cc70203ffe9061e1ff1aee9a97482cbc...\u001b[0m\n"
    },
    {
        "timestamp": 1729791040.229221,
        "line": "\u001b[32m17:30:40 Collecting merge commit for Ie76942a4cc70203ffe9061e1ff1aee9a97482cbc if it exists...\u001b[0m\n"
    },
    {
        "timestamp": 1729791040.2330613,
        "line": "\u001b[32m17:30:40 No merge commit found.\u001b[0m\n"
    },
    {
        "timestamp": 1729791040.8474085,
        "line": "\u001b[32m17:30:40 Started scap prep auto\u001b[0m\n"
    },
    {
        "timestamp": 1729791040.855461,
        "line": "\u001b[32m17:30:40 Update http://gerrit.traindev:8080/operations/mediawiki-config (train-dev branch) in /srv/mediawiki-staging\u001b[0m\n"
    },
    {
        "timestamp": 1729791040.9048533,
        "line": "\u001b[32m17:30:40 http://gerrit.traindev:8080/operations/mediawiki-config checked out at commit 6f5171c520291875dab2de76099f3940ebed83bb\u001b[0m\n"
    },
    {
        "timestamp": 1729791041.2525077,
        "line": "\u001b[32m17:30:41 Update http://gerrit.traindev:8080/mediawiki/core (wmf/1.43.0-wmf.19 branch) in /srv/mediawiki-staging/php-1.43.0-wmf.19\u001b[0m\n"
    },
    {
        "timestamp": 1729791041.4299302,
        "line": "\u001b[32m17:30:41 http://gerrit.traindev:8080/mediawiki/core checked out at commit dc7a1d9138b4b5490fb31f3dedeb5787ae952975\u001b[0m\n"
    },
    {
        "timestamp": 1729791059.2163491,
        "line": "Applying patch /srv/patches/1.43.0-wmf.19/core/01-T999999.patch in /srv/mediawiki-staging/php-1.43.0-wmf.19\n"
    },
    {
        "timestamp": 1729791059.2166271,
        "line": "Applying patch /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999997.patch in /srv/mediawiki-staging/php-1.43.0-wmf.19\n"
    },
    {
        "timestamp": 1729791059.2168367,
        "line": "Applying patch /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999998.patch in /srv/mediawiki-staging/php-1.43.0-wmf.19\n"
    },
    {
        "timestamp": 1729791059.2170632,
        "line": "[APPLIED] /srv/patches/1.43.0-wmf.19/core/01-T999999.patch\n"
    },
    {
        "timestamp": 1729791059.217321,
        "line": "[APPLIED] /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999997.patch\n"
    },
    {
        "timestamp": 1729791059.2175326,
        "line": "[APPLIED] /srv/patches/1.43.0-wmf.19/extensions/GrowthExperiments/01-T999998.patch\n"
    },
    {
        "timestamp": 1729791059.306328,
        "line": "\u001b[0m\u001b[32m17:30:59 MediaWiki wmf/1.43.0-wmf.19 successfully checked out.\u001b[0m\n"
    },
    {
        "timestamp": 1729791059.3087454,
        "line": "\u001b[32m17:30:59 Update http://gerrit.traindev:8080/mediawiki/core (wmf/1.43.0-wmf.20 branch) in /srv/mediawiki-staging/php-1.43.0-wmf.20\u001b[0m\n"
    },
    {
        "timestamp": 1729791059.4510505,
        "line": "\u001b[32m17:30:59 http://gerrit.traindev:8080/mediawiki/core checked out at commit 86aa317f7e2d68ca8290963ca9bb95f25f705144\u001b[0m\n"
    },
    {
        "timestamp": 1729791078.3189747,
        "line": "Applying patch /srv/patches/1.43.0-wmf.20/core/01-T999999.patch in /srv/mediawiki-staging/php-1.43.0-wmf.20\n"
    },
    {
        "timestamp": 1729791078.3194737,
        "line": "Applying patch /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999997.patch in /srv/mediawiki-staging/php-1.43.0-wmf.20\n"
    },
    {
        "timestamp": 1729791078.3200014,
        "line": "Applying patch /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999998.patch in /srv/mediawiki-staging/php-1.43.0-wmf.20\n"
    },
    {
        "timestamp": 1729791078.3205082,
        "line": "[APPLIED] /srv/patches/1.43.0-wmf.20/core/01-T999999.patch\n"
    },
    {
        "timestamp": 1729791078.3210201,
        "line": "[APPLIED] /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999997.patch\n"
    },
    {
        "timestamp": 1729791078.3215077,
        "line": "[APPLIED] /srv/patches/1.43.0-wmf.20/extensions/GrowthExperiments/01-T999998.patch\n"
    },
    {
        "timestamp": 1729791078.4100802,
        "line": "\u001b[0m\u001b[32m17:31:18 MediaWiki wmf/1.43.0-wmf.20 successfully checked out.\u001b[0m\n"
    },
    {
        "timestamp": 1729791078.410557,
        "line": "\u001b[32m17:31:18 Finished scap prep auto (duration: 00m 37s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.134235,
        "line": "\u001b[0m\u0007           \u001b[0m\u001b[36m___\u001b[0m \u001b[0m\u001b[36m____\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.1345277,
        "line": "         \u001b[0m\u001b[36m⎛   ⎛ ,----\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.1348522,
        "line": "          \u001b[0m\u001b[36m\\  //==--'\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.135114,
        "line": "     \u001b[0m\u001b[1;35m_//|,.·\u001b[0m\u001b[36m//==--'\u001b[0m    \u001b[0m\u001b[37m______\u001b[0m\u001b[32m____\u001b[0m\u001b[37m_\u001b[0m\u001b[32m____\u001b[0m\u001b[37m___\u001b[0m\u001b[32m____\u001b[0m\u001b[37m__\u001b[0m\u001b[32m____\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.1353474,
        "line": "    \u001b[0m\u001b[1;35m_\u001b[0m\u001b[33mOO≣=-\u001b[0m\u001b[1;35m  \u001b[0m\u001b[36m︶\u001b[0m\u001b[1;35m \u001b[0mᴹw\u001b[0m\u001b[1;35m ⎞_§\u001b[0m \u001b[0m\u001b[37m______\u001b[0m\u001b[32m  ___\\ ___\\ ,\\__ \\/ __ \\\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.1355414,
        "line": "   \u001b[0m\u001b[1;35m(\u001b[0m\u001b[1;35m∞\u001b[0m\u001b[1;35m)\u001b[0m\u001b[1;35m_,\u001b[0m\u001b[1;35m )  (     |\u001b[0m  \u001b[0m\u001b[37m______\u001b[0m\u001b[32m/__  \\/ /__ / /_/ / /_/ /\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.1357694,
        "line": "     \u001b[0m\u001b[1;35m¨--¨|| |- (  /\u001b[0m \u001b[0m\u001b[37m______\u001b[0m\u001b[32m\\____/ \\___/ \\__^_/  .__/\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.135953,
        "line": "         \u001b[0m\u001b[34m««\u001b[0m\u001b[1;35m_/\u001b[0m  \u001b[0m\u001b[34m«\u001b[0m\u001b[1;35m_/\u001b[0m \u001b[0m\u001b[34mjgs/bd808\u001b[0m                \u001b[0m\u001b[32m/_/\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.1361902,
        "line": "\n"
    },
    {
        "timestamp": 1729791079.144785,
        "line": "\u001b[32m17:31:19 Started scap sync-world: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]]\u001b[0m\n"
    },
    {
        "timestamp": 1729791079.3692968,
        "line": "\u001b[32m17:31:19 Started cache_git_info\u001b[0m\n"
    },
    {
        "timestamp": 1729791080.7728138,
        "line": "\u001b[32m17:31:20 Finished cache_git_info (duration: 00m 01s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791080.7803717,
        "line": "\u001b[32m17:31:20 Started l10n-update\u001b[0m\n"
    },
    {
        "timestamp": 1729791081.7377357,
        "line": "\u001b[32m17:31:21 Updating ExtensionMessages-1.43.0-wmf.19.php\u001b[0m\n"
    },
    {
        "timestamp": 1729791086.2245903,
        "line": "\u001b[32m17:31:26 Updating LocalisationCache for 1.43.0-wmf.19 using 10 thread(s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791086.2250357,
        "line": "\u001b[32m17:31:26 Running rebuildLocalisationCache.php\u001b[0m\n"
    },
    {
        "timestamp": 1729791087.267052,
        "line": "\u001b[32m17:31:27 0 languages rebuilt out of 1\u001b[0m\n"
    },
    {
        "timestamp": 1729791087.2674003,
        "line": "\u001b[32m17:31:27 Use --force to rebuild the caches which are still fresh.\u001b[0m\n"
    },
    {
        "timestamp": 1729791088.5101168,
        "line": "\u001b[32m17:31:28 Generating JSON versions and md5 files (as www-data)\u001b[0m\n"
    },
    {
        "timestamp": 1729791090.2729962,
        "line": "\u001b[32m17:31:30 Updating ExtensionMessages-1.43.0-wmf.20.php\u001b[0m\n"
    },
    {
        "timestamp": 1729791094.715798,
        "line": "\u001b[32m17:31:34 Updating LocalisationCache for 1.43.0-wmf.20 using 10 thread(s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791094.7161608,
        "line": "\u001b[32m17:31:34 Running rebuildLocalisationCache.php\u001b[0m\n"
    },
    {
        "timestamp": 1729791095.72105,
        "line": "\u001b[32m17:31:35 0 languages rebuilt out of 1\u001b[0m\n"
    },
    {
        "timestamp": 1729791095.7213402,
        "line": "\u001b[32m17:31:35 Use --force to rebuild the caches which are still fresh.\u001b[0m\n"
    },
    {
        "timestamp": 1729791096.9879594,
        "line": "\u001b[32m17:31:36 Generating JSON versions and md5 files (as www-data)\u001b[0m\n"
    },
    {
        "timestamp": 1729791097.7331192,
        "line": "\u001b[32m17:31:37 Finished l10n-update (duration: 00m 16s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791097.7394972,
        "line": "\u001b[32m17:31:37 Checking for new runtime errors locally\u001b[0m\n"
    },
    {
        "timestamp": 1729791100.1244893,
        "line": "\u001b[32m17:31:40 Started build-and-push-container-images\u001b[0m\n"
    },
    {
        "timestamp": 1729791100.1310284,
        "line": "\u001b[32m17:31:40 K8s images build/push output redirected to /home/debian/scap-image-build-and-push-log\u001b[0m\n"
    },
    {
        "timestamp": 1729791113.0190358,
        "line": "\u001b[32m17:31:53 Finished build-and-push-container-images (duration: 00m 12s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791113.0828044,
        "line": "\u001b[32m17:31:53 Started sync-masters\u001b[0m\n"
    },
    {
        "timestamp": 1729791113.089464,
        "line": "17:31:53 sync-masters:   0% (ok: 0; fail: 0; left: 1)                           \n"
    },
    {
        "timestamp": 1729791117.9000127,
        "line": "17:31:57 sync-masters: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)             \n"
    },
    {
        "timestamp": 1729791117.9002554,
        "line": "\u001b[32m17:31:57 Finished sync-masters (duration: 00m 04s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791117.9685087,
        "line": "\u001b[32m17:31:57 Started sync-testservers-k8s\u001b[0m\n"
    },
    {
        "timestamp": 1729791118.127536,
        "line": "17:31:58 K8s deployment progress:   0% (ok: 0; fail: 0; left: 1)                \n"
    },
    {
        "timestamp": 1729791125.1697536,
        "line": "17:32:05 K8s deployment progress: 100% (ok: 1; fail: 0; left: 0)                \n"
    },
    {
        "timestamp": 1729791125.1700563,
        "line": "\u001b[32m17:32:05 Finished sync-testservers-k8s (duration: 00m 07s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791125.176609,
        "line": "\u001b[32m17:32:05 Started sync-testservers\u001b[0m\n"
    },
    {
        "timestamp": 1729791125.1830604,
        "line": "17:32:05 sync-testservers:   0% (ok: 0; fail: 0; left: 1)                       \n"
    },
    {
        "timestamp": 1729791129.1403675,
        "line": "17:32:09 sync-testservers: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)         \n"
    },
    {
        "timestamp": 1729791129.1406877,
        "line": "\u001b[32m17:32:09 Per-host sync duration: average 4.0s, median 4.0s\u001b[0m\n"
    },
    {
        "timestamp": 1729791129.142686,
        "line": "\u001b[32m17:32:09 rsync transfer: average 361,996 bytes/host, total 361,996 bytes\u001b[0m\n"
    },
    {
        "timestamp": 1729791129.143222,
        "line": "\u001b[32m17:32:09 Started scap-cdb-rebuild\u001b[0m\n"
    },
    {
        "timestamp": 1729791129.1499414,
        "line": "17:32:09 scap-cdb-rebuild:   0% (ok: 0; fail: 0; left: 1)                       \n"
    },
    {
        "timestamp": 1729791130.2244906,
        "line": "17:32:10 scap-cdb-rebuild: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)         \n"
    },
    {
        "timestamp": 1729791130.224729,
        "line": "\u001b[32m17:32:10 Finished scap-cdb-rebuild (duration: 00m 01s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.2332773,
        "line": "\u001b[32m17:32:10 Started sync_wikiversions\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.2394514,
        "line": "17:32:10 sync_wikiversions:   0% (ok: 0; fail: 0; left: 1)                      \n"
    },
    {
        "timestamp": 1729791130.624655,
        "line": "17:32:10 sync_wikiversions: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)        \n"
    },
    {
        "timestamp": 1729791130.62566,
        "line": "\u001b[32m17:32:10 Finished sync_wikiversions (duration: 00m 00s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.637763,
        "line": "\u001b[32m17:32:10 Started php-fpm-restarts\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.6477823,
        "line": "\u001b[32m17:32:10 Running '/usr/local/sbin/check-and-restart-php php7.4-fpm 9223372036854775807' on 1 host(s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.6487672,
        "line": "17:32:10 php-fpm-restart:   0% (ok: 0; fail: 0; left: 0)                        \n"
    },
    {
        "timestamp": 1729791130.906611,
        "line": "17:32:10 php-fpm-restart: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)          \n"
    },
    {
        "timestamp": 1729791130.9069424,
        "line": "\u001b[32m17:32:10 Finished php-fpm-restarts (duration: 00m 00s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.9140558,
        "line": "\u001b[32m17:32:10 Finished sync-testservers (duration: 00m 05s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.920383,
        "line": "\u001b[32m17:32:10 Started check-testservers\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.9264805,
        "line": "\u001b[32m17:32:10 Executing check 'check_testservers_baremetal'\u001b[0m\n"
    },
    {
        "timestamp": 1729791130.9267495,
        "line": "\u001b[32m17:32:10 Executing check 'check_testservers_k8s'\u001b[0m\n"
    },
    {
        "timestamp": 1729791133.493145,
        "line": "\u001b[32m17:32:13 Finished check-testservers (duration: 00m 02s)\u001b[0m\n"
    },
    {
        "timestamp": 1729791133.5057504,
        "line": "\u001b[32m17:32:13 trainbranchbot, debian: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]] synced to the testservers (https://wikitech.wikimedia.org/wiki/Mwdebug)\u001b[0m\n"
    },
    {
        "timestamp": 1729791133.506054,
        "line": "Changes synced to the testservers. (see https://wikitech.wikimedia.org/wiki/Mwdebug)\nPlease do any necessary checks before continuing.\nContinue with sync? [y/N]: "
    },
    {
        "timestamp": 1729791170.1961339,
        "line": "\u001b[32m17:32:50 Sync cancelled.\u001b[0m\n"
    },
    {
        "timestamp": 1729791170.3829591,
        "line": "\u001b[0m"
    }
]
export default fakeJobOutput
