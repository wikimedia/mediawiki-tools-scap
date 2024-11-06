const sync_remaining_script = [
  {
    "type": "status",
    "status": "trainbranchbot, debian: Continuing with sync",
    "gap": 0.009709835052490234
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:11 trainbranchbot, debian: Continuing with sync\u001b[0m\n",
    "gap": 0.0014009475708007812
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:11 Started sync-canaries-k8s\u001b[0m\n",
    "gap": 0.0011091232299804688
  },
  {
    "type": "status",
    "status": "Started sync-canaries-k8s",
    "gap": 0.011042356491088867
  },
  {
    "type": "status",
    "status": "Started Running helmfile -e traindev --selector name=canary write-values --output-file-template /tmp/tmpcit7uw9a in /srv/deployment-charts/helmfile.d/services/mw-web",
    "gap": 0.008914470672607422
  },
  {
    "type": "status",
    "status": "Finished Running helmfile -e traindev --selector name=canary write-values --output-file-template /tmp/tmpcit7uw9a in /srv/deployment-charts/helmfile.d/services/mw-web (duration: 00m 00s)",
    "gap": 0.0978078842163086
  },
  {
    "type": "line",
    "line": "18:29:11 K8s deployment progress:   0% (ok: 0; fail: 0; left: 1)                \n",
    "gap": 0.08527946472167969
  },
  {
    "type": "status",
    "status": "Started Running helmfile -e traindev --selector name=canary apply in /srv/deployment-charts/helmfile.d/services/mw-web",
    "gap": 0.31770753860473633
  },
  {
    "type": "status",
    "status": "Finished Running helmfile -e traindev --selector name=canary apply in /srv/deployment-charts/helmfile.d/services/mw-web (duration: 00m 06s)",
    "gap": 6.868600606918335
  },
  {
    "type": "line",
    "line": "18:29:19 K8s deployment progress: 100% (ok: 1; fail: 0; left: 0)                \n",
    "gap": 0.041730403900146484
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:19 Finished sync-canaries-k8s (duration: 00m 07s)\u001b[0m\n",
    "gap": 0.0003304481506347656
  },
  {
    "type": "status",
    "status": "Finished sync-canaries-k8s (duration: 00m 07s)",
    "gap": 0.007078886032104492
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:19 Waiting 20 seconds for canary traffic...\u001b[0m\n",
    "gap": 0.0004284381866455078
  },
  {
    "type": "status",
    "status": "Waiting 20 seconds for canary traffic...",
    "gap": 0.006107330322265625
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:39 Logstash checker Counted 9 error(s) in the last 20 seconds. OK.\u001b[0m\n",
    "gap": 20.026267051696777
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:39 Started sync-prod-k8s\u001b[0m\n",
    "gap": 0.001474618911743164
  },
  {
    "type": "status",
    "status": "Started sync-prod-k8s",
    "gap": 0.013412714004516602
  },
  {
    "type": "status",
    "status": "Started Running helmfile -e traindev --selector name=main write-values --output-file-template /tmp/tmp0fja_u_x in /srv/deployment-charts/helmfile.d/services/mw-web",
    "gap": 0.00950479507446289
  },
  {
    "type": "status",
    "status": "Finished Running helmfile -e traindev --selector name=main write-values --output-file-template /tmp/tmp0fja_u_x in /srv/deployment-charts/helmfile.d/services/mw-web (duration: 00m 00s)",
    "gap": 0.08891105651855469
  },
  {
    "type": "line",
    "line": "18:29:39 K8s deployment progress:   0% (ok: 0; fail: 0; left: 1)                \n",
    "gap": 0.08671975135803223
  },
  {
    "type": "status",
    "status": "Started Running helmfile -e traindev --selector name=main apply in /srv/deployment-charts/helmfile.d/services/mw-web",
    "gap": 0.3192319869995117
  },
  {
    "type": "status",
    "status": "Finished Running helmfile -e traindev --selector name=main apply in /srv/deployment-charts/helmfile.d/services/mw-web (duration: 00m 06s)",
    "gap": 6.913289785385132
  },
  {
    "type": "line",
    "line": "18:29:46 K8s deployment progress: 100% (ok: 1; fail: 0; left: 0)                \n",
    "gap": 0.03627634048461914
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:46 Finished sync-prod-k8s (duration: 00m 07s)\u001b[0m\n",
    "gap": 0.0002543926239013672
  },
  {
    "type": "status",
    "status": "Finished sync-prod-k8s (duration: 00m 07s)",
    "gap": 0.006310701370239258
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:46 Started sync-apaches\u001b[0m\n",
    "gap": 0.0003535747528076172
  },
  {
    "type": "status",
    "status": "Started sync-apaches",
    "gap": 0.0059680938720703125
  },
  {
    "type": "line",
    "line": "18:29:46 sync-apaches:   0% (ok: 0; fail: 0; left: 1)                           \n",
    "gap": 0.00040602684020996094
  },
  {
    "type": "line",
    "line": "18:29:50 sync-apaches: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)             \n",
    "gap": 3.880542516708374
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:50 Per-host sync duration: average 3.9s, median 3.9s\u001b[0m\n",
    "gap": 0.0002467632293701172
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:50 rsync transfer: average 361,996 bytes/host, total 361,996 bytes\u001b[0m\n",
    "gap": 0.0003936290740966797
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:50 Finished sync-apaches (duration: 00m 03s)\u001b[0m\n",
    "gap": 0.0004801750183105469
  },
  {
    "type": "status",
    "status": "Finished sync-apaches (duration: 00m 03s)",
    "gap": 0.006490230560302734
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:50 Started scap-cdb-rebuild\u001b[0m\n",
    "gap": 0.00034737586975097656
  },
  {
    "type": "status",
    "status": "Started scap-cdb-rebuild",
    "gap": 0.006087064743041992
  },
  {
    "type": "line",
    "line": "18:29:50 scap-cdb-rebuild:   0% (ok: 0; fail: 0; left: 2)                       \n",
    "gap": 0.00034689903259277344
  },
  {
    "type": "line",
    "line": "18:29:51 scap-cdb-rebuild: 100% (in-flight: 0; ok: 2; fail: 0; left: 0)         \n",
    "gap": 1.125810146331787
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:51 Finished scap-cdb-rebuild (duration: 00m 01s)\u001b[0m\n",
    "gap": 0.00018739700317382812
  },
  {
    "type": "status",
    "status": "Finished scap-cdb-rebuild (duration: 00m 01s)",
    "gap": 0.0058438777923583984
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:51 Started sync_wikiversions\u001b[0m\n",
    "gap": 0.00029540061950683594
  },
  {
    "type": "status",
    "status": "Started sync_wikiversions",
    "gap": 0.005778789520263672
  },
  {
    "type": "line",
    "line": "18:29:51 sync_wikiversions:   0% (ok: 0; fail: 0; left: 2)                      \n",
    "gap": 0.0003600120544433594
  },
  {
    "type": "line",
    "line": "18:29:52 sync_wikiversions: 100% (in-flight: 0; ok: 2; fail: 0; left: 0)        \n",
    "gap": 0.39142394065856934
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:52 Finished sync_wikiversions (duration: 00m 00s)\u001b[0m\n",
    "gap": 0.0014798641204833984
  },
  {
    "type": "status",
    "status": "Finished sync_wikiversions (duration: 00m 00s)",
    "gap": 0.010810375213623047
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:52 Started php-fpm-restarts\u001b[0m\n",
    "gap": 0.0010678768157958984
  },
  {
    "type": "status",
    "status": "Started php-fpm-restarts",
    "gap": 0.009773969650268555
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:52 Running '/usr/local/sbin/check-and-restart-php php7.4-fpm 9223372036854775807' on 1 host(s)\u001b[0m\n",
    "gap": 0.0010890960693359375
  },
  {
    "type": "line",
    "line": "18:29:52 php-fpm-restart:   0% (ok: 0; fail: 0; left: 0)                        \n",
    "gap": 0.001039266586303711
  },
  {
    "type": "line",
    "line": "18:29:52 php-fpm-restart: 100% (in-flight: 0; ok: 1; fail: 0; left: 0)          \n",
    "gap": 0.27623677253723145
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:52 Finished php-fpm-restarts (duration: 00m 00s)\u001b[0m\n",
    "gap": 0.00024771690368652344
  },
  {
    "type": "status",
    "status": "Finished php-fpm-restarts (duration: 00m 00s)",
    "gap": 0.006343841552734375
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:52 Running purgeMessageBlobStore.php\u001b[0m\n",
    "gap": 0.0003802776336669922
  },
  {
    "type": "status",
    "status": "Finished scap sync-world: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]] (duration: 01m 45s)",
    "gap": 1.1839027404785156
  },
  {
    "type": "line",
    "line": "\u001b[32m18:29:53 Finished scap sync-world: Backport for [[gerrit:7|group1 to 1.43.0-wmf.20 (T123123)]] (duration: 01m 45s)\u001b[0m\n",
    "gap": 0.0003719329833984375
  },
  {
    "type": "line",
    "line": "\u001b[0m",
    "gap": 0.1516561508178711
  },
  {
    "type": "eof",
    "gap": 2.3603439331054688e-05
  }
]

export default sync_remaining_script
