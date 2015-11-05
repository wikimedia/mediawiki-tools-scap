############
Architecture
############

This document is mainly relevant only for git-based repositories,
which currently (2015-10) does **NOT** include MediaWiki deploys.

Process model
-------------

Scap's basic architecture consists of a main ``deploy`` process run by the
user on the deployment host and a number of spawned ``deploy-local``
subprocesses running over SSH that perform the actual work of each deployment
"stage".

Structured logging events are sent back over the existing SSH channel, as
line-wise JSON, where they are parsed and fed back into a unified logging
stream. Optionally, a ``deploy-log`` process may be started by the user to
filter and view the logging stream during or after the run.

.. blockdiag::

  blockdiag {
    default_linecolor = "#222"
    class actor [color = "#555", shape = actor, style = none]
    class process [color = "#00af89", textcolor = white, shape = roundedbox]
    class component [color = "#2962cc", textcolor = white, shape = circle]
    class storage [color = "#555", textcolor = white, shape = flowchart.database]
    class check [color = "#ffb50d", textcolor = "#222", shape = circle]

    user -> deploy, deploy-log [hstyle = generalization, style = dashed]

    deploy <-> deploy-local [label = "SSH", style = dashed]
    deploy -> logger, httpd [hstyle = composition]

    logger -> log [style = dashed]
    log <- deploy-log

    deploy-local -> git, jinja, systemctl, checks [hstyle = composition]

    httpd <-> git -> repo [style = dashed]

    jinja -> config [style = dashed]
    config, systemctl, checks -> service [style = dashed, hstyle = generalization]

    group {
      color = none

      group {
        color = none

        user [class = actor, label = "Deployer"]
      }

      group {
        color = none
        orientation = portrait

        deploy [class = process]
        logger [class = component]
        httpd [class = component]
        log [class = storage, stacked, label = "scap/log/{tag}.log"]
        deploy-log [class = process, label = "deploy-log {filter}"]
      }

      group {
        color = none
        orientation = portrait

        deploy-local [class = process, stacked, label = "deploy-local -s {stage}"]

        group {
          color = none
          orientation = portrait

          group {
            color = none
            orientation = portrait

            git [class = component]
            repo [class = storage]
          }

          group {
            color = none
            orientation = portrait

            jinja [class = component]
            config [class = storage]
          }

          jinja [class = component]
          systemctl [class = component]
          checks [class = check, stacked]
        }

      }

      service [class = process]
    }
  }

Process flow
------------

Scap's overall deployment process is represented in the following diagram,
with a detailed explanation below.

.. actdiag::

  actdiag {
    default_linecolor = "#222"
    edge_layout = flowchart
    class terminus [color = "#00af89", textcolor = white, shape = roundedbox]
    class progress [color = "#2962cc", textcolor = white, shape = roundedbox]
    class step [color = "#555", textcolor = white]
    class control [color = "#ccc", textcolor = "#222"]
    class check [color = "#ffb50d", textcolor = "#222", shape = roundedbox]

    deploy -> resolve_targets -> prepare_config -> prepare_repo ->
      next_group ->
      deploy_local_config -> deploy_local_fetch -> deploy_local_promote

    deploy_local_promote -> group_deployed -> deploy_complete
    group_deployed -> next_group [style = dashed]

    deploy_local_config -> config_deploy_fetch -> config_deploy_vars ->
      config_deploy_render -> config_deploy_checks
    deploy_local_fetch -> fetch_repo -> fetch_checkout -> fetch_submodules ->
      fetch_checks
    deploy_local_promote -> promote_link -> promote_config ->
      promote_restart -> promote_checks

    puppet -> provide_secrets -> config_deploy_vars

    lane host {
      label = "Deploy host"
      color = "#ddd"
      fontsize = 14

      deploy [class = terminus, label = "$ deploy"]
      resolve_targets [class = step, label = "Resolve targets"]
      prepare_config [class = step, label = "Prepare config"]
      prepare_repo [class = step, label = "Prepare repo"]

      next_group [class = control, shape = flowchart.loopin, label = "Deploy each group"]

      deploy_local_config [class = progress, label = "Stage: config"]
      deploy_local_fetch [class = progress, label = "Stage: fetch"]
      deploy_local_promote [class = progress, label = "Stage: promote"]

      group_deployed [class = control, shape = flowchart.loopout, label = "Group deployed"]
      deploy_complete [class = terminus, label = "Deploy complete"]
    }

    lane target {
      label = "Deploy targets"
      color = "#ddd"
      fontsize = 14

      puppet [class = terminus]
      provide_secrets [class = step, label = "Provide secrets"]

      config_deploy_fetch [class = step, label = "Fetch template"]
      config_deploy_vars [class = step, label = "Combine vars"]
      config_deploy_render [class = step, label = "Render new config"]
      config_deploy_checks [class = check, label = "Perform checks"]

      fetch_repo [class = step, label = "Fetch repo"]
      fetch_checkout [class = step, label = "Checkout revision"]
      fetch_submodules [class = step, label = "Update submodules"]
      fetch_checks [class = check, label = "Perform checks"]

      promote_link [class = step, label = "Link repo"]
      promote_config [class = step, label = "Link config"]
      promote_restart [class = step, label = "Restart service"]
      promote_checks [class = check, label = "Perform checks"]
    }
  }

After some preparation of the local repo and configuration, the main
deployment process is run for each of the configured target groups. This
process is composed of three distinct stages, *config*, *fetch*, and
*promote*, run across the group targets in that order. Concurrency for each
stage can be either completely serial or highly parallel, again depending on
configuration.  For fine tuning of the groups and stage concurrency, see
``server_groups`` and ``batch_size`` under :ref:`available-configuration`.

