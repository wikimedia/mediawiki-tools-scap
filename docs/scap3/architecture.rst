
.. _architecture:

############
Architecture
############

This document is mainly relevant only for git-based repositories,
which currently (2015-10) does **NOT** include MediaWiki deploys.

Process model
-------------

Scap's basic architecture consists of a main ``scap deploy`` process run by the
user on the deployment host and a number of spawned ``scap deploy-local``
subprocesses running over SSH that perform the actual work of each deployment
"stage".

Structured logging events are sent back over the existing SSH channel, as
line-wise JSON, where they are parsed and fed back into a unified logging
stream. Optionally, a ``scap deploy-log`` process may be started by the user to
filter and view the logging stream during or after the run.

.. graphviz::

  digraph "scap-architecture" {
    graph [fontname="Helvetica", rankdir=LR];
    node [fontname="Helvetica", style=filled];
    edge [fontname="Helvetica", color="#222"];
    
    // Class definitions
    node[shape=box, style="filled,rounded", color="#00af89", fontcolor=white];
    deploy [label="scap deploy"];
    deploy_log [label="scap deploy-log"];
    deploy_local [label="scap deploy-local"];
    service [label="service"]

    node[shape=box, color="#555", fontcolor="#222", style=solid];
    user [label="Deployer"];
    
    node[shape=circle, style="filled", color="#2962cc", fontcolor=white];
    logger [label="logger"];
    httpd [label="httpd"];
    git [label="git"];
    jinja [label="jinja"];
    systemctl [label="systemctl"];
    
    node[shape=cylinder, color="#555"];
    log [label="scap/log/{tag}.log"];
    repo [label="repo"];
    config [label="config"];
    
    node[shape=circle, color="#ffb50d", fontcolor="#222"];
    check [label="checks"];

    // Connections
    user -> deploy [style=dashed];
    user -> deploy_log [style=dashed];

    deploy -> deploy_local [label="SSH", style=dashed, dir=both];
    
    deploy -> logger;
    deploy -> httpd;

    logger -> log [style=dashed];
    log -> deploy_log [dir=both, style=dashed];
    
    deploy_local -> git;
    deploy_local -> jinja;
    deploy_local -> systemctl;
    deploy_local -> check;
    
    httpd -> git [style=dashed, dir=both];
    git -> repo [style=dashed];
    
    jinja -> config [style=dashed];
    
    config -> service [style=dashed];
    systemctl -> service [style=dashed];
    check -> service [style=dashed];
  }
  
Process flow
------------

Scap's overall deployment process is represented in the following diagram,
with a detailed explanation below.

.. graphviz::

  digraph "scap-process-flow" {
    graph [fontname="Helvetica", rankdir=TD];
    node [fontname="Helvetica", style=filled];
    edge [fontname="Helvetica", color="#222"];
    
    // Node styles
    node[shape=box, style="filled,rounded", color="#00af89", fontcolor=white];
    terminus [label="$ deploy"];
    deploy_complete [label="Deploy complete"];
    puppet [label="puppet"];

    node[shape=box, style=filled, color="#2962cc", fontcolor=white];
    config_stage [label="Stage: config"];
    fetch_stage [label="Stage: fetch"];
    promote_stage [label="Stage: promote"];
    finalize_stage [label="Stage: finalize"];
    
    node[shape=box, style=filled, color="#555", fontcolor=white];
    resolve_targets [label="Resolve targets"];
    prepare_config [label="Prepare config"];
    prepare_repo [label="Prepare repo"];
    provide_secrets [label="Provide secrets"];
    fetch_template [label="Fetch template"];
    combine_vars [label="Combine vars"];
    render_config [label="Render new config"];
    fetch_repo [label="Fetch repo"];
    checkout_revision [label="Checkout revision"];
    update_submodules [label="Update submodules"];
    link_repo [label="Link repo"];
    link_config [label="Link config"];
    restart_service [label="Restart service"];
    update_state [label="Update state"];
    rm_old_revs [label="Delete old revs"];
    
    node[shape=diamond, color=gray, fontcolor=black];
    next_group [label="Deploy each group"];
    group_deployed [label="Group deployed"];
    
    node[shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
    before_checks [label="Perform before checks"];
    after_checks [label="Perform after checks"];

    // Connections
    terminus -> resolve_targets -> prepare_config -> prepare_repo -> next_group;
    next_group -> config_stage -> before_checks -> fetch_template -> combine_vars -> render_config -> after_checks;

    config_stage -> fetch_stage;
    fetch_stage -> promote_stage;
    promote_stage -> finalize_stage;
    
    fetch_stage -> before_checks_f -> fetch_repo -> checkout_revision -> update_submodules -> after_checks_f;
    promote_stage -> before_checks_p -> link_repo -> link_config -> restart_service -> after_checks_p;
    finalize_stage -> before_checks_fin -> update_state -> rm_old_revs -> after_checks_fin;

    finalize_stage -> group_deployed;
    group_deployed -> next_group [style=dashed];
    group_deployed -> deploy_complete;
    
    puppet -> provide_secrets -> combine_vars;

    // Separate nodes for multiple uses of "checks"
    before_checks_f [label="Perform before checks", shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
    after_checks_f [label="Perform after checks", shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
    
    before_checks_p [label="Perform before checks", shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
    after_checks_p [label="Perform after checks", shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
    
    before_checks_fin [label="Perform before checks", shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
    after_checks_fin [label="Perform after checks", shape=circle, style=filled, color="#ffb50d", fontcolor="#222"];
  }

After some preparation of the local repo and configuration, the main
deployment process is run for each of the configured target groups. This
process is composed of four distinct stages, *config*, *fetch*, *promote*, and
*finalize*, run across the group targets in that order. Concurrency for each
stage can be either completely serial or highly parallel, again depending on
configuration.  For fine tuning of the groups and stage concurrency, see
``server_groups`` and ``batch_size`` under :ref:`available-configuration`.

