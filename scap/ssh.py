# -*- coding: utf-8 -*-
"""
    scap.ssh
    ~~~~~~~~
    This module provides functions for running commands on remote hosts
    via SSH.

"""
import logging
import os
import select
import shlex
import subprocess
import sys


SSH = ('/usr/bin/ssh', '-oBatchMode=yes', '-oSetupTimeout=10')


def cluster_ssh(hosts, command, limit=80):
    """Run a command via SSH on multiple hosts concurrently."""
    hosts = set(hosts)
    procs = {}
    fds = {}
    poll = select.epoll()
    try:
        while hosts or procs:
            if hosts and len(procs) < limit:
                host = hosts.pop()
                ssh_command = SSH + (host,) + tuple(command)
                proc = subprocess.Popen(ssh_command, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, preexec_fn=os.setsid)
                procs[proc.pid] = (proc, host)
                poll.register(proc.stdout, select.EPOLLIN)
            else:
                pid, status = os.waitpid(-1, os.WNOHANG)
                for fd, event in poll.poll(0.01):
                    fds[fd] = fds.get(fd, '') + os.read(fd, 1048576)
                if pid:
                    status = -(status & 255) or (status >> 8)
                    proc, host = procs.pop(pid)
                    poll.unregister(proc.stdout)
                    output = fds.pop(proc.stdout.fileno(), '')
                    yield host, status, output
    finally:
        poll.close()
        for pid, (proc, host) in procs.items():
            proc.kill()


def cluster_monitor(hosts, command):
    """Monitor execution of a command on the cluster.

    Run a command on the cluster via ssh and monitor progress with a live
    updating progress message and real-time error logging. Progress messages
    are sent to stderr. Failed commands will be logged as warnings as they
    occur. Returns the count of successful and failed commands.

    :param hosts: Hosts to execute command on
    :type host: sequence
    :param command: Command to execute
    :type command: str or sequence
    :returns: tuple (count of successful responses, count of failed responses)
    """
    logger = logging.getLogger('ssh')
    expect = len(hosts)
    ok = 0
    failed = 0
    done = 0

    try:
        command = shlex.split(command)
    except AttributeError:
        pass

    try:
        for done, (host, status, output) in \
                enumerate(cluster_ssh(hosts, command), start=1):
            if status == 0:
                ok += 1
            else:
                failed += 1
                if done > 1:
                    # Start a new console line on the assumption that logger
                    # will be appending to console as well.
                    sys.stderr.write('\n')
                logger.warning('%s on %s returned [%d]: %s',
                    command, host, status, output)

            # Jump cursor back to position 0 and write status message.
            sys.stderr.write('\r%s: %.0f%% (ok: %d; fail: %d; left: %d)' % (
                command, 100.0 * (float(done) / expect), ok, failed,
                expect - done))

    finally:
        sys.stderr.write('\n')

    return (ok, failed)
