# -*- coding: utf-8 -*-
"""
    scap.ssh
    ~~~~~~~~
    This module provides functions for running commands on remote hosts
    via SSH.

"""
import os
import select
import subprocess


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
                ssh_command = SSH + (host, command)
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


def cluster_run(hosts, command, max_fails=0):
    """Run a command via SSH on multiple hosts concurrently. Wait until
    all spawned processes complete, then return a tuple of (ok, failed)
    mappings."""
    max_fails = round(max_fails)
    failed = {}
    ok = {}
    for host, status, output in cluster_ssh(hosts, command):
        if status == 0:
            ok[host] = status, output
        else:
            failed[host] = status, output
            if len(failed) > max_fails:
                raise RuntimeError(command, failed)
    return ok, failed
