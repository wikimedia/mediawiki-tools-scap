from __future__ import absolute_import

from scap import cmd


def test_command_args():
    ssh = cmd.Command("/usr/bin/ssh", cmd.arg("user", "-oUser={}"))
    sudo = cmd.Command("sudo", cmd.arg("user", "-u {}"), "-n", "--")
    cmdline = ssh(
        "some.host",
        sudo("remote_cmd", "some", "args", user="sudo_user"),
        user="ssh_user",
    )

    assert cmdline == [
        "/usr/bin/ssh",
        "-oUser=ssh_user",
        "some.host",
        "sudo",
        "-u sudo_user",
        "-n",
        "--",
        "remote_cmd",
        "some",
        "args",
    ]
