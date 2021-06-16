# Note: This is not a self-standing Python file. It gets embedded into a
# generated program by Subplot. That is why any functions that will be in the
# full program are accessed by retrieving them via the globals() dict.
#
# Part of the interface for step functions expected by Subplot is that any
# parts of steps captured by bindings (see scap.yaml) are given to the step
# functions as keyword arguments. The step functions "declare" what they expect
# with 'foo=None' in the function definition.

import logging
import os
import random
import re


def nop(ctx, *args, **kwargs):
    pass


def mkdir(ctx, pathname=None):
    os.makedirs(pathname)


def file_from(ctx, filename=None, source=None):
    get_file = globals()["get_file"]

    dirname = os.path.dirname(filename) or "."
    logging.debug("file_from: filename=%s", filename)
    logging.debug("file_from: dirname=%s", dirname)
    if not os.path.exists(dirname):
        os.makedirs("./" + dirname)

    _write(filename, get_file(source) + b"\n")


def git_init(ctx, pathname=None):
    _runcmd(ctx, ["git", "init", pathname])
    exit_code_is(ctx, 0)

    # Write a random number to a file to make sure commit ids are
    # always different.
    _write(
        os.path.join(pathname, "dummy"), str(random.randint(0, 2 ** 62)).encode("UTF-8")
    )

    # Add the file so the repo isn't empty and has a master branch.
    _runcmd(ctx, ["git", "add", "."], cwd=pathname)
    _runcmd(ctx, ["git", "commit", "-mdummy", "dummy"], cwd=pathname)
    _save_git_tip(ctx, pathname, "master")


def git_commit_file(ctx, source=None, pathname=None):
    git_commit_file_from(ctx, filename=source, source=source, pathname=pathname)


def git_commit_file_from(ctx, filename=None, source=None, pathname=None):
    get_file = globals()["get_file"]
    _write(os.path.join(pathname, filename), get_file(source) + b"\n")

    _runcmd(ctx, ["git", "add", filename], cwd=pathname)
    exit_code_is(ctx, 0)

    _runcmd(ctx, ["git", "commit", "-qm", source], cwd=pathname)
    exit_code_is(ctx, 0)

    _save_git_tip(ctx, pathname, "master")


def git_am(ctx, dirname=None, filename=None):
    _runcmd(ctx, ["git", "am", filename], cwd=dirname)


def git_am_abort(ctx, dirname=None):
    _runcmd(ctx, ["git", "am", "--abort"], cwd=dirname)


def git_am_in_progress(ctx, pathname=None):
    _runcmd(ctx, ["git", "am", "--abort"], cwd=pathname)
    stdout = ctx["stdout"]
    assert ctx["exit"] != 0
    assert "git am" in stdout


def run_scap_subcommand_help(ctx, subcommand=None):
    _scap(ctx, [subcommand, "--help"])
    runcmd_exit_code_is = globals()["runcmd_exit_code_is"]
    runcmd_exit_code_is(ctx, 0)


def run_scap(ctx):
    _scap(ctx, [])


def run_scap_version(ctx):
    _scap(ctx, ["version"])


def run_scap_sync(ctx):
    _scap(ctx, ["sync"])


def run_scap_apply_patches(ctx, code=None, patches=None, train=None):
    _scap(
        ctx,
        [
            "apply-patches",
            f"-Dstage_dir:{code}",
            f"-Dpatch_path:{patches}",
            "--train",
            train,
        ],
    )


def run_scap_test_patches(ctx, code=None, patches=None, train=None):
    _scap(
        ctx,
        [
            "test-patches",
            f"-Dstage_dir:{code}",
            f"-Dpatch_path:{patches}",
            "--train",
            train,
        ],
    )


def run_scap_list_patches(ctx, patches=None, train=None):
    _scap(ctx, ["list-patches", f"-Dpatch_path:{patches}", "--train", train])


def git_working_tree_is_clean(ctx, pathname=None):
    assert_eq = globals()["assert_eq"]
    exit, lines = git_working_tree_status(ctx, pathname=pathname)
    assert_eq(exit, 0)
    for line in lines:
        assert "You are in the middle of an am session." not in line


def git_working_tree_is_dirty(ctx, pathname=None):
    assert_ne = globals()["assert_ne"]
    exit, lines = git_working_tree_status(ctx, pathname=pathname)
    assert_ne(lines, [])


def git_working_tree_status(ctx, pathname=None):
    runcmd_get_stdout = globals()["runcmd_get_stdout"]
    runcmd_get_exit_code = globals()["runcmd_get_exit_code"]

    _runcmd(ctx, ["git", "status", "--ignore-submodules"], cwd=pathname)
    exit = runcmd_get_exit_code(ctx)
    stdout = runcmd_get_stdout(ctx)
    lines = stdout.splitlines()
    lines = [x for x in lines if x != "?? extensions/"]
    logging.debug("tree status: %r", lines)
    return exit, lines


def repo_has_changed(ctx, pathname=None):
    assert_ne = globals()["assert_ne"]
    assert_ne(_get_git_tip(pathname, "master"), _remembered_tip(ctx, pathname))


def repo_has_not_changed(ctx, pathname=None):
    assert_eq = globals()["assert_eq"]
    assert_eq(_get_git_tip(pathname, "master"), _remembered_tip(ctx, pathname))


def repo_change_matches(ctx, pathname=None, filename=None):
    get_file = globals()["get_file"]
    runcmd_get_stdout = globals()["runcmd_get_stdout"]

    # Check that the changes since the remembered tip match a patch.
    _runcmd(ctx, ["git", "diff", _remembered_tip(ctx, pathname), "HEAD"], cwd=pathname)
    exit_code_is(ctx, 0)

    diff = runcmd_get_stdout(ctx)
    _write("diff", diff.encode("UTF-8"))
    _write(filename, get_file(filename))
    _runcmd(ctx, ["interdiff", "diff", filename], input=diff)
    exit_code_is(ctx, 0)
    stdout_is_empty(ctx)


def run_scap_sync_world(ctx):
    scap = _binary("bin", "scap")
    _runcmd(ctx, [scap, "sync-world"])


def exit_code_is(ctx, exit_code=None):
    runcmd_exit_code_is = globals()["runcmd_exit_code_is"]
    runcmd_exit_code_is(ctx, exit_code)


def stdout_is_empty(ctx):
    assert_eq = globals()["assert_eq"]
    runcmd_get_stdout = globals()["runcmd_get_stdout"]

    stdout = runcmd_get_stdout(ctx)
    assert_eq(stdout, "")


def stdout_matches(ctx, pattern=None):
    assert_ne = globals()["assert_ne"]
    runcmd_get_stdout = globals()["runcmd_get_stdout"]

    stdout = runcmd_get_stdout(ctx)
    m = re.search(pattern, stdout, flags=re.M)
    if not m:
        logging.debug("stdout: %r" % repr(stdout))
        logging.debug("pattern: %r" % repr(pattern))
    assert_ne(m, None)


def stdout_matches_and_later(ctx, pattern1=None, pattern2=None):
    assert_ne = globals()["assert_ne"]
    runcmd_get_stdout = globals()["runcmd_get_stdout"]

    output = runcmd_get_stdout(ctx)
    m1 = re.search(pattern1, output)
    assert_ne(m1, None)

    m2 = re.search(pattern2, output[m1.end() :])
    assert_ne(m2, None)


def stdout_does_not_match(ctx, pattern=None):
    assert_eq = globals()["assert_eq"]
    runcmd_get_stdout = globals()["runcmd_get_stdout"]

    stdout = runcmd_get_stdout(ctx)
    m = re.search(pattern, stdout)
    assert_eq(m, None)


def stderr_matches(ctx, pattern=None):
    assert_ne = globals()["assert_ne"]
    runcmd_get_stderr = globals()["runcmd_get_stderr"]

    stderr = runcmd_get_stderr(ctx)
    m = re.search(pattern, stderr)
    assert_ne(m, None)


def _binary(*basenames):
    srcdir = globals()["srcdir"]
    return os.path.join(srcdir, *basenames)


def _scap(ctx, args):
    srcdir = globals()["srcdir"]

    scap = _binary("bin", "scap")
    env = dict(os.environ)
    env["PYTHONPATH"] = srcdir
    if "SSH_AUTH_SOCK" in env:
        del env["SSH_AUTH_SOCK"]
    _runcmd(ctx, [scap] + args, env=env)


def _runcmd(ctx, argv, cwd=None, env=None, input=""):
    if env is None:
        env = {}
    runcmd_run = globals()["runcmd_run"]
    logging.debug("_runcmd: cwd={}".format(cwd))
    runcmd_run(ctx, argv, cwd=cwd, env=env)


def _write(filename, content):
    dirname = os.path.dirname(filename)
    if dirname and not os.path.exists(dirname):
        os.makedirs(dirname)
    with open(filename, "wb") as f:
        f.write(content)


def _save_git_tip(ctx, pathname, ref):
    tip = _get_git_tip(pathname, ref)
    tips = ctx.get("tips", {})
    tips[pathname] = tip
    ctx["tips"] = tips


def _get_git_tip(pathname, ref):
    runcmd_get_stdout = globals()["runcmd_get_stdout"]

    Context = globals()["Context"]

    dummy = Context()
    _runcmd(dummy, ["git", "rev-parse", ref], cwd=pathname)
    exit_code_is(dummy, 0)
    stdout = runcmd_get_stdout(dummy)
    return stdout.strip()


def _remembered_tip(ctx, pathname):
    ret = ctx["tips"][pathname]
    return ret


def remove_file(ctx, filename=None):
    os.remove(filename)
