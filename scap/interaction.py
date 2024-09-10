import os
import sys

import scap.ansi as ansi
import scap.spiderpig.io as spiderpigio

YES_NO_CHOICES = {"Yes": "y", "No": "n"}


def have_terminal() -> bool:
    """
    Returns True if both stdin and stdout are attached to a terminal.
    """
    return sys.stdin.isatty() and sys.stdout.isatty()


def interactive() -> bool:
    """
    Returns True if there is some means available to interact with a user.
    """
    return spiderpig_mode() or have_terminal()


def spiderpig_mode() -> bool:
    return bool(os.environ.get("SPIDERPIG_IO_KEY"))


def _get_io():
    return SpiderpigInteraction if spiderpig_mode() else TerminalInteraction


def input_line(prompt: str) -> str:
    if not interactive():
        return ""

    return _get_io().input_line(prompt)


def prompt_choices(question: str, choices, default=None) -> str:
    """
    Use the 'choices' dict to present a list of choices to the user.
    If 'choices' is the class 'bool', then choices is defined to be
    {"Yes": "y", "No": "n"}.

    'choices' maps choice descriptions to the key the user should enter to
    select that choice, for example: {"Continue": "c", "Exit": "e"}.

    Returns the valid choice that the user made.  Re-prompts if an
    invalid choice is made.  If `default` is supplied, it will be returned
    if the terminal is not interactive, or if the user just hits enter.
    """

    if not interactive() and default:
        # NOTE: No prompt is seen at all under these circumstances.
        return default

    if choices == bool:
        choices = YES_NO_CHOICES

    while True:
        resp = _get_io().prompt_choices(question, choices, default).strip()
        if not resp and default:
            return default

        if resp in choices.values():
            return resp
        if resp:
            print(f"Invalid choice: {repr(resp)}")


def prompt_user_for_confirmation(prompt_message, default="n") -> bool:
    """
    Prompts user with `prompt_message` and expects yes/no answer.
    """
    return prompt_choices(prompt_message, bool, default) == "y"


class TerminalInteraction:
    @classmethod
    def input_line(cls, prompt) -> str:
        return input(prompt)

    @classmethod
    def prompt_choices(cls, question, choices, default) -> str:
        """
        Use the 'choices' dict to present a list of choices to the user.
        Returns the user's response.  Validation of the response is not
        performed.
        """
        return input(cls.generate_prompt_text(question, choices, default))

    @classmethod
    def generate_prompt_text(cls, question, choices, default=None) -> str:
        if choices == YES_NO_CHOICES:
            if default == "y":
                prompt = "[Y/n]"
            elif default == "n":
                prompt = "[y/N]"
            else:
                prompt = "[y/n]"
            return f"{question} {prompt}: "

        for choice, key in choices.items():
            if key == default:
                before = ansi.esc(ansi.BRIGHT)
                after = ansi.esc(ansi.RESET_ALL)
            else:
                before = ""
                after = ""
            print(f"{before}[{key}] {choice}{after}")
        default_text = f" (default: [{default}])" if default else ""

        return f"{question}{default_text}: "


class SpiderpigInteraction:
    @classmethod
    def input_line(cls, prompt) -> str:
        return spiderpigio.input_line(prompt)

    @classmethod
    def prompt_choices(cls, question, choices, default):
        return spiderpigio.prompt_choices(question, choices, default)
