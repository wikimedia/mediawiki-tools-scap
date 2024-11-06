import os
import sys

from typing import Optional

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


def GetIO():
    cls = SpiderpigIO if spiderpig_mode() else TerminalIO
    return cls()


class UserIOBase:
    def output_line(self, line: str, sensitive: bool = False):
        self._output_line(line, sensitive)

    def input_line(self, prompt: str) -> str:
        if not interactive():
            return ""

        return self._input_line(prompt)

    def prompt_choices(self, question: str, choices, default=None) -> str:
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
            resp = self._prompt_choices(question, choices, default).strip()
            if not resp and default:
                return default

            if resp in choices.values():
                return resp
            if resp:
                self.output_line(f"Invalid choice: {repr(resp)}")

    def prompt_user_for_confirmation(self, prompt_message, default="n") -> bool:
        """
        Prompts user with `prompt_message` and expects yes/no answer.
        """
        return self.prompt_choices(prompt_message, bool, default) == "y"

    def report_status(self, status: Optional[str]):
        self._report_status(status)


class TerminalIO(UserIOBase):
    def _output_line(self, line, sensitive):
        print(line)

    def _input_line(self, prompt) -> str:
        return input(prompt)

    def _prompt_choices(self, question, choices, default) -> str:
        """
        Use the 'choices' dict to present a list of choices to the user.
        Returns the user's response.  Validation of the response is not
        performed.
        """
        return input(self.generate_prompt_text(question, choices, default))

    @classmethod
    def generate_prompt_text(self, question, choices, default=None) -> str:
        if choices == YES_NO_CHOICES:
            if default == "y":
                prompt = "[Y/n]"
            elif default == "n":
                prompt = "[y/N]"
            else:
                prompt = "[y/n]"
            return f"{question} {prompt}: "

        choices_text = ""
        for choice, key in choices.items():
            if key == default:
                before = ansi.esc(ansi.BRIGHT)
                after = ansi.esc(ansi.RESET_ALL)
            else:
                before = ""
                after = ""
            choices_text += f"{before}[{key}] {choice}{after}\n"
        default_text = f" (default: [{default}])" if default else ""

        return f"{choices_text}{question}{default_text}: "

    def _report_status(self, status: Optional[str]):
        pass


class SpiderpigIO(UserIOBase):
    def _output_line(self, line, sensitive):
        return spiderpigio.output_line(line, sensitive)

    def _input_line(self, prompt) -> str:
        return spiderpigio.input_line(prompt)

    def _prompt_choices(self, question, choices, default):
        return spiderpigio.prompt_choices(question, choices, default)

    def _report_status(self, status: Optional[str]):
        spiderpigio.report_status(status)
