.. _terminal:

#######################
Terminal User Interface
#######################

Scap includes a few classes and generally-useful utility functions which assist
in implementing a well behaved command line interface and fancy features such
as: colored & formatted output, confirmation prompts and progress bars.

scap.terminal.term
~~~~~~~~~~~~~~~~~~

Terminal output formatting is handled by the TerminalIO class which can be
accessed through the global terminal.term instance. term's methods write
directly to the wrapped output stream, instead of returning control sequences
as strings and leaving the string concatenation up to the caller.
This allows for a cleaner, "fluent interface[1]" style.

For example:

.. code-block:: python

    from scap.terminal import TERM

    TERM.move(TERM.height - 1, 0)
    TERM.fg(4).write("some text")
    TERM.fg(7).bold().write(' bold text')
    TERM.clear_eol().flush().restore()
..

* [1] https://en.wikipedia.org/wiki/Fluent_interface

Colors:

.. code-block:: python

    from scapext.terminal import TerminalIO

    term = TerminalIO()

    for idx in range(16):
        term.fg(idx, 'Color {0}'.format(idx)).nl()
..

Scroll Regions:

.. code-block:: python

    term = TerminalIO(sys.__stdout__)
    term.scroll_region(0, term.height - 1) \
    term.move(term.height, 0)
    term.write('Persistent Status Line below scroll region')
    term.clear_eol()
    term.move_x(term.height - 1) \
    term.writeln('inside scroll region')
    for i in range(1, 100)
        term.writeln('Scrolling text %s' % i)
..
