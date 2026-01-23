import signal
from contextlib import contextmanager


@contextmanager
def sigint_interruptible():
    """
    Ensure SIGINT raises KeyboardInterrupt instead of restarting blocked
    system calls (PEP 475). Temporarily installs the default SIGINT handler
    and disables SA_RESTART via signal.siginterrupt(), then restores the
    previous handler on exit.
    """
    prev_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.siginterrupt(signal.SIGINT, True)
    try:
        yield
    finally:
        signal.siginterrupt(signal.SIGINT, False)
        signal.signal(signal.SIGINT, prev_handler)
