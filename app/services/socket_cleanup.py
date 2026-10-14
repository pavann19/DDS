"""Narrow recovery for Python's Windows Proactor shutdown-reset cleanup bug.

This is not an exception filter for application or protocol errors. The verified
stdlib shutdown call has already notified the protocol but has not closed the
socket or detached the transport. Complete those operations exactly once.
"""
import asyncio
import linecache
import logging
import sys
from asyncio.proactor_events import _ProactorBasePipeTransport

logger = logging.getLogger(__name__)


def recover_shutdown_reset(context):
    exc = context.get("exception")
    handle = context.get("handle")
    callback = getattr(handle, "_callback", None)
    transport = getattr(callback, "__self__", None)
    if (not isinstance(exc, ConnectionResetError)
            or getattr(exc, "winerror", None) != 10054
            or exc.__context__ is not None
            or not isinstance(transport, _ProactorBasePipeTransport)
            or getattr(callback, "__func__", None) is not
            _ProactorBasePipeTransport._call_connection_lost):
        return False
    tb = exc.__traceback__
    while tb is not None and tb.tb_frame.f_code is not callback.__func__.__code__:
        tb = tb.tb_next
    # Fail closed if Python changes the implementation or the exception instead
    # originated in protocol.connection_lost or another operation.
    if (tb is None or tb.tb_frame.f_code is not callback.__func__.__code__
            or linecache.getline(tb.tb_frame.f_code.co_filename, tb.tb_lineno).strip()
            != "self._sock.shutdown(socket.SHUT_RDWR)"):
        return False
    if transport._called_connection_lost or transport._sock is None:
        return False
    transport._sock.close()
    transport._sock = None
    if transport._server is not None:
        transport._server._detach(transport)
        transport._server = None
    transport._called_connection_lost = True
    logger.info("Completed Proactor socket cleanup after peer reset (WinError 10054)")
    return True


def install_socket_cleanup_handler(loop):
    """Install only on Windows Proactor loops; return ownership-safe restoration."""
    if sys.platform != "win32" or not isinstance(loop, asyncio.ProactorEventLoop):
        return lambda: None
    previous = loop.get_exception_handler()

    def handler(current_loop, context):
        try:
            if recover_shutdown_reset(context):
                return
        except Exception as cleanup_error:
            current_loop.default_exception_handler({
                "message": "Failed completing Proactor peer-reset cleanup",
                "exception": cleanup_error,
            })
        if previous is not None:
            previous(current_loop, context)
        else:
            current_loop.default_exception_handler(context)

    loop.set_exception_handler(handler)

    def restore():
        if loop.get_exception_handler() is handler:
            loop.set_exception_handler(previous)

    return restore
