import asyncio
from unittest.mock import Mock

import pytest
from asyncio.proactor_events import _ProactorBasePipeTransport
from app.services.socket_cleanup import recover_shutdown_reset, install_socket_cleanup_handler


def broken_transport():
    transport = object.__new__(_ProactorBasePipeTransport)
    error = ConnectionResetError("peer reset")
    error.winerror = 10054
    transport._sock = Mock()
    transport._sock.fileno.return_value = 7
    transport._sock.shutdown.side_effect = error
    transport._protocol = Mock()
    transport._server = Mock()
    transport._called_connection_lost = False
    transport._closing = True
    return transport


def reset_context(transport):
    try:
        transport._call_connection_lost(None)
    except ConnectionResetError as exc:
        return {"exception": exc, "handle": asyncio.Handle(transport._call_connection_lost, (None,), Mock())}
    raise AssertionError("expected shutdown reset")


def test_verified_shutdown_reset_completes_close_and_detach_once():
    transport = broken_transport()
    socket, server = transport._sock, transport._server
    context = reset_context(transport)
    assert not socket.close.called  # actual installed stdlib leaves cleanup incomplete
    assert recover_shutdown_reset(context)
    socket.close.assert_called_once()
    server._detach.assert_called_once_with(transport)
    transport._protocol.connection_lost.assert_called_once_with(None)
    assert transport._sock is None and transport._called_connection_lost
    assert not recover_shutdown_reset(context)


@pytest.mark.parametrize("winerror", [10053, 10061, None])
def test_other_socket_errors_are_not_suppressed(winerror):
    transport = broken_transport()
    context = reset_context(transport)
    context["exception"].winerror = winerror
    assert not recover_shutdown_reset(context)
    transport._sock.close()  # prevent resource warnings from intentionally rejected fixture
    transport._sock = None


def test_protocol_reset_is_not_mistaken_for_shutdown_reset():
    transport = broken_transport()
    error = ConnectionResetError("protocol application failure")
    error.winerror = 10054
    transport._protocol.connection_lost.side_effect = error
    transport._sock.shutdown.side_effect = None
    socket = transport._sock
    context = reset_context(transport)
    assert not recover_shutdown_reset(context)
    assert socket.close.called


@pytest.mark.skipif(not hasattr(asyncio, "ProactorEventLoop"), reason="Windows Proactor only")
def test_handler_delegates_and_restores_previous_owner():
    loop = asyncio.ProactorEventLoop()
    previous = Mock()
    loop.set_exception_handler(previous)
    restore = install_socket_cleanup_handler(loop)
    context = {"message": "application error", "exception": ValueError("visible")}
    loop.call_exception_handler(context)
    previous.assert_called_once_with(loop, context)
    restore()
    assert loop.get_exception_handler() is previous
    restore = install_socket_cleanup_handler(loop)
    replacement = Mock()
    loop.set_exception_handler(replacement)
    restore()
    assert loop.get_exception_handler() is replacement
    loop.close()


@pytest.mark.skipif(not hasattr(asyncio, "ProactorEventLoop"), reason="Windows Proactor only")
def test_real_event_loop_callback_recovery_keeps_protocol_notification_once():
    loop = asyncio.ProactorEventLoop()
    previous = Mock()
    loop.set_exception_handler(previous)
    restore = install_socket_cleanup_handler(loop)
    transport = broken_transport()
    socket, server = transport._sock, transport._server
    loop.call_soon(transport._call_connection_lost, None)
    loop.run_until_complete(asyncio.sleep(0))
    previous.assert_not_called()
    socket.close.assert_called_once()
    server._detach.assert_called_once_with(transport)
    transport._protocol.connection_lost.assert_called_once_with(None)
    restore()
    loop.close()


def test_chained_protocol_error_is_never_hidden_by_shutdown_recovery():
    transport = broken_transport()
    transport._protocol.connection_lost.side_effect = ValueError("application failure")
    context = reset_context(transport)
    assert isinstance(context["exception"].__context__, ValueError)
    assert not recover_shutdown_reset(context)
    transport._sock.close()
    transport._sock = None
