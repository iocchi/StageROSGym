"""Sender--Receiver pairs for socket communication."""

from __future__ import absolute_import, division, print_function

import sys
import queue
import threading
import time
import socket
from socketserver import TCPServer, BaseRequestHandler

from .tools import QuitWithResources


class Sender(object):
    """Generic sender class.

    The sender is a server that runs asynchronously and sends any data
    that is passed to its send function. This never closes. I'm expecting the
    program terminates with a Ctrl-C, as often happens in long trainings.
    """

    QUEUE_SIZE = 20

    def __init__(self, msg_length, port, wait=False):
        """Initialize.

        :param msg_length: the fixed length of messages (bytes).
        :param port: (int) a port to use for incoming requests.
        :param wait: if False, a send() returns immediately; if True,
            send() waits if there are too many messages still to be sent.
        """

        # Store
        self.MSG_LENGTH = msg_length
        self._port = port

        # Create connection
        self.server = Sender.OneRequestTCPServer(
            ("0.0.0.0", port), Sender.RequestHandler)

        # Data to send
        self._data_queue = queue.Queue(self.QUEUE_SIZE if wait else 0)
        self.server._data_queue = self._data_queue

    def start(self):
        """Start sending messages on queue."""

        # Finally close
        def close():
            self.server.server_close()
            print("\nSender closed")
        QuitWithResources.add("Sender:" + str(self._port), close)

        thread = threading.Thread(target=self.server.serve_forever)
        thread.daemon = True
        thread.start()

        while not self.server.is_serving:
            time.sleep(0.1)

    def send(self, data):
        """Send data asynchronously.

        :param data: binary data
        :return: True if the data was correctly pushed to the sending queue
        """

        # Checks
        if not isinstance(data, bytes):
            raise TypeError("Can only send bytes")
        if len(data) != self.MSG_LENGTH:
            raise ValueError("Message with the wrong length")
        if not self.server.is_serving:
            return False

        # Send
        self._data_queue.put(data, block=True)
        return True

    class OneRequestTCPServer(TCPServer):
        """Restrict to only one connection."""

        request_queue_size = 1
        allow_reuse_address = True
        is_serving = False
        timeout = None

        def handle_error(self, request, client_address):
            """Stop the server on broken connection."""

            print("Broken connection", file=sys.stderr)
            self.server_close()
            self.is_serving = False

        def serve_forever(self):
            """Forward."""

            self.is_serving = True
            TCPServer.serve_forever(self)

    class RequestHandler(BaseRequestHandler):
        """This actually sends data to the client who requested."""

        def handle(self):
            """Send."""

            while True:
                data = self.server._data_queue.get(block=True, timeout=None)
                try:
                    self.request.sendall(data)

                except OSError:
                    break


class Receiver(object):
    """Generic receiver class."""

    QUEUE_SIZE = 20

    def __init__(self, msg_length, ip, port, wait=False):
        """Initialize.

        :param msg_length: the fixed length of messages (bytes)
        :param ip: ip address of the sender (str)
        :param port: port of the sender (int)
        :param wait: if True, if receive() is not called often enough,
            it no longer accepts new messages. This is only useful with a
            Sender that also waits.
        """

        self.MSG_LENGTH = msg_length
        self.ip = ip
        self.port = port

        # Create connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Received data
        self._data_queue = queue.Queue(self.QUEUE_SIZE if wait else 0)

    def start(self):
        """Start receiving messages to a queue."""

        # Connect
        self.sock.connect((self.ip, self.port))
        self.sock.settimeout(None)

        # Start receiving
        thread = threading.Thread(target=self._always_receive)
        thread.daemon = True
        thread.start()
        self.receiving_thread = thread

    def _always_receive(self):
        """Continuously receive data; internal use."""

        while True:

            # Receive a complete message
            chunks = []
            remaining_bytes = self.MSG_LENGTH
            while remaining_bytes > 0:

                # Read
                chunk = self.sock.recv(min(remaining_bytes, 2048))
                if chunk == b"":
                    print("Closed", file=sys.stderr)
                    self._data_queue.put(None, block=True)  # Signal EOT
                    return
                chunks.append(chunk)

                remaining_bytes -= len(chunk)

            # Return
            msg = b"".join(chunks)
            self._data_queue.put(msg, block=True)

    def receive(self, wait=False):
        """Return a message received.

        :param wait: if true, waits until a complete message is received
        :return: a bytes object containing the message, or None if there are
            no new messages
        :raises: IOError at the end of transmission
        """

        if not wait and self._data_queue.empty():
            return None

        # note: this is safe, because i must be the only consumer
        msg = self._data_queue.get(block=wait, timeout=None)

        if msg is None:
            raise IOError("End Of Transmission")

        return msg
