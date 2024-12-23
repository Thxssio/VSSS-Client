import socket

from .communication import communication


class UDPCommunication(communication):
    def __init__(self, ip="127.0.0.1", port=None):
        self.ip = ip
        self.port = port
        self.sock = None
        if self.port is not None:
            self.open_socket()

    def open_socket(self):
        """Open a new UDP socket."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def send_command(self, command):
        """Send a command to the predefined IP and port."""
        if self.sock and self.port is not None:
            self.sock.sendto(command, (self.ip, self.port))
        else:
            raise ValueError("Socket is not initialized or port is not set.")

    def update_port(self, new_port):
        """Update the UDP port and recreate the socket if necessary."""
        if new_port != self.port:
            self.port = new_port
            if self.sock:
                self.sock.close()
            self.open_socket()
            print(f"UDP port updated to {self.port}")