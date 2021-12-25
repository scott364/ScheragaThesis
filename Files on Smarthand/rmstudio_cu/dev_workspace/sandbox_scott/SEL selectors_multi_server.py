import selectors
import socket


def name_1_read_callback(connection, data):
    connection.sendall(data + b" name_1\n")


name_1_server_address = ("localhost", 9001)


def name_2_read_callback(connection, data):
    connection.sendall(data + b" name_2\n")


name_2_server_address = ("localhost", 9002)

EXAMPLE_SERVER_DATA = {
    "name_1": (name_1_read_callback, name_1_server_address),
    "name_2": (name_2_read_callback, name_2_server_address),
}


class SelectorsMultiServer:
    def __init__(self, server_data=None):
        print("__init__()")
        self.running = False
        self.read_bytes = 1024
        self.selector = selectors.DefaultSelector()
        self.socks = {}

        if server_data is None:
            server_data = EXAMPLE_SERVER_DATA
        self.server_data = server_data

    def read(self, connection, mask, name):
        client_address = connection.getpeername()
        print("read({}), name: {}".format(client_address, name))
        data = connection.recv(self.read_bytes)
        print("  received {!r}".format(data))
        callback, _server_address = self.server_data[name]
        callback(connection, data)

    def accept(self, connection, mask, name):
        "Callback for new connections"
        connection, addr = self.socks[name].accept()
        print("accept({}), name: {}".format(addr, name))
        connection.setblocking(False)
        self.selector.register(connection, selectors.EVENT_READ, (self.read, name))

    def __enter__(self):
        print("__enter__()")
        self.running = True
        for name in self.server_data:
            _read_callback, server_address = self.server_data[name]
            print("starting up on {} port {}".format(*server_address))
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            sock.bind(server_address)
            sock.listen(5)
            self.selector.register(sock, selectors.EVENT_READ, (self.accept, name))
            self.socks[name] = sock
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("__exit__()")
        print("shutting down")
        self.selector.close()
        print("shutting down complete")
        self.running = False

    def run(self):
        while self.running:
            print("{} waiting for I/O".format(self))
            for key, mask in self.selector.select(timeout=1):
                callback, name = key.data
                callback(key.fileobj, mask, name)


def main():
    print("testing SelectorsMultiServer")

    with SelectorsMultiServer() as server:
        print("server: {server}".format(server=server))
        server.run()


if __name__ == "__main__":
    main()
