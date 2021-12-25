import selectors
import socket
import multiprocessing as mp
from collections import defaultdict

name_1_server_address = ("localhost", 9001)
name_2_server_address = ("localhost", 9002)


def get_message(queue, timeout):
    try:
        msg = queue.get(block=True, timeout=timeout)
        return msg
    except Exception as e:
        print(e)
        return None


def server_write_callback(key, server, name):
    write_q = server.connection_queues[name]
    msg = get_message(write_q, timeout=2)
    if msg is not None:
        connection = key.fileobj
        connection.sendall(msg)


def name_1_read_callback(key, server, name):
    connection = key.fileobj
    data = connection.recv(server.read_bytes)
    connection.sendall(data + b" name_1\n")


def name_2_read_callback(key, server, name):
    connection = key.fileobj
    data = connection.recv(server.read_bytes)
    connection.sendall(data + b" name_2\n")


EXAMPLE_SERVER_DATA = {
    "name_1": (name_1_read_callback, server_write_callback, name_1_server_address),
    "name_2": (name_2_read_callback, server_write_callback, name_2_server_address),
}


class SelectorsMultiServer:
    def __init__(self, input_q=None, output_q=None, server_data=None):
        print("__init__()")
        self.running = False
        self.read_bytes = 1024
        self.selector = selectors.DefaultSelector()
        self.socks = {}
        self.connections = defaultdict(list)
        self.connection_queues = {}
        self.select_timeout = 5  # seconds

        if server_data is None:
            server_data = EXAMPLE_SERVER_DATA
        self.server_data = server_data

        if input_q is None:
            input_q = mp.Queue()
        self.input_q = input_q

        if output_q is None:
            output_q = mp.Queue()
        self.output_q = output_q

    def read(self, key, server, name):
        callback, _server_address = self.server_data[name]
        callback(key, self)

    def accept(self, key, server, name):
        "Callback for new connections"
        connection, addr = self.socks[name].accept()
        self.connections[name].append(connection)

        print(f"accept({addr}), name: {name}")
        connection.setblocking(False)
        read_callback, write_callback, _server_address = self.server_data[name]
        self.selector.register(
            connection,
            selectors.EVENT_READ | selectors.EVENT_WRITE,
            (read_callback, write_callback, name),
        )

    def __enter__(self):
        print("__enter__()")
        self.running = True
        for name in self.server_data:
            _, _, server_address = self.server_data[name]
            print("starting up on {} port {}".format(*server_address))
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            sock.bind(server_address)
            sock.listen(100)
            self.selector.register(
                sock, selectors.EVENT_READ, (self.accept, None, name)
            )
            self.socks[name] = sock
            self.connection_queues[name] = mp.Queue()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("__exit__()")
        print(f"exc_type: {exc_type} \n, exc_val: {exc_val} \n, exc_tb: {exc_tb} \n")
        print("shutting down")
        # Shut down and close connections
        for name in self.connections:
            connections = self.connections[name]
            for connection in connections:
                connection.close()
        # Shut down and close sockets
        for name in self.socks:
            sock = self.socks[name]
            sock.close()
        # Close selector
        self.selector.close()
        print("shutting down complete")
        self.running = False

    def run(self):
        while self.running:
            print(f"{self} waiting for I/O")
            for key, mask in self.selector.select(timeout=self.select_timeout):
                read_callback, write_callback, name = key.data
                if mask & selectors.EVENT_READ:
                    read_callback(key, self, name)
                if mask & selectors.EVENT_WRITE:
                    write_callback(key, self, name)


def main():
    print("testing SelectorsMultiServer")

    with SelectorsMultiServer() as server:
        print(f"server: {server}")
        server.run()


if __name__ == "__main__":
    main()
