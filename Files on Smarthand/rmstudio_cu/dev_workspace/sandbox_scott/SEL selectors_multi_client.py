# https://pymotw.com/3/selectors/
import selectors
import socket


def name_1_read_callback(connection, mask, name):
    data = connection.recv(1024)
    if data:
        print("  received {!r}".format(data))


def name_1_write_callback(connection, mask, name):
    connection.sendall(b"hi")


name_1_server_address = ("localhost", 9001)


def name_2_read_callback(connection, mask, name):
    data = connection.recv(1024)
    if data:
        print("  received {!r}".format(data))


def name_2_write_callback(connection, mask, name):
    print("  ready to write")
    outgoing = [
        b"It will be repeated.",
        b"This is the message.  ",
    ]
    while outgoing:
        # Send the next message.
        next_msg = outgoing.pop()
        print("  sending {!r}".format(next_msg))
        connection.sendall(next_msg)


name_2_server_address = ("localhost", 9002)

EXAMPLE_CLIENT_DATA = {
    "name_1": (name_1_read_callback, name_1_write_callback, name_1_server_address),
    "name_2": (name_2_read_callback, name_2_write_callback, name_2_server_address),
}


class SelectorsMultiClient:
    def __init__(self, client_data=None):
        print("__init__()")
        self.running = False
        self.selector = selectors.DefaultSelector()
        self.socks = {}

        if client_data is None:
            client_data = EXAMPLE_CLIENT_DATA
        self.client_data = client_data

    def __enter__(self):
        print("__enter__()")
        self.running = True
        for name in self.client_data:
            read_callback, write_callback, server_address = self.client_data[name]
            # Connecting is a blocking operation, so call setblocking()
            # after it returns.
            print("connecting to {} port {}".format(*server_address))
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(server_address)
            sock.setblocking(False)
            # Set up the selector to watch for when the socket is ready
            # to send data as well as when there is data to read.
            self.selector.register(
                sock,
                selectors.EVENT_READ | selectors.EVENT_WRITE,
                (read_callback, write_callback, name),
            )
            self.socks[name] = sock
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("__exit__()")
        print(f"exc_type: {exc_type} \n, exc_val: {exc_val} \n, exc_tb: {exc_tb} \n")
        print("shutting down")
        self.selector.close()
        print("shutting down complete")
        self.running = False

    def run(self):
        while self.running:
            print(f"{self} waiting for I/O")
            for key, mask in self.selector.select(timeout=1):  # 1 second timeout
                read_callback, write_callback, name = key.data
                if mask & selectors.EVENT_READ:
                    read_callback(key.fileobj, mask, name)
                if mask & selectors.EVENT_WRITE:
                    write_callback(key.fileobj, mask, name)


def main():
    print("testing SelectorsMultiClient")
    with SelectorsMultiClient() as client:
        print(f"client: {client}")
        while client.running:
            client.run()


if __name__ == "__main__":
    main()
