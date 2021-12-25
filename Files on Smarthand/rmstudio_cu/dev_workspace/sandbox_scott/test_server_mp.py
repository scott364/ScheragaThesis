import multiprocessing as mp

from selectors_multi_server import (
    SelectorsMultiServer,
    server_write_callback,
    get_message,
)


def name_2_read_callback(key, server, name):
    connection = key.fileobj
    data = connection.recv(server.read_bytes)
    print(data)
    if data and data == b"STOP":
        server.running = False


def name_1_read_callback(key, server, name):
    connection = key.fileobj
    data = connection.recv(server.read_bytes)
    connection.sendall(data + b" name_1\n")
    # send cross connection using a queue
    cross_message = b"Message initated by name_1, sent to name_2"
    server.connection_queues["name_2"].put(cross_message)


def main():
    print("testing SelectorsMultiServer with multiprocessing")
    name_1_server_address = ("localhost", 9001)
    name_2_server_address = ("localhost", 9002)

    input_q = mp.Queue()
    output_q = mp.Queue()

    server_data = {
        "name_1": (name_1_read_callback, server_write_callback, name_1_server_address),
        "name_2": (name_2_read_callback, server_write_callback, name_2_server_address),
    }

    selectors_args = {
        "input_q": input_q,
        "output_q": output_q,
        "server_data": server_data,
    }

    with SelectorsMultiServer(**selectors_args) as server:
        server.run()


if __name__ == "__main__":
    main()
