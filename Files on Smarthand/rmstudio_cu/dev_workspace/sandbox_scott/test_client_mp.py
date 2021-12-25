import multiprocessing as mp


from selectors_multi_client import SelectorsMultiClient, get_message


name_1_server_address = ("localhost", 9001)
name_2_server_address = ("localhost", 9002)


def name_1_read_callback(key, client, name):
    connection = key.fileobj
    data = connection.recv(1024)
    if data:
        print("  received {!r}".format(data))


def name_1_write_callback(key, client, name):
    connection = key.fileobj
    msg = get_message(client.input_q, timeout=2)
    if msg is not None:
        print(msg)
        connection.sendall(b"START")


def name_2_read_callback(key, client, name):
    connection = key.fileobj
    data = connection.recv(1024)
    cross_message = b"Message initated by name_1, sent to name_2"
    if data:
        print("  received {!r}".format(data))
        if data == cross_message:
            connection.sendall(b"STOP")
            client.running = False


def name_2_write_callback(key, client, name):
    pass


client_data = {
    "name_1": (name_1_read_callback, name_1_write_callback, name_1_server_address),
    "name_2": (name_2_read_callback, name_2_write_callback, name_2_server_address),
}


def main():
    print("testing SelectorsMultiClient with multiprocessing")
    # TODO: write to the input queue using a different process
    # TODO: message acknowledgements and message type scheme
    # for sockets and mp.Queues

    input_q = mp.Queue()
    input_q.put("MSG: start")
    output_q = mp.Queue()

    selectors_args = {
        "input_q": input_q,
        "output_q": output_q,
        "client_data": client_data,
    }

    with SelectorsMultiClient(**selectors_args) as client:
        client.run()


if __name__ == "__main__":
    main()
