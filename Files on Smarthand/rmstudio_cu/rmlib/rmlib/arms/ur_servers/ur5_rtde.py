import os
import sys
import rtde
import rtde_config
import netifaces as ni
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ("/RPC2",)

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    # Add these headers to all responses
    def end_headers(self):
        self.send_header(
            "Access-Control-Allow-Headers",
            "Origin, X-Requested-With, Content-Type, Accept",
        )
        self.send_header("Access-Control-Allow-Origin", "*")
        SimpleXMLRPCRequestHandler.end_headers(self)


def setup_rtde_proxy(robot_arm_ip):
    # Get rmlib path
    full_path = os.path.realpath(__file__)
    ur_servers_path = os.path.dirname(full_path) + "/"

    conf = rtde_config.ConfigFile(ur_servers_path + "ur5_interface_configuration.xml")
    output_names, output_types = conf.get_recipe("out")
    rtde_proxy = rtde.RTDE(robot_arm_ip, 30004)
    rtde_proxy.connect()

    if not rtde_proxy.send_output_setup(output_names, output_types):
        raise Exception("wrong config file")
    else:
        rtde_proxy.send_start()
        return rtde_proxy


def main():
    # Set arm ip address as parameter 1
    print("ip address", sys.argv[1])
    robot_arm_ip = sys.argv[1]

    # Set xmlrpc port number as parameter 2
    print("port number", sys.argv[2])
    port_number = int(sys.argv[2])

    # Find ip address of gripper
    robot_proxy_ip = ni.ifaddresses("eth0")[ni.AF_INET][0]["addr"]

    rtde_proxy = setup_rtde_proxy(robot_arm_ip)

    def get_state():
        state = rtde_proxy.receive_last_buffered()
        return state

    with SimpleXMLRPCServer(
        (robot_proxy_ip, port_number), requestHandler=RequestHandler, allow_none=True
    ) as server:
        server.register_function(get_state, "get_state")

        print("UR RTDE Server Running")
        server.serve_forever()


if __name__ == "__main__":
    main()
