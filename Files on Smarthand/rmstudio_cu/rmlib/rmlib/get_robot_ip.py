from scapy.all import srp, Ether, ARP

def get_ip(mac_addy, subnet_mask, timeout=2):
    ans, unans = srp(Ether(dst=mac_addy) / ARP(pdst=subnet_mask), timeout=timeout)
    return ans


def main():
    subnet_mask = "128.138.224.0/24"
    mac_addresses = {"smarthand": "00:04:4B:A5:36:00", "eSeries": "D4:3D:7E:B7:0E:BB"}
    output = {}
    for name, addy in mac_addresses.items():
        ans = get_ip(addy, subnet_mask)
        if ans: # empty list returns false
            first_query_answer = ans[0]
            output[name] = first_query_answer.query.pdst # destination ip for an answered query
    print(output)

if __name__ == "__main__":
    main()
