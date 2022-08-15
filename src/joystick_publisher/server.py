import socket

# choose some properties of the server:
ip_address = "192.168.2.69"
port = 20001
buffer_size = 1024

msg_from_server = "Hello from your server!"
bytes_to_send = str.encode(msg_from_server)

# create the socket:
udp_server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# bind the socket to the address and port:
udp_server_socket.bind((ip_address, port))

# We are now up and listening!
print("UDP server up and listening")

# Set up the loop to listen on this address:
while True:
    # receieve from the socket:
    bytes_address_pair = udp_server_socket.recvfrom(buffer_size)
    msg = bytes_address_pair[0]
    addr = bytes_address_pair[1]

    # The msg that we rx, and the address:
    client_msg = f"Msg from client: {msg}"
    client_ip = f"IP address of the client {addr}"

    print(client_msg)
    print(client_ip)

    # reply our msg to the address that contacted us:
    udp_server_socket.sendto(bytes_to_send, addr)
