import socket

msg_from_client = "Hello UDP Server!"
bytes_to_send = str.encode(msg_from_client)
server_address_port = ("192.168.2.69", 20001)
buffer_size = 1024

# create our socket:
udp_client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# send to the server:
udp_client_socket.sendto(bytes_to_send, server_address_port)

# wait for a reply:
msg_from_server = udp_client_socket.recvfrom(buffer_size)
msg = f"Msg from server: {msg_from_server[0]}"
print(msg)