from socket import socket,AF_INET,SOCK_DGRAM,SOL_SOCKET,SO_BROADCAST, SO_REUSEADDR

PORT = 5050

sock = socket(AF_INET, SOCK_DGRAM)
sock.bind( ('', PORT) )
sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1) # <- wichtig für den test!

while True:
    msg,(ip,port) = sock.recvfrom(1024)
    print (ip,msg)
