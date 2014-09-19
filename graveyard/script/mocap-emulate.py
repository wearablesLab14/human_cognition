from sys import argv,exit
from subprocess import call
from socket import socket,AF_INET,SOCK_DGRAM,SOL_SOCKET,SO_BROADCAST, SO_REUSEADDR
import atexit

PREFIX = "192.168.128"
PORT   = 5050

if len(argv) < 2:
    print ("supply text file(s) containing emulated data")
    exit(-1)

data = [ open(f,"r").readlines() for f in argv[1:] ]
addr = [ ("eth0:%d"%i, PREFIX+".%d"%i) for i in range(2,2+len(data)) ]

for interface,ip in addr:
    call('sudo ip addr add %s/24 dev eth0 label %s'%(ip,interface),shell=True)
    print('sudo ip addr add %s/24 dev eth0 label %s'%(ip,interface))

def teardown(reason):
    call('sudo ip addr del %s.2/24 dev eth0'%PREFIX, shell=True)
atexit.register(teardown, 'teardown')

socks = [ socket(AF_INET, SOCK_DGRAM) for i in range(len(data)) ]
for sock,(interface,ip) in zip(socks,addr):
    sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    sock.bind( (ip, PORT) )

n = max( [ len(d) for d in data ] )
for i in range(n):
    for sock,source in zip(socks,data):
        try:
            buf = bytes(source[i])
            sock.sendto(buf,('<broadcast>',PORT))
        except IndexError as e:
            pass
