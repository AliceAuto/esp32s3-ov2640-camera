# 中继服务端连通/逻辑自检客户端(在服务器本机跑)
import socket, json, sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8001

def rl(exact):
    d = s.recv(exact)
    if not d:
        raise EOFError("EOF while recv")
    return d

s = socket.socket()
s.settimeout(6)
s.connect((HOST, PORT))
print("connect-ok", HOST, PORT)
# list
b = json.dumps({"t": "list"}).encode()
s.sendall(len(b).to_bytes(4, "big") + b)
try:
    while True:
        got = rl(4096)
        print("recv", len(got), got[:300])
except (socket.timeout,) as e:
    print("TIMEOUT-no-more")
except EOFError as e:
    print("EOF", e)
s.close()