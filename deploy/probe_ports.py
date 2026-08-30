# 严谨公网端口探测: 区分 放行/拒绝/被墙(超时)
import socket, sys

def probe(host, port, timeout=4):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return "OPEN"
    except socket.timeout:
        return "FILTERED(timeout)"
    except ConnectionRefusedError:
        return "REFUSED"
    except OSError as e:
        return f"ERR({e.errno})"
    finally:
        s.close()

host = sys.argv[1] if len(sys.argv) > 1 else "8.148.79.85"
ports = [22, 80, 443, 8000, 8001, 8500, 9000, 18000, 8080, 31234, 28500]
for p in ports:
    print(f"{p}: {probe(host, p)}")