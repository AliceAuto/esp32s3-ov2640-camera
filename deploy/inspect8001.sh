#!/bin/bash
echo "=== containers (all states) with 8001 in ports ==="
docker ps -a --format '{{.Names}} | {{.Status}} | {{.Ports}}' | grep -Ei '8001|172\.17\.0\.2' || echo none-8001-container
echo
echo "=== who is 172.17.0.2 (via docker) ==="
docker network inspect bridge --format '{{range .Containers}}{{.Name}} {{.IPv4Address}}{{println}}{{end}}' 2>/dev/null
echo
echo "=== iptables entries mentioning 8001 (nat) ==="
iptables -t nat -L -n | grep -n "8001" || echo no-nat-8001
echo
echo "=== iptables entries mentioning 8001 (filter) ==="
iptables -L -n | grep -n "8001" || echo no-filter-8001
echo
echo "=== tcp connect to 172.17.0.2:8001 (after dnats DNS) ==="
timeout 3 bash -c "exec 3<>/dev/tcp/172.17.0.2/8001 && head -c 64 <&3 && echo || echo FAIL_CONNECT" 2>&1 || echo CONNECT_FAIL