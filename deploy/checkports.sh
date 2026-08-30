#!/bin/bash
for p in 8500 9000 18000 28500 31234 8002 8088; do
  dnat=$(iptables -t nat -L PREROUTING -n 2>/dev/null | grep -c "dpt:${p} ")
  lst=$(ss -ltn | grep -c ":${p} ")
  dpr=$(docker ps --format '{{.Ports}}' 2>/dev/null | grep -c "${p}")
  echo "port=${p} dnat=${dnat} local=${lst} docker=${dpr}"
done