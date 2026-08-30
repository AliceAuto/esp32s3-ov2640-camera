#!/bin/bash
# 备份当前 nat/filter 规则到文件(可回滚)
mkdir -p /opt/ov2640-relay/backup
iptables-save > /opt/ov2640-relay/backup/iptables.bak.$(date +%F_%T).txt
echo "backup written"
ls -1 /opt/ov2640-relay/backup/

# 删除 8001 的陈旧 docker 残留规则(全部指向已死的 sillytavern:8001)
echo "--- deleting stale 8001 rules (errors mean rule already gone = OK) ---"
iptables -t nat -D PREROUTING -p tcp -d 8.148.79.85 --dport 8001 -j DNAT --to-destination 172.17.0.2:8001 2>&1
iptables -t nat -D PREROUTING -p tcp --dport 8001 -j DNAT --to-destination 172.17.0.2:8001 2>&1
iptables -t nat -D POSTROUTING -p tcp -d 172.17.0.2 --dport 8001 -j MASQUERADE 2>&1
iptables -D FORWARD -p tcp -d 172.17.0.2 --dport 8001 -j ACCEPT 2>&1

echo "--- remaining 8001 references ---"
iptables -t nat -L -n | grep "8001" || echo "nat-clean"
iptables -L -n | grep "8001" || echo "filter-clean"