#!/bin/bash

# ZLAN 7104 Port Scanner
# Tests for available TCP ports and services

ZLAN_IP="192.168.1.152"

echo "=== ZLAN 7104 PORT SCANNER ==="
echo "Testing IP: $ZLAN_IP"
echo "Scanning for available TCP ports..."
echo

# Test ping first
echo "🔍 Testing basic connectivity..."
if ping -c 3 $ZLAN_IP > /dev/null 2>&1; then
    echo "✅ Ping successful - ZLAN is reachable"
else
    echo "❌ Ping failed - ZLAN not reachable"
    exit 1
fi
echo

# Common ZLAN/Serial-to-WiFi ports
echo "🔍 Testing common ZLAN ports..."
COMMON_PORTS=(23 80 443 502 1001 2001 4001 8080 8081 8101 8899 9999 10001 10002)

for port in "${COMMON_PORTS[@]}"; do
    echo -n "Port $port: "
    if nc -z -w 2 $ZLAN_IP $port 2>/dev/null; then
        echo "✅ OPEN"
        
        # Try to get banner/response
        echo "   Testing service response..."
        response=$(echo -e "\r\n" | nc -w 2 $ZLAN_IP $port 2>/dev/null | head -1)
        if [ ! -z "$response" ]; then
            echo "   Response: $response"
        else
            echo "   No banner response"
        fi
        
        # Test HTTP if it's a web port
        if [[ $port == 80 || $port == 8080 || $port == 8081 ]]; then
            echo "   Testing HTTP..."
            http_response=$(curl -m 3 -s http://$ZLAN_IP:$port | head -1)
            if [ ! -z "$http_response" ]; then
                echo "   HTTP Response: $http_response"
            fi
        fi
        echo
    else
        echo "❌ Closed"
    fi
done

echo
echo "🔍 Quick scan of other common ports..."
OTHER_PORTS=(21 22 25 53 110 143 993 995 1883 5000 5001 8000 8888)

open_ports=()
for port in "${OTHER_PORTS[@]}"; do
    if nc -z -w 1 $ZLAN_IP $port 2>/dev/null; then
        open_ports+=($port)
    fi
done

if [ ${#open_ports[@]} -gt 0 ]; then
    echo "✅ Additional open ports found: ${open_ports[*]}"
    
    # Test each additional open port
    for port in "${open_ports[@]}"; do
        echo "Testing port $port..."
        response=$(echo -e "\r\n" | nc -w 2 $ZLAN_IP $port 2>/dev/null | head -1)
        if [ ! -z "$response" ]; then
            echo "   Response: $response"
        fi
    done
else
    echo "❌ No additional open ports found"
fi

echo
echo "🔍 Testing UDP services (common ZLAN UDP ports)..."
UDP_PORTS=(53 123 161 502 1001)

for port in "${UDP_PORTS[@]}"; do
    echo -n "UDP Port $port: "
    # UDP test is less reliable, but we can try
    if nc -u -z -w 1 $ZLAN_IP $port 2>/dev/null; then
        echo "✅ Responding"
    else
        echo "❌ No response"
    fi
done

echo
echo "=== SCAN COMPLETE ==="
echo
echo "📋 SUMMARY:"
echo "If NO TCP ports are open:"
echo "  → ZLAN needs configuration (reset and setup via web interface)"
echo
echo "If ports ARE open:"
echo "  → Try connecting with: ./examples/consensualMovements -remoteHost $ZLAN_IP -remoteRobotTcpPort [OPEN_PORT]"
echo
echo "Common ZLAN configurations:"
echo "  → Port 23: Telnet mode"
echo "  → Port 8101: TCP server mode (preferred for robots)"
echo "  → Port 80/8080: Web configuration interface"
