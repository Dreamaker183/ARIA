#!/bin/bash

echo "=== ROBOT COMMUNICATION DEBUGGER ==="
echo "Testing connection to Pioneer 3DX via ZLAN 7104"
echo

ROBOT_IP="192.168.1.152"
ROBOT_PORT="5000"

echo "🔍 Step 1: Testing basic TCP connection..."
if nc -z -w 2 $ROBOT_IP $ROBOT_PORT; then
    echo "✅ TCP connection successful"
else
    echo "❌ TCP connection failed"
    exit 1
fi

echo
echo "🔍 Step 2: Testing if robot responds to data..."
echo "Sending test commands..."

# Test 1: Send basic commands
echo "Test 1: Basic commands"
(echo -e "SYNC0\r\n"; sleep 1; echo -e "SYNC1\r\n"; sleep 1; echo -e "SYNC2\r\n"; sleep 1) | nc -w 3 $ROBOT_IP $ROBOT_PORT > /tmp/robot_response.txt 2>&1

if [ -s /tmp/robot_response.txt ]; then
    echo "✅ Robot responded:"
    cat /tmp/robot_response.txt | hexdump -C
else
    echo "❌ No response from robot"
fi

echo
echo "Test 2: Try ARIA protocol sync"
# Send ARIA sync packets (binary)
printf "\xFA\xFB\x03\x00\x00\x00\x02\x00" | nc -w 2 $ROBOT_IP $ROBOT_PORT > /tmp/robot_response2.txt 2>&1

if [ -s /tmp/robot_response2.txt ]; then
    echo "✅ Robot responded to ARIA sync:"
    cat /tmp/robot_response2.txt | hexdump -C
else
    echo "❌ No response to ARIA sync"
fi

echo
echo "🔍 Step 3: Testing with telnet-style connection..."
echo "Connecting with telnet (press Ctrl+C to exit)..."
echo "Try typing: SYNC, STATUS, HELLO"
echo
telnet $ROBOT_IP $ROBOT_PORT

echo
echo "=== DEBUG COMPLETE ==="
echo
echo "📋 TROUBLESHOOTING CHECKLIST:"
echo "1. ✅ Check robot power (LED indicators, sounds)"
echo "2. ✅ Check serial cable connection (ZLAN to robot)"
echo "3. ✅ Check robot mode switches (should be REMOTE/AUTO)"
echo "4. ✅ Try power cycling the robot"
echo "5. ✅ Check ZLAN 7104 configuration (baud rate: 9600)"
echo
echo "If robot still doesn't respond:"
echo "- Robot may be in wrong mode"
echo "- Serial cable may be faulty"
echo "- Robot may need different baud rate"
echo "- Robot may need to be 'woken up' with specific command"
