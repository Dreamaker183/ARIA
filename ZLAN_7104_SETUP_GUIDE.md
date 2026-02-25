# ZLAN 7104 WiFi Module Setup Guide
## Pioneer 3DX Robot Wireless Connection

### 📋 **Your Network Configuration**
- **ZLAN 7104 IP Address:** `192.168.1.152`
- **Router Gateway:** `192.168.1.254`
- **WiFi MAC Address:** `62:9a:b7:03:cb:4e`
- **Default Port:** `8101`

---

## 🔧 **Step 1: ZLAN 7104 Initial Configuration**

### **Physical Connection:**
1. **Power off** your Pioneer 3DX robot
2. **Connect ZLAN 7104** to the robot's serial port (usually DB9 connector)
3. **Power on** the robot - ZLAN module should get power from robot

### **ZLAN 7104 Configuration:**
The ZLAN 7104 needs to be configured to:
- Connect to your WiFi network
- Use IP address `192.168.1.152`
- Forward serial data to TCP port `8101`

**Configuration Methods:**
1. **Web Interface:** Connect to ZLAN's default IP (usually `192.168.4.1` in AP mode)
2. **Serial Configuration:** Use AT commands via serial connection
3. **Configuration Software:** Use ZLAN's Windows configuration tool

---

## 🌐 **Step 2: WiFi Network Connection**

### **Make sure your computer and robot are on the same network:**

1. **Check your computer's network:**
```bash
# Check your IP address
ifconfig | grep "inet " | grep -v 127.0.0.1

# Ping the router to confirm connectivity
ping 192.168.1.254

# Ping the ZLAN module
ping 192.168.1.152
```

2. **Verify ZLAN 7104 is connected:**
```bash
# Test if ZLAN module is responding
telnet 192.168.1.152 8101
# (Press Ctrl+] then 'quit' to exit)
```

---

## 🚀 **Step 3: Run the Consensual Movements Program**

### **🔌 WHEN CONNECTED TO ZLAN WIFI (NO INTERNET ACCESS):**

**Since you can ping 192.168.1.152, you're connected! Follow these exact steps:**

#### **Step 3A: Navigate to ARIA Directory**
```bash
cd /Users/ansonchan/Pokeguide/ARIA
```

#### **Step 3B: Test Connection First**
```bash
# Verify you can reach the robot
ping -c 3 192.168.1.152

# Test if port 8101 is open
nc -zv 192.168.1.152 8101
# OR try telnet test:
telnet 192.168.1.152 8101
# (Press Ctrl+] then type 'quit' to exit)
```

#### **Step 3C: Start the Robot Program**
```bash
# Method 1: Direct connection (RECOMMENDED when you can ping)
./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 8101

# Method 2: Auto-discovery (will find your IP automatically)
./examples/consensualMovements -wifi
```

#### **Step 3D: Enable Robot Movement**
1. **Wait for connection message:** "Successfully connected to robot: CITYUN.HK_3871"
2. **Wait for prompt:** "Press SPACE to start movement"
3. **Press SPACE key** to enable autonomous navigation
4. **Robot will start moving!** 🤖

#### **Step 3E: Monitor and Control**
- **Watch the status:** Position, battery, sonar readings
- **Emergency stop:** Press Ctrl+C to stop safely
- **Normal operation:** Robot navigates autonomously using consensual movements algorithm

### **🌐 WHEN YOU HAVE INTERNET ACCESS:**

### **Method 1: Auto-Discovery (Recommended)**
```bash
cd /Users/ansonchan/Pokeguide/ARIA
./examples/consensualMovements -wifi
```

### **Method 2: Direct IP Connection**
```bash
./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 8101
```

### **Method 3: Manual IP Entry**
```bash
./examples/consensualMovements -wifi
# When prompted, enter: 192.168.1.152
```

---

## 🎮 **Step 4: Operating the Robot**

1. **Wait for connection confirmation**
2. **Press SPACE** when prompted to enable movement
3. **Robot will start autonomous navigation**
4. **Press Ctrl+C** to stop safely

---

## 🔍 **Troubleshooting**

### **✅ CONNECTED TO ZLAN WIFI (You can ping 192.168.1.152):**

**Problem:** Program says "Could not connect to robot"
```bash
# Test if robot port is open
nc -zv 192.168.1.152 8101
# OR
telnet 192.168.1.152 8101
```

**If port test fails:**
- Check robot is powered ON
- Verify ZLAN 7104 has power (LED indicators)
- Check serial cable between ZLAN and robot
- Try different port: 23, 8080, or 9999

**If port test works but robot doesn't respond:**
- Robot might be in wrong mode
- Check baud rate settings (9600 vs 38400)
- Try power cycling the robot
- Check robot's internal serial settings

**Problem:** Robot connects but doesn't move
- Make sure to press SPACE when prompted
- Check robot's motor enable switch
- Verify robot batteries are charged
- Look for error messages in the output

### **❌ GENERAL CONNECTION Issues:**

**Problem:** Can't ping 192.168.1.152
```bash
# Check if ZLAN is on the network
nmap -sn 192.168.1.0/24 | grep -B2 -A2 "62:9a:b7:03:cb:4e"
```

**Problem:** Connection timeout
- Verify ZLAN 7104 is powered on
- Check WiFi network connectivity
- Confirm IP address hasn't changed
- Try different port (some use 23 or 8080)

**Problem:** Robot doesn't respond
- Check serial connection between ZLAN and robot
- Verify baud rate settings (usually 9600 or 38400)
- Ensure robot is powered on and ready

### **Network Diagnostics:**
```bash
# Scan for the ZLAN module
nmap -p 8101 192.168.1.152

# Check network connectivity
traceroute 192.168.1.152

# Monitor network traffic
sudo tcpdump -i any host 192.168.1.152
```

---

## ⚙️ **ZLAN 7104 Configuration Reference**

### **Typical Settings:**
- **WiFi SSID:** Your network name
- **WiFi Password:** Your network password
- **IP Mode:** Static IP
- **IP Address:** 192.168.1.152
- **Subnet Mask:** 255.255.255.0
- **Gateway:** 192.168.1.254
- **DNS:** 8.8.8.8
- **Serial Settings:** 9600 baud, 8N1 (match robot)
- **TCP Server Port:** 8101
- **Protocol:** TCP Server mode

### **AT Commands (if using serial config):**
```
AT+WMODE=STA                    # Station mode
AT+WSSSID=YourWiFiName         # Set SSID
AT+WSKEY=WPA2,YourPassword     # Set password
AT+WANN=STATIC,192.168.1.152,255.255.255.0,192.168.1.254
AT+TCPSERVER=8101              # TCP server on port 8101
AT+UART=9600,8,1,NONE,NFC      # Serial settings
AT+SAVE                        # Save configuration
AT+REBOOT                      # Restart module
```

---

## 🎯 **Quick Commands for ZLAN WiFi Connection (NO INTERNET)**

**You're connected to ZLAN WiFi and can ping 192.168.1.152 - Use these commands:**

```bash
# Step 1: Go to ARIA directory
cd /Users/ansonchan/Pokeguide/ARIA

# Step 2: Test connectivity
ping -c 3 192.168.1.152

# Step 3: Test robot port
nc -zv 192.168.1.152 8101

# Step 4: Start robot program (MAIN COMMAND)
./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 8101

# Step 5: When prompted, press SPACE to start movement
```

## 🎯 **Alternative Quick Commands (WITH INTERNET)**

```bash
# Test 1: Basic connectivity
ping -c 3 192.168.1.152

# Test 2: Port connectivity  
nc -zv 192.168.1.152 8101

# Test 3: WiFi auto-discovery
./examples/consensualMovements -wifi

# Test 4: Direct connection
./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 8101
```

---

## 📞 **Support Information**

If you need help:
1. Check ZLAN 7104 status LEDs
2. Verify network settings match this guide
3. Test with a simple telnet connection first
4. Use the diagnostic commands above

**Your specific configuration:**
- IP: `192.168.1.152:8101`
- MAC: `62:9a:b7:03:cb:4e`
- Gateway: `192.168.1.254`
