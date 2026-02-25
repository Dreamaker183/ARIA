# 🔧 ZLAN 7104 Diagnostic Report

## ✅ **Current Status:**
- **Ping Test:** ✅ SUCCESS - ZLAN responds to ping at 192.168.1.152
- **Network Connection:** ✅ CONNECTED - ZLAN is on WiFi network
- **TCP Ports:** ❌ FAILED - No ports are open (22, 23, 80, 8080, 8101, 9999, 10001)

## 🚨 **Problem Identified:**
**ZLAN 7104 is not configured for TCP server mode or robot is not connected**

---

## 🔧 **SOLUTION STEPS:**

### **Step 1: Check Physical Connections**
1. **Verify ZLAN 7104 is connected to robot's serial port**
2. **Check power LEDs on ZLAN module**
3. **Ensure robot is powered ON**
4. **Verify serial cable is secure**

### **Step 2: Access ZLAN Configuration**

#### **Method A: Web Interface (if available)**
Try accessing ZLAN's web interface:
```bash
# Try common web interface addresses
open http://192.168.1.152
open http://192.168.1.152:80
open http://192.168.1.152:8080
```

#### **Method B: Try Default AP Mode**
1. **Reset ZLAN 7104** (hold reset button 10+ seconds)
2. **Look for WiFi network** named something like "ZLAN_xxxx" or "USR-WIFI232-xxxx"
3. **Connect to that network** (password might be "12345678")
4. **Access web interface** at http://192.168.4.1

#### **Method C: Serial Configuration**
If you have a USB-to-serial adapter:
1. **Connect directly to ZLAN's serial pins**
2. **Use terminal program** (screen, minicom, or Arduino IDE)
3. **Send AT commands** to configure

### **Step 3: Required ZLAN Configuration**

**Essential Settings:**
```
Work Mode: TCP Server
Local Port: 8101
Baud Rate: 9600 (match robot)
Data Bits: 8
Stop Bits: 1
Parity: None
Flow Control: None
```

**Network Settings:**
```
WiFi Mode: Station (STA)
SSID: [Your WiFi Network]
Password: [Your WiFi Password]
IP Mode: Static
IP Address: 192.168.1.152
Subnet: 255.255.255.0
Gateway: 192.168.1.254
```

### **Step 4: AT Commands (if using serial)**
```
AT+WMODE=STA                    # Station mode
AT+WSSSID=YourWiFiName         # Set SSID
AT+WSKEY=WPA2,YourPassword     # Set password
AT+WANN=STATIC,192.168.1.152,255.255.255.0,192.168.1.254
AT+NETP=TCP,SERVER,8101,192.168.1.152  # TCP server mode
AT+UART=9600,8,1,NONE,NFC      # Serial settings
AT+SAVE                        # Save configuration
AT+REBOOT                      # Restart module
```

---

## 🎯 **Quick Test After Configuration:**

```bash
# Test 1: Verify port is now open
nc -zv 192.168.1.152 8101

# Test 2: Try telnet connection
telnet 192.168.1.152 8101

# Test 3: Run robot program
cd /Users/ansonchan/Pokeguide/ARIA
./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 8101
```

---

## 🔍 **Alternative Troubleshooting:**

### **If ZLAN is configured but still not working:**

1. **Check robot serial settings:**
   - Baud rate: 9600 or 38400
   - Robot might need to be in specific mode

2. **Try different ports:**
   ```bash
   # Some ZLAN modules use different ports
   ./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 23
   ./examples/consensualMovements -remoteHost 192.168.1.152 -remoteRobotTcpPort 8080
   ```

3. **Check ZLAN status LEDs:**
   - Power LED: Should be solid
   - WiFi LED: Should be solid (connected)
   - Data LED: Should blink when data flows

4. **Factory reset ZLAN:**
   - Hold reset button for 10+ seconds
   - Reconfigure from scratch

---

## 📞 **Next Steps:**

1. **First:** Check physical connections and power
2. **Second:** Try to access ZLAN web interface or reset to AP mode
3. **Third:** Configure ZLAN for TCP server mode on port 8101
4. **Fourth:** Test connection and run robot program

**The robot program is ready - we just need to configure the ZLAN module properly!**
