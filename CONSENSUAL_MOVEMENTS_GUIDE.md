# Consensual Movements Navigation System
## Professional Autonomous Navigation for Pioneer 3DX

### 🚀 Quick Start Guide

#### 1. **Serial Connection (Direct USB)**
```bash
cd /Users/ansonchan/Pokeguide/ARIA
./examples/consensualMovements -robotPort /dev/tty.usbserial-140
```

#### 2. **SSH/TCP Connection (Network)**
```bash
# Connect to robot over network
./examples/consensualMovements -remoteHost <ROBOT_IP> -remoteRobotTcpPort 8101

# Example with specific IP
./examples/consensualMovements -remoteHost 192.168.1.100 -remoteRobotTcpPort 8101
```

#### 3. **Simulator Connection**
```bash
./examples/consensualMovements -remoteIsSim
```

### 🎮 Operation Instructions

1. **Start the program** with your preferred connection method
2. **Wait for initialization** - System will connect and initialize sensors
3. **Press SPACE** when prompted to enable autonomous movement
4. **Press Ctrl+C** to stop the robot safely

### 📊 System Features

#### **Multi-Behavior Consensus Algorithm**
- **Obstacle Avoidance** (Weight: 0.8) - Primary safety behavior
- **Goal Seeking** (Weight: 0.6) - Navigate to random waypoints  
- **Wall Following** (Weight: 0.4) - Follow walls when detected
- **Exploration** (Weight: 0.3) - Intelligent random exploration
- **Smooth Motion** (Weight: 0.5) - Comfortable acceleration

#### **Safety Systems**
- **Critical Distance**: 400mm - Emergency stop
- **Safe Distance**: 800mm - Obstacle avoidance activation
- **16 Sonar Sensors** - Full 360° coverage
- **Sensor Filtering** - Noise reduction and outlier rejection
- **Battery Monitoring** - Low voltage warnings
- **Connection Monitoring** - Automatic disconnection handling

#### **Performance Parameters**
- **Max Linear Velocity**: 400mm/s
- **Max Angular Velocity**: 60°/s  
- **Update Rate**: 20Hz (50ms cycle)
- **Goal Range**: ±3000mm (6m diameter exploration area)

### 🔧 Connection Troubleshooting

#### **Serial Connection Issues**
```bash
# Check available ports
ls /dev/tty.usb*

# Use correct port
./examples/consensualMovements -robotPort /dev/tty.usbserial-XXX
```

#### **SSH/TCP Connection Issues**
```bash
# Test network connectivity first
ping <ROBOT_IP>

# Check robot server is running on port 8101
telnet <ROBOT_IP> 8101

# Use correct parameters
./examples/consensualMovements -remoteHost <ROBOT_IP> -remoteRobotTcpPort 8101
```

### 📈 Real-Time Monitoring

The system provides live feedback:
```
Status: Pos(1234,5678) Heading:45.2° Battery:12.1V Sonar:1500mm
🚧 Obstacle detected at 450mm
⚠️  LOW BATTERY WARNING: 10.8V
```

### 🛡️ Safety Notes

- **Always supervise** the robot during autonomous operation
- **Clear the area** of obstacles before starting
- **Monitor battery levels** - Stop operation below 11V
- **Use emergency stop** (Ctrl+C) if needed
- **Test in safe environment** first

### 🔄 Behavior Explanations

1. **Consensus Decision Making**: All behaviors "vote" with confidence levels
2. **Weighted Averaging**: Higher priority behaviors have more influence
3. **Real-time Adaptation**: System responds to changing conditions
4. **Smooth Transitions**: Gradual velocity changes for comfort
5. **Goal Management**: Automatic waypoint generation and achievement

### 📝 Log Output Interpretation

```
ConsensualMovements: Consensus -> Vel:323 RotVel:2.2 Conf:0.28 MinDist:5000
```

- **Vel**: Linear velocity in mm/s
- **RotVel**: Angular velocity in degrees/s  
- **Conf**: Consensus confidence (0.0-1.0)
- **MinDist**: Closest obstacle distance in mm

### 🎯 Advanced Usage

#### **Custom Parameters**
Edit the source code to modify:
- Behavior weights (lines 40-45)
- Safety distances (lines 119-120)  
- Velocity limits (lines 117-118)
- Update rates (line 156)

#### **SSH Deployment**
```bash
# Copy to robot computer
scp examples/consensualMovements user@robot:/home/user/

# Run remotely via SSH
ssh user@robot './consensualMovements -robotPort /dev/ttyS0'
```

---

## 🤖 **Your Pioneer 3DX is now ready for professional autonomous navigation!**

The consensual movements algorithm provides safe, intelligent, and adaptive robot navigation suitable for research, education, and practical applications.
