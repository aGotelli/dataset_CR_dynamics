# Windows Time Synchronization Setup Guide

## Overview

This guide explains how to synchronize system clocks between two Windows computers connected via Ethernet for precise timestamping in multi-computer data acquisition systems.

**Use Case**: Synchronizing timestamps between Vicon motion capture data, ATI force/torque sensors, and CyberGear motor data across multiple computers.

## Why Clock Synchronization?

### Advantages
- ✅ **Simple implementation** - No code changes needed in existing data collection scripts
- ✅ **Universal solution** - Works with all data streams (Vicon, ATI F/T, CyberGear)
- ✅ **High precision** - Can achieve sub-millisecond accuracy with proper setup
- ✅ **Transparent** - Existing `time.time()` calls automatically become synchronized
- ✅ **Industry standard** - Professional data acquisition approach

### Expected Performance
- **Good synchronization**: < 5ms average offset, < 2ms jitter
- **Excellent synchronization**: < 1ms average offset, < 0.5ms jitter

## Network Setup

### 1. Physical Connection
Connect both computers via Ethernet cable (direct connection or through a switch).

### 2. IP Configuration
Configure static IP addresses on the same subnet:

| Computer | Role | IP Address | Subnet Mask |
|----------|------|------------|-------------|
| Computer A | Time Server | 192.168.1.100 | 255.255.255.0 |
| Computer B | Time Client | 192.168.1.101 | 255.255.255.0 |

**To set static IP on Windows:**
1. Open Network and Sharing Center
2. Click "Change adapter settings"
3. Right-click your Ethernet adapter → Properties
4. Select "Internet Protocol Version 4 (TCP/IPv4)" → Properties
5. Select "Use the following IP address"
6. Enter the IP address and subnet mask above
7. Leave gateway and DNS blank for direct connection

### 3. Test Connectivity
```powershell
# From Computer B, test connection to Computer A
ping 192.168.1.100
```

## Time Server Setup (Computer A)

**⚠️ Run all commands as Administrator in PowerShell**

### Basic Server Configuration
```powershell
# Stop Windows Time service
net stop w32time

# Configure as NTP server
w32tm /config /manualpeerlist:"time.windows.com" /syncfromflags:manual /reliable:yes /update

# Enable NTP server
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer" /v Enabled /t REG_DWORD /d 1 /f

# Set high precision timing
w32tm /config /update /manualpeerlist:"time.windows.com" /syncfromflags:manual /reliable:yes

# Start the service
net start w32time

# Force synchronization with external time server
w32tm /resync /force

# Verify configuration
w32tm /query /configuration
```

### High-Precision Configuration
```powershell
# Set Windows to high-precision event timer mode
bcdedit /set useplatformclock true

# Configure W32Time for higher precision
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config" /v MaxPosPhaseCorrection /t REG_DWORD /d 3600 /f
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config" /v MaxNegPhaseCorrection /t REG_DWORD /d 3600 /f
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config" /v UpdateInterval /t REG_DWORD /d 30 /f

# Restart W32Time service
net stop w32time
net start w32time
```

## Time Client Setup (Computer B)

**⚠️ Run all commands as Administrator in PowerShell**

### Basic Client Configuration
```powershell
# Stop Windows Time service
net stop w32time

# Configure to sync with Computer A (replace IP if different)
w32tm /config /manualpeerlist:"192.168.1.100" /syncfromflags:manual /reliable:no /update

# Start the service
net start w32time

# Force immediate synchronization
w32tm /resync /force

# Verify synchronization
w32tm /query /status
```

### High-Precision Configuration
```powershell
# Set Windows to high-precision event timer mode
bcdedit /set useplatformclock true

# Configure W32Time for higher precision
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config" /v MaxPosPhaseCorrection /t REG_DWORD /d 3600 /f
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config" /v MaxNegPhaseCorrection /t REG_DWORD /d 3600 /f
reg add "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config" /v UpdateInterval /t REG_DWORD /d 30 /f

# Restart W32Time service
net stop w32time
net start w32time
```

## Verification and Testing

### 1. Check Service Status
```powershell
# On both computers - check W32Time status
w32tm /query /status
w32tm /query /peers
w32tm /query /configuration
```

### 2. Test NTP Connectivity
```powershell
# From Computer B, test NTP connection to Computer A
w32tm /stripchart /computer:192.168.1.100 /samples:5
```

### 3. Automated Synchronization Test

Use the provided Python verification script:

**On Computer A (Time Server):**
```bash
python verify_time_sync.py server
```

**On Computer B (Time Client):**
```bash
python verify_time_sync.py client 192.168.1.100
```

## Monitoring and Maintenance

### Daily Checks
```powershell
# Check synchronization status
w32tm /query /status

# Force re-synchronization if needed
w32tm /resync /force
```

### Quality Indicators

**Excellent Synchronization:**
- W32Time offset: < 1ms
- Network latency: < 1ms
- Verification script: < 1ms average, < 0.5ms jitter

**Good Synchronization:**
- W32Time offset: < 5ms
- Network latency: < 5ms
- Verification script: < 5ms average, < 2ms jitter

**Poor Synchronization (needs attention):**
- W32Time offset: > 10ms
- High network jitter
- Verification script: > 10ms average

### Troubleshooting

#### Problem: "Access Denied" Errors
**Solution**: Run PowerShell as Administrator

#### Problem: Time Sync Not Working
```powershell
# Check Windows Time service
sc query w32time

# Restart if needed
net stop w32time
net start w32time

# Check firewall (allow NTP port 123)
netsh advfirewall firewall add rule name="NTP" dir=in action=allow protocol=UDP localport=123
```

#### Problem: Large Time Offsets
1. Check network connectivity: `ping 192.168.1.100`
2. Verify NTP server is running on Computer A
3. Check Windows Time event logs: Event Viewer → Windows Logs → System
4. Force immediate sync: `w32tm /resync /force`

#### Problem: High Jitter
1. Check network cable quality
2. Reduce network traffic during data collection
3. Consider using a dedicated network switch
4. Check for CPU load during data acquisition

## Reboot Procedure

After initial setup, **reboot both computers** to ensure all settings take effect.

1. Reboot Computer A (Time Server) first
2. Wait for Computer A to fully boot and sync with external time server
3. Reboot Computer B (Time Client)
4. Verify synchronization after both computers are running

## Integration with Data Collection

### No Code Changes Required

Your existing data collection scripts using `time.time()` will automatically benefit from synchronized clocks:

```python
# This timestamp will now be synchronized across computers
timestamp = time.time()
```

### Optional: Enhanced Precision

For even higher precision, you can use the provided `sync_utils.py` module:

```python
from sync_utils import SynchronizedDataCollector

collector = SynchronizedDataCollector(use_high_precision=True)
timestamp = collector.get_timestamp()  # High-precision synchronized timestamp
```

## Security Considerations

### Internal Network Only
This setup is designed for isolated data collection networks. For production environments:

- Use proper NTP server hierarchy
- Configure Windows Firewall rules
- Consider network security policies

### Firewall Configuration
```powershell
# Allow NTP traffic (port 123 UDP)
netsh advfirewall firewall add rule name="NTP-Out" dir=out action=allow protocol=UDP remoteport=123
netsh advfirewall firewall add rule name="NTP-In" dir=in action=allow protocol=UDP localport=123
```

## Advanced Options

### External Time Reference
For absolute time accuracy, ensure Computer A synchronizes with a reliable external NTP server:

```powershell
# Use multiple time servers for redundancy
w32tm /config /manualpeerlist:"time.windows.com,time.nist.gov,pool.ntp.org" /syncfromflags:manual /reliable:yes /update
```

### Hardware Time Synchronization
For microsecond-level precision, consider:
- GPS-based time servers
- Precision Time Protocol (PTP) hardware
- Dedicated timing hardware (if budget allows)

## Performance Validation

Use the provided verification tools to validate your setup:

1. **Network test**: `python verify_time_sync.py` 
2. **Timing analysis**: `python sync_utils.py`
3. **Continuous monitoring**: Run verification periodically during data collection

## Summary Checklist

- [ ] Both computers configured with static IPs
- [ ] Network connectivity verified
- [ ] Computer A configured as NTP server
- [ ] Computer B configured as NTP client  
- [ ] High-precision timing settings applied
- [ ] Both computers rebooted
- [ ] Synchronization verified with test scripts
- [ ] Time offset < 5ms (preferably < 1ms)
- [ ] Jitter < 2ms (preferably < 0.5ms)

## Support Files

This setup includes the following support files:

- `verify_time_sync.py`: Automated synchronization testing
- `sync_utils.py`: High-precision timing utilities
- `time_server.txt`: Quick reference commands for server setup
- `time_client.txt`: Quick reference commands for client setup

---

**Note**: This procedure has been tested with Windows 10/11 systems connected via Gigabit Ethernet. Performance may vary with different hardware configurations.
