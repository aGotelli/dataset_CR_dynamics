#include <iostream>
#include "Wire.h"
#include <CH341DLL.H>

#ifdef _WIN32
    int main() {
        // Get version information
        unsigned long version = CH341GetVersion();
        std::cout << "CH341 Driver Version: 0x" << std::hex << version << std::dec << std::endl;
        
        // Try to open device 0
        std::cout << "\nAttempting to open device 0..." << std::endl;
        if (CH341OpenDevice(0)) {
            std::cout << "Device 0 opened successfully!" << std::endl;
            
            // Set I2C mode (mode 1 is typically I2C mode)
            std::cout << "Setting I2C mode..." << std::endl;
            if (CH341SetStream(0, 1)) {
                std::cout << "I2C mode set successfully" << std::endl;
                
                // Example: Try to read from a common I2C address (0x6A is common for ISM330DHCX)
                // This is just a test - replace with your actual sensor address
                unsigned char buffer;
                unsigned char sensorAddr1 = 0x6A;  // ISM330DHCX I2C address
                unsigned char sensorAddr2 = 0x6B;  // ISM330DHCX I2C address
                unsigned char regAddr = 0x0F;     // WHO_AM_I register
                
                std::cout << "Attempting to read WHO_AM_I register from ISM330DHCX..." << std::endl;
                if (CH341ReadI2C(0, sensorAddr1, regAddr, &buffer)) {
                    std::cout << "WHO_AM_I value 1: 0x" << std::hex << (int)buffer << std::dec << std::endl;
                    if (buffer == 0x6B) {
                        std::cout << "SUCCESS: ISM330DHCX sensor detected!" << std::endl;
                    } else {
                        std::cout << "Different sensor or no response (expected 0x6B)" << std::endl;
                    }
                } else {
                    std::cout << "Failed to read from I2C device" << std::endl;
                }
                if (CH341ReadI2C(0, sensorAddr2, regAddr, &buffer)) {
                    std::cout << "WHO_AM_I value 2: 0x" << std::hex << (int)buffer << std::dec << std::endl;
                    if (buffer == 0x6B) {
                        std::cout << "SUCCESS: ISM330DHCX sensor detected!" << std::endl;
                    } else {
                        std::cout << "Different sensor or no response (expected 0x6B)" << std::endl;
                    }
                } else {
                    std::cout << "Failed to read from I2C device" << std::endl;
                }
                
            } else {
                std::cout << "Failed to set I2C mode" << std::endl;
            }
            
            // Close device
            CH341CloseDevice(0);
            std::cout << "Device 0 closed" << std::endl;
            
        } else {
            std::cout << "Failed to open device 0" << std::endl;
            std::cout << "This might mean:" << std::endl;
            std::cout << "1. No CH341 device is connected" << std::endl;
            std::cout << "2. Device driver is not installed" << std::endl;
            std::cout << "3. Device is already in use" << std::endl;
        }
        
        std::cout << "\nTest completed" << std::endl;
        return 0;
    }

#elif defined(__linux__)
    int main() {
        std::cout << "Testing ISM330DHCX sensor on Linux platform..." << std::endl;
        
        TwoWire wire("/dev/i2c-1"); // Adjust I2C device path as needed
        wire.begin();
        
        // Test reading WHO_AM_I register
        uint8_t sensorAddr = 0x6A; // ISM330DHCX I2C address (could also be 0x6B)
        uint8_t regAddr = 0x0F;    // WHO_AM_I register
        
        std::cout << "Attempting to read WHO_AM_I register..." << std::endl;
        
        wire.beginTransmission(sensorAddr);
        wire.write(regAddr);
        if (wire.endTransmission() == 0) {
            if (wire.requestFrom(sensorAddr, (uint8_t)1) == 1) {
                uint8_t whoAmI = wire.read();
                std::cout << "WHO_AM_I value: 0x" << std::hex << (int)whoAmI << std::dec << std::endl;
                
                if (whoAmI == 0x6B) {
                    std::cout << "SUCCESS: ISM330DHCX sensor detected!" << std::endl;
                } else {
                    std::cout << "Different sensor or no response (expected 0x6B)" << std::endl;
                }
            } else {
                std::cout << "Failed to read from sensor" << std::endl;
            }
        } else {
            std::cout << "Failed to communicate with sensor" << std::endl;
        }
        
        wire.end();
        std::cout << "\nTest completed" << std::endl;
        return 0;
    }

#else
    #error "Unsupported platform"
#endif
