#include "Wire.h"
#include <iostream>

int main() {
    std::cout << "Testing ISM330DHCX sensor using unified TwoWire API..." << std::endl;
    
#ifdef _WIN32
    std::cout << "Platform: Windows (using CH341 USB-to-I2C)" << std::endl;
    TwoWire wire("CH341");
#elif defined(__linux__)
    std::cout << "Platform: Linux (using native I2C)" << std::endl;
    TwoWire wire("/dev/i2c-1"); // Adjust I2C device path as needed
#endif
    
    // Initialize I2C interface
    wire.begin();
    
    // Test reading WHO_AM_I register from ISM330DHCX
    uint8_t sensorAddr = 0x6A; // ISM330DHCX I2C address (could also be 0x6B)
    uint8_t regAddr = 0x0F;    // WHO_AM_I register
    
    std::cout << "Attempting to read WHO_AM_I register..." << std::endl;
    
    // Send register address
    wire.beginTransmission(sensorAddr);
    wire.write(regAddr);
    int result = wire.endTransmission();
    
    if (result == 0) {
        // Read the register
        if (wire.requestFrom(sensorAddr, (uint8_t)1) == 1) {
            uint8_t whoAmI = wire.read();
            std::cout << "WHO_AM_I value: 0x" << std::hex << (int)whoAmI << std::dec << std::endl;
            
            if (whoAmI == 0x6B) {
                std::cout << "SUCCESS: ISM330DHCX sensor detected!" << std::endl;
                
                // Try reading accelerometer data
                std::cout << "\nReading accelerometer data..." << std::endl;
                
                // Read OUTX_L_A (0x28) - Low byte of X-axis acceleration
                uint8_t accelRegAddr = 0x28;
                wire.beginTransmission(sensorAddr);
                wire.write(accelRegAddr);
                
                if (wire.endTransmission() == 0) {
                    if (wire.requestFrom(sensorAddr, (uint8_t)6) == 6) { // Read 6 bytes (X, Y, Z)
                        int16_t accelX = wire.read() | (wire.read() << 8);
                        int16_t accelY = wire.read() | (wire.read() << 8);
                        int16_t accelZ = wire.read() | (wire.read() << 8);
                        
                        std::cout << "Accelerometer data:" << std::endl;
                        std::cout << "  X: " << accelX << std::endl;
                        std::cout << "  Y: " << accelY << std::endl;
                        std::cout << "  Z: " << accelZ << std::endl;
                    } else {
                        std::cout << "Failed to read accelerometer data" << std::endl;
                    }
                } else {
                    std::cout << "Failed to send accelerometer register address" << std::endl;
                }
                
            } else {
                std::cout << "Different sensor or no response (expected 0x6B)" << std::endl;
                
                // Try alternative address
                sensorAddr = 0x6B;
                std::cout << "Trying alternative address 0x6B..." << std::endl;
                
                wire.beginTransmission(sensorAddr);
                wire.write(regAddr);
                if (wire.endTransmission() == 0) {
                    if (wire.requestFrom(sensorAddr, (uint8_t)1) == 1) {
                        whoAmI = wire.read();
                        std::cout << "WHO_AM_I value (0x6B): 0x" << std::hex << (int)whoAmI << std::dec << std::endl;
                        
                        if (whoAmI == 0x6B) {
                            std::cout << "SUCCESS: ISM330DHCX sensor detected at address 0x6B!" << std::endl;
                        }
                    }
                }
            }
        } else {
            std::cout << "Failed to read from sensor" << std::endl;
        }
    } else {
        std::cout << "Failed to communicate with sensor (error code: " << result << ")" << std::endl;
        std::cout << "This might mean:" << std::endl;
        std::cout << "1. Sensor is not connected" << std::endl;
        std::cout << "2. Wrong I2C address" << std::endl;
        std::cout << "3. I2C interface not working properly" << std::endl;
    }
    
    wire.end();
    std::cout << "\nUnified API test completed" << std::endl;
    return 0;
}
