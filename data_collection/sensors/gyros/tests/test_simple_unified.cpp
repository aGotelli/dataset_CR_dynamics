#include "Wire.h"
#include <iostream>

int main() {
    std::cout << "Testing simplified unified TwoWire API..." << std::endl;
    
#ifdef _WIN32
    std::cout << "Platform: Windows (using CH341 USB-to-I2C)" << std::endl;
#elif defined(__linux__)
    std::cout << "Platform: Linux (using native I2C)" << std::endl;
#endif
    
    try {
        TwoWire wire;
        std::cout << "TwoWire object created successfully" << std::endl;
        
        std::cout << "Calling wire.begin()..." << std::endl;
        wire.begin();
        std::cout << "wire.begin() completed" << std::endl;
        
        std::cout << "Testing basic I2C communication..." << std::endl;
        
        // Simple test - just try to start and end transmission
        uint8_t testAddr = 0x6A;
        wire.beginTransmission(testAddr);
        std::cout << "beginTransmission() called" << std::endl;
        
        int result = wire.endTransmission();
        std::cout << "endTransmission() returned: " << result << std::endl;
        
        std::cout << "Cleaning up..." << std::endl;
        wire.end();
        std::cout << "wire.end() completed" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Simplified test completed successfully" << std::endl;
    return 0;
}
