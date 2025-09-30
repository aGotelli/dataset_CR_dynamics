#include <iostream>
#include <vector>
#include <string>
#include <windows.h>

int main() {
    std::cout << "Testing CH341 DLL function discovery..." << std::endl;
    
    // Try to load the DLL
    HMODULE hDLL = LoadLibraryA("CH341DLLA64.dll");
    if (!hDLL) {
        std::cerr << "Failed to load CH341DLLA64.dll. Error: " << GetLastError() << std::endl;
        return -1;
    }
    
    std::cout << "CH341DLLA64.dll loaded successfully!" << std::endl;
    
    // Test common function names by trying to get their addresses
    std::vector<std::string> commonFunctions = {
        "CH341OpenDevice", "CH341CloseDevice", "CH341GetVersion",
        "CH341GetDeviceCount", "CH341SetStream", "CH341ReadI2C",
        "CH341WriteI2C", "CH341StreamI2C", "CH341StreamSPI4", "CH341StreamSPI5",
        "CH341SetOutput", "CH341GetInput", "CH341ReadData", 
        "CH341WriteData", "CH341ResetDevice", "CH341GetStatus",
        "CH341SetTimeout", "CH341FlushBuffer", "CH341GetDeviceName",
        "CH341SetExclusive", "CH341GetDriverVersion"
    };
    
    std::cout << "\nTesting common function names:\n";
    std::cout << "================================\n";
    
    int foundCount = 0;
    for (const auto& funcName : commonFunctions) {
        FARPROC proc = GetProcAddress(hDLL, funcName.c_str());
        std::cout << funcName << ": " << (proc ? "FOUND" : "NOT FOUND") << std::endl;
        if (proc) foundCount++;
    }
    
    std::cout << "\nSummary: Found " << foundCount << " out of " << commonFunctions.size() << " tested functions." << std::endl;
    
    if (foundCount == 0) {
        std::cout << "\nNo standard CH341 functions found. This might mean:" << std::endl;
        std::cout << "1. The DLL uses different function names" << std::endl;
        std::cout << "2. The functions are exported with C++ name mangling" << std::endl;
        std::cout << "3. The DLL is for a different architecture" << std::endl;
        std::cout << "4. You need to check the official CH341 documentation" << std::endl;
    }
    
    FreeLibrary(hDLL);
    std::cout << "\nTest completed." << std::endl;
    return 0;
}
