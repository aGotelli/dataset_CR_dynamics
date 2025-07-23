#include <iostream>
#include <chrono>
#include <vector>
#include <windows.h>

int main() {
    // Open serial port
    HANDLE serial = CreateFileA("COM5", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    
    // Set basic serial parameters
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(serial, &dcb);
    dcb.BaudRate = 115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    SetCommState(serial, &dcb);
    
    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadTotalTimeoutConstant = 100;
    SetCommTimeouts(serial, &timeouts);
    
    // First, measure the actual response size
    std::cout << "Measuring response size...\n";
    char testBuffer[100];
    DWORD testWritten, testRead;
    WriteFile(serial, "?\r", 2, &testWritten, NULL);
    ReadFile(serial, testBuffer, 100, &testRead, NULL);
    testBuffer[testRead] = '\0';  // Null terminate for display
    
    std::cout << "Response size: " << testRead << " bytes\n";
    std::cout << "Response content: '" << testBuffer << "'\n";
    
    // Use the measured size for timing tests
    const int samples = 10000;
    std::vector<double> times(samples);
    
    std::cout << "Measuring " << samples << " samples with " << testRead << " byte buffer...\n";
    
    DWORD written;
    DWORD read;
    char buffer[100];  // Keep large buffer but only read what we need
    for (int i = 0; i < samples; i++) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Send query
        WriteFile(serial, "?\r", 2, &written, NULL);
        
        // Read response - use the measured size
        ReadFile(serial, buffer, testRead, &read, NULL);
        
        auto end = std::chrono::high_resolution_clock::now();
        
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        times[i] = ms;
    }
    
    // Calculate average
    double total = 0;
    for (double t : times) total += t;
    double avg = total / times.size();
    
    std::cout << "Average time: " << avg << " ms\n";
    std::cout << "Data rate: " << (1000.0 / avg) << " Hz\n";
    
    CloseHandle(serial);
    return 0;
}
