#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include "gyro.h"

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <log_folder> <frequency [Hz]> <duration [s]" << std::endl;
        return 1;
    }

    
    std::filesystem::path log_folder_path = std::filesystem::path(argv[1]);

    if (!std::filesystem::exists(log_folder_path))
        std::filesystem::create_directory(log_folder_path);
    else
        std::cout << "[WARNING] Log folder already exists. Data will be appended to existing files.\n";

    std::cout << "Initializing gyro..." << std::endl;
    
#ifdef __linux__
    std::cout << "Using Linux I2C interface" << std::endl;
    GyroAPI gyro_api = GyroAPI(); // Default Linux path: /dev/i2c-16
#elif defined(_WIN32)
    std::cout << "Using Windows CH341 USB-to-I2C interface" << std::endl;
    GyroAPI gyro_api = GyroAPI(); // Default Windows device: CH341
#endif

    // Try both possible addresses
    gyro_api.add_device(ISM330DHCX_ADDRESS_LOW); // Soldered address (0x6A) End effector right now
    gyro_api.add_device(ISM330DHCX_ADDRESS_HIGH); // Default (unsoldered) address (0x6B) Middle one right now 

    // Check if any devices were successfully detected
    if (!gyro_api.statusCheck()) {
        std::cout << "No devices detected. Exiting..." << std::endl;
        return 1;
    }

    int duration = std::stoi(argv[3]); // Desired frequency in Hz

    int frequency = std::stoi(argv[2]); // Desired frequency in Hz
    gyro_api.setRecord(true, frequency);
    gyro_api.startUpdateLoop(argv[1]);
    std::cout << "Started recording at " << frequency << " Hz. Press Enter to stop." << std::endl;

    // std::cin.get();  // Stop condition - currently set to ENTER

    // Wait for the specified duration
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));

    // TODO: Change stop condition if wanted 
    gyro_api.stopUpdateLoop(); 
    std::cout << "Recording stopped." << std::endl;
    return 0; 
}
