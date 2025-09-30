#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <memory>

#ifdef _WIN32
#include <windows.h>
#include <string>
#include <CH341DLL.H>
#elif defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#endif


// TwoWire class with platform-specific implementations
class TwoWire
{
public:
#ifdef _WIN32
    TwoWire(const char *device = "CH341") : deviceName(device) { std::cout << "TwoWire constructor called.\n"; }

    ~TwoWire()
    {
        end();
    }

    void begin()
    {
        if (!CH341OpenDevice(0))
        {
            std::cerr << "Failed to open CH341 device 0" << std::endl;
            return;
        }

        if (!CH341SetStream(0, 3))
        { // Set I2C mode
            std::cerr << "Failed to set I2C mode" << std::endl;
            CH341CloseDevice(0);
            return;
        }

        isInitialized = true;
        txBuffer.clear();
        while (!rxBuffer.empty())
            rxBuffer.pop();
        
        std::cout << "CH341 device initialized successfully" << std::endl;
    }

    void end()
    {
        CH341CloseDevice(0);
        isInitialized = false;
        txBuffer.clear();
        while (!rxBuffer.empty())
            rxBuffer.pop();
    }

    void beginTransmission(uint8_t address)
    {
        targetAddress = address;
        txBuffer.clear();
    }

    void write(uint8_t data)
    {
        txBuffer.push_back(data);
    }

    void write(const uint8_t *data, int length)
    {
        txBuffer.insert(txBuffer.end(), data, data + length);
    }

    int endTransmission(bool stop = true)
    {
        if (!isInitialized)
            return -1;

        // For I2C write, first byte is usually the register address
        if (txBuffer.size() == 1)
        {
            // Just setting register address - store it for later use in requestFrom
            lastRegisterAddress = txBuffer[0];
            // std::cout << "[WIRE] Set register address: 0x" << std::hex << (int)lastRegisterAddress << std::dec << std::endl;
            return 0;
        }
        else if (txBuffer.size() >= 2)
        {
            // Write data to register
            unsigned char regAddr = txBuffer[0];
            std::vector<unsigned char> data(txBuffer.begin() + 1, txBuffer.end());

            // std::cout << "[WIRE] Writing to 0x" << std::hex << (int)targetAddress << std::dec
            //           << " reg 0x" << std::hex << (int)regAddr << std::dec << ": ";
            // for (size_t i = 0; i < data.size(); i++) {
            //     std::cout << "0x" << std::hex << (int)data[i] << std::dec;
            //     if (i < data.size() - 1) std::cout << " ";
            // }
            // std::cout << std::endl;

            bool success = true;
            // success = CH341StreamI2C(0, txBuffer.size(), txBuffer.data(), 0, NULL); 
            for (uint8_t item : data)
            {
                success = success & CH341WriteI2C(0, targetAddress, regAddr, item);
            }
            if (!success)
            {
                std::cout << "[WIRE] Write FAILED! CH341 error." << std::endl;
                DWORD error = GetLastError();
                std::cout << "[WIRE] GetLastError: " << error << std::endl;
            }
            else
            {
                // std::cout << "[WIRE] Write successful." << std::endl;
            }
            return success ? 0 : -1;
        }
        else
        { // Buffer is empty
            // std::cout << "[WIRE] Calling endTransmission with empty buffer\n";
            bool success = CH341WriteI2C(0, targetAddress, 0x00, 0x00);
            return success ? 0 : -1;
        }

        return 0;
    }

    uint16_t requestFrom(uint8_t address, uint8_t numBytes, bool stop = true)
    {
        if (!isInitialized)
            return 0;

        // Debug important reads
        // {
            // std::cout << "[WIRE] Reading " << (int)numBytes << " bytes from 0x" << std::hex << (int)address << std::dec
            //           << " reg 0x" << std::hex << (int)lastRegisterAddress << std::dec << std::endl;
        // }

        std::vector<unsigned char> buffer(numBytes); 

        bool success = true; 
        for (int i = 0; i < numBytes; i++)
        {
            uint8_t read_byte;
            success = success & CH341ReadI2C(0, address, lastRegisterAddress, &read_byte);
            lastRegisterAddress += 0x1; 
            buffer[i] = read_byte;
        }

        if (!success)
        {
            std::cout << "[WIRE] Read FAILED! CH341 error." << std::endl;
            DWORD error = GetLastError();
            std::cout << "[WIRE] GetLastError: " << error << std::endl;
            return 0;
        }
        // }

        // Debug important read results
        // {
            // std::cout << "[WIRE] Read result: ";
            // for (int i = 0; i < numBytes; i++) {
            //     std::cout << "0x" << std::hex << (int)buffer[i] << std::dec;
            //     if (i < numBytes - 1) std::cout << " ";
            // }
            // std::cout << std::endl;
        // }

        // Push received data into RX queue
        for (int i = 0; i < numBytes; i++)
        {
            rxBuffer.push(buffer[i]);
        }

        return numBytes;
    }

    uint8_t read()
    {
        if (rxBuffer.empty())
        {
            return 0xFF; // mimic Arduino: return -1, but cast to uint8_t
        }
        uint8_t b = rxBuffer.front();
        rxBuffer.pop();
        return b;
    }

    int available() const
    {
        return static_cast<int>(rxBuffer.size());
    }

private:
    std::string deviceName;
    bool isInitialized = false;
    uint8_t targetAddress = 0;
    uint8_t lastRegisterAddress = 0;
    std::vector<uint8_t> txBuffer;
    std::queue<uint8_t> rxBuffer;

#elif defined(__linux__)
    TwoWire(const char *device = "/dev/i2c-16") : devicePath(device) {}

    ~TwoWire()
    {
        if (fd >= 0)
        {
            close(fd);
        }
    }

    void begin()
    {
        if (fd >= 0)
            close(fd); // close if previously open
        fd = open(devicePath.c_str(), O_RDWR);
        if (fd < 0)
        {
            perror("Failed to open I2C device");
        }
        txBuffer.clear();
        while (!rxBuffer.empty())
            rxBuffer.pop();
    }

    void end()
    {
        if (fd >= 0)
        {
            close(fd);
            fd = -1;
        }
        txBuffer.clear();
        while (!rxBuffer.empty())
            rxBuffer.pop();
    }

    void beginTransmission(uint8_t address)
    {
        targetAddress = address;
        txBuffer.clear();
    }

    void write(uint8_t data)
    {
        txBuffer.push_back(data);
    }

    void write(const uint8_t *data, int length)
    {
        txBuffer.insert(txBuffer.end(), data, data + length);
    }

    int endTransmission(bool stop = true)
    {
        if (fd < 0)
            return -1;
        if (ioctl(fd, I2C_SLAVE, targetAddress) < 0)
        {
            perror("Failed to set I2C address");
            return -1;
        }

        ssize_t written = ::write(fd, txBuffer.data(), txBuffer.size());
        if (written != (ssize_t)txBuffer.size())
        {
            perror("Failed to write all bytes");
            return -1;
        }
        return 0; // success
    }

    uint16_t requestFrom(uint8_t address, uint8_t numBytes, bool stop = true)
    {
        if (fd < 0)
            return 0;
        if (ioctl(fd, I2C_SLAVE, address) < 0)
        {
            perror("Failed to set I2C address");
            return 0;
        }

        std::vector<uint8_t> buf(numBytes);
        ssize_t readBytes = ::read(fd, buf.data(), numBytes);
        if (readBytes < 0)
        {
            perror("Failed to read");
            return 0;
        }

        // push into RX queue
        for (ssize_t i = 0; i < readBytes; i++)
        {
            rxBuffer.push(buf[i]);
        }
        return static_cast<uint16_t>(readBytes);
    }

    uint8_t read()
    {
        if (rxBuffer.empty())
        {
            return 0xFF; // mimic Arduino: return -1, but cast to uint8_t
        }
        uint8_t b = rxBuffer.front();
        rxBuffer.pop();
        return b;
    }

    int available() const
    {
        return static_cast<int>(rxBuffer.size());
    }

private:
    std::string devicePath;
    int fd = -1;
    uint8_t targetAddress = 0;
    std::vector<uint8_t> txBuffer;
    std::queue<uint8_t> rxBuffer;
#endif // __linux__
};