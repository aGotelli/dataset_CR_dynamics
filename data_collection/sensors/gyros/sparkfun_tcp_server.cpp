#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <sstream>
#include <chrono>
#include <memory>

#ifdef _WIN32
    #include <winsock2.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #define SOCKET int
    #define closesocket close
#endif
#include "gyro.h"

class SparkFunTCPServer {
private:
    int m_frequency { 300 };
    int m_port { 9999 };  // Changed from 8080 to 9999
    std::unique_ptr<GyroAPI> m_gyro_api;
    std::string m_folder_path;
    double m_duration;
    
public:
    SparkFunTCPServer(int t_frequency) 
    : m_frequency(t_frequency)
    {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
#endif

        // Initialize once at startup; setup_gyro configures recording on success
        setup_gyro();
    }

    bool setup_gyro() 
    {
        // Recreate the device cleanly
        m_gyro_api.reset();
#ifdef __linux__
        std::cout << "Using Linux I2C interface" << std::endl;
        m_gyro_api = std::make_unique<GyroAPI>();
#elif defined(_WIN32)
        std::cout << "Using Windows CH341 USB-to-I2C interface" << std::endl;
        m_gyro_api = std::make_unique<GyroAPI>();
#endif

        // Try both possible addresses
        m_gyro_api->add_device(ISM330DHCX_ADDRESS_LOW);
        //std::this_thread::sleep_for(std::chrono::seconds(1));
        m_gyro_api->add_device(ISM330DHCX_ADDRESS_HIGH);

        if (!m_gyro_api->statusCheck()) 
            return false;

        // Configure recording frequency after a healthy init
        m_gyro_api->setRecord(true, m_frequency);
        return true;
    }
    
    void start() {
        SOCKET server = socket(AF_INET, SOCK_STREAM, 0);
        
        // Allow socket reuse
        int opt = 1;
        setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
        
        sockaddr_in addr = {AF_INET, htons(m_port), INADDR_ANY};
        
        int result = bind(server, (sockaddr*)&addr, sizeof(addr));
        if(result != 0) {
            std::cout << "Bind failed!" << std::endl;
            return;
        }
        
        listen(server, 1);
        std::cout << "Server listening on port " << std::dec << m_port << std::endl;
        
        
        std::vector<char> load = {
            static_cast<char>(124),
            static_cast<char>(92),
            static_cast<char>(45),
            static_cast<char>(47)
        };
        int it = 0;
        while(true) {
            SOCKET client = accept(server, nullptr, nullptr);
            if(client == INVALID_SOCKET) {
                // Accept failed or timed out, continue waiting
                std::cout << "\rWaiting connection " << load[it++];
                std::cout.flush();
                if(it == 3)
                    it = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            
            std::cout << "\tClient connected!" << std::endl;

            // Only re-init if device is unhealthy
            if(!m_gyro_api || !m_gyro_api->statusCheck()) {
                std::cout << "\t\tReset Connection..." << std::endl;
                setup_gyro();
            }
            
            while(true) {
                char buf[256];
                int bytes = recv(client, buf, 256, 0);
                if(bytes <= 0) break;
                
                buf[bytes] = '\0';
                std::string cmd(buf);
                // std::cout << "\tReceived " << bytes << " bytes: " << cmd << std::endl;
                
                if(cmd.find("setup") != std::string::npos) {
                    std::cout << "\tReceived SETUP command" << std::endl;
                    // std::cout << "\tRaw command: " << cmd << std::endl;
                    
                    // Parse folder and duration from JSON
                    size_t folder_pos = cmd.find("\"folder\"");
                    size_t duration_pos = cmd.find("\"duration\"");
                    
                    if(folder_pos != std::string::npos) {
                        size_t colon_pos = cmd.find(":", folder_pos);
                        size_t start = cmd.find("\"", colon_pos) + 1;  // Find opening quote after colon
                        size_t end = cmd.find("\"", start);           // Find closing quote
                        if(start != std::string::npos && end != std::string::npos) {
                            m_folder_path = cmd.substr(start, end - start);
                        }
                    }
                    
                    if(duration_pos != std::string::npos) {
                        size_t colon_pos = cmd.find(":", duration_pos);
                        size_t start = colon_pos + 1;
                        // Skip whitespace
                        while(start < cmd.length() && (cmd[start] == ' ' || cmd[start] == '\t')) start++;
                        size_t end = cmd.find_first_of(",}", start);
                        if(start != std::string::npos && end != std::string::npos) {
                            m_duration = std::stod(cmd.substr(start, end - start));
                        }
                    }
                    
                    std::cout << "\t\tFolder received: '" << m_folder_path << "'" << std::endl;
                    std::cout << "\t\tDuration received: " << m_duration << " seconds\n\n" << std::endl;
                    const std::string ready = "{\"status\": \"ready\"}";
                    send(client, ready.c_str(), (int)ready.size(), 0);
                } 
                if(cmd.find("start") != std::string::npos) {
                    std::cout << "\tReceived START command" << std::endl;
                    const std::string started = "{\"status\": \"Recording started\"}";
                    send(client, started.c_str(), (int)started.size(), 0);
                    
                    if(m_gyro_api && !m_folder_path.empty()) {
                        // Record start time
                        auto start_time = std::chrono::steady_clock::now();
                        
                        m_gyro_api->startUpdateLoop(const_cast<char*>(m_folder_path.c_str()));
                        
                        // Wait for the specified duration
                        std::this_thread::sleep_for(std::chrono::duration<double>(m_duration));
                        
                        // Stop recording
                        m_gyro_api->stopUpdateLoop();
                        
                        auto end_time = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                        std::cout << "\t Recording stopped after " << elapsed.count() << " ms" << std::endl;
                    } else {
                        std::cout << "\t❌ Cannot start recording: gyro not initialized or folder not set" << std::endl;
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            closesocket(client);
            std::cout << "Client disconnected" << std::endl;
        }
    }
};

int main(int argc, char **argv)
{


    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <frequency_hz>" << std::endl;
        return 1;
    }

    std::cout << "Starting Server" << std::endl;

    int frequency = std::stoi(argv[1]); // Desired frequency in Hz

    SparkFunTCPServer tcp_server(frequency);
    tcp_server.start();
    
    return 0;
}
    
