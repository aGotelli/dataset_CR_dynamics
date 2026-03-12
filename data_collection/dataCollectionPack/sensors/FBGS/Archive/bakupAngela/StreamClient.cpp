#pragma once

#include <inttypes.h>
#include <string>
#include <chrono>
#include "StreamClient.h"

StreamClient::StreamClient(const char* _address, const char* _port, double runtime)
{
    // Initialize connection variables
    address = _address;
    port = _port;
    ConnectSocket = INVALID_SOCKET;
    isConnected = NOT_CONNECTED;
    stop = FALSE;

    fiberIndex = 255;
    error = 65000;
    length = 0;
    lineNumber = 0;
    timestamp = 0;

    // initialize data array pointers
    curvature = nullptr;
    angle = nullptr;
    shape = nullptr;
    temperature = nullptr;

    recvbuf = new char[DEFAULT_BUFLEN];

    fieldLength = 0;
    ID = 255;

    // initialize dynamic array lengths
    curvatureArrayLength = 0;
    angleArrayLength = 0;
    shapeArrayLength = 0;
    temperatureArrayLength = 0;
    arrayLength = 0;

    // initialize Cores dynamic arrays
    for (int i = 0; i < 4; i++)
    {
        allCoresData[i].spectrumWL = nullptr;
        allCoresData[i].spectrumPower = nullptr;
        allCoresData[i].peaksWL = nullptr;
        allCoresData[i].peaksPower = nullptr;
    }

    dataToSave.reserve(int(runtime * 110.0)); 

    //spectrumWLSize = { 0; 0; 0;0 };
    //spectrumPowerSize = 0;
    //peaksWLSize = 0;
    //peaksPowerSize = 0;
    
}

StreamClient::~StreamClient()
{
    if (isConnected)
        closeStream();

    // Delete all dynamic arrays
    
    if (curvature != nullptr)
        delete curvature;

    if (angle != nullptr)
        delete angle;

    if (shape != nullptr)
        delete shape;

    if (temperature != nullptr)
        delete temperature;

    for (int j = 0; j < 4; j++)
    {
        if (allCoresData[j].spectrumWL != nullptr)
            delete allCoresData[j].spectrumWL;

        if (allCoresData[j].spectrumPower != nullptr)
            delete allCoresData[j].spectrumPower;

        if (allCoresData[j].peaksWL != nullptr)
            delete allCoresData[j].peaksWL;

        if (allCoresData[j].peaksPower != nullptr)
            delete allCoresData[j].peaksPower;
    }
}

int StreamClient::connectStream()
{
    static WSADATA wsaData;
    static struct addrinfo* result = NULL,
        * ptr = NULL,
        hints;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 2;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(address, port, &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 3;
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 4;
        }

        // Connect to server.
        iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 5;
    }
    //printf("Bytes Sent: %ld\n", iResult);

    // shutdown the connection since no data will be sent
    iResult = shutdown(ConnectSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return 6;
    }
    else
        isConnected = CONNECTED;
    return 0;
}

int StreamClient::readStream()
{
    iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

    if (iResult > 0)
    {
        memcpy(&length, recvbuf, 4);
        memcpy(&fiberIndex, &recvbuf[4], 1);
        uint32_t fieldOffset = 5;

        //printf("length : %" PRIu32 "\nFiber index : %" PRIu8 "\n", length, fiberIndex);

        while (fieldOffset < length)
        {
            memcpy(&fieldLength, &recvbuf[fieldOffset], 4);
            fieldOffset += 4;
            memcpy(&ID, &recvbuf[fieldOffset], 2);
            fieldOffset += 2;

            //printf("Field length : %d\nID : %d\n", fieldLength, ID);
            //printf("ID : % d", ID);

            switch (ID)
            {
            default:
                return 1; // not recognized message ID
                break;
            case 0: // error
                memcpy(&error, &recvbuf[fieldOffset], 2);
                fieldOffset += 2;
                printf("     error : %d\n", error);
                break;

            case 1: // line number
                memcpy(&lineNumber, &recvbuf[fieldOffset], 8);
                fieldOffset += 8;
                //printf("     Line number : %" PRIu64 "\n", lineNumber);
                break;

            case 2:    // timestamp
                memcpy(&timestamp, &recvbuf[fieldOffset], 8);
                //printf("     Timestamp : %f\n", timestamp);
                fieldOffset += 8;
                break;

            case 3:    // Curvature
                // copies the number of elements in the array
                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                fieldOffset += 4;

                // Allocates/reallocates array if necessary
                if (curvatureArrayLength != arrayLength)
                {
                    curvatureArrayLength = arrayLength;
                    if (curvature != nullptr)
                        delete[] curvature;
                    if (arrayLength != 0)
                        curvature = new float_t[arrayLength];
                    else
                        curvature = nullptr;
                }

                memcpy(curvature, &recvbuf[fieldOffset], 4 * curvatureArrayLength);
                fieldOffset += 4 * curvatureArrayLength;

                //printf("     Curvature : %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", curvature[0], curvature[1], curvature[2], curvature[3], curvature[4], curvature[5], curvature[6], curvature[7], curvature[8], curvature[9], curvature[10], curvature[11], curvature[11]);
                break;

            case 4:    // angle
                // copies the number of elements in the array
                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                fieldOffset += 4;

                // Allocates/reallocates array if necessary
                if (angleArrayLength != arrayLength)
                {
                    angleArrayLength = arrayLength;
                    if (angle != nullptr)
                        delete[] angle;
                    if (arrayLength != 0)
                        angle = new float_t[arrayLength];
                    else
                        angle = nullptr;
                }

                memcpy(angle, &recvbuf[fieldOffset], 4 * angleArrayLength);
                fieldOffset += 4 * angleArrayLength;

                //printf("     Angle : %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", angle[0], angle[1], angle[2], angle[3], angle[4], angle[5], angle[6], angle[7], angle[8], angle[9], angle[10], angle[11], angle[11]);
                break;

            case 5: // Shape
                // Reads the number of values in the array
                uint32_t arrayWidth;
                memcpy(&arrayWidth, &recvbuf[fieldOffset], 4);
                uint32_t arrayHeight;
                memcpy(&arrayHeight, &recvbuf[fieldOffset+4], 4);

                arrayLength = arrayHeight * arrayWidth;

                fieldOffset += 8;

                // Allocates/reallocates array if necessary
                if (shapeArrayLength != arrayLength)
                {
                    shapeArrayLength = arrayLength;
                    shapeArrayLength = arrayLength;
                    if (shape != nullptr)
                        delete[] shape;
                    if (arrayLength != 0)
                        shape = new float_t[arrayLength];
                    else
                        shape = nullptr;
                }

                memcpy(shape, &recvbuf[fieldOffset], 4 * shapeArrayLength);
                fieldOffset += 4 * shapeArrayLength;

				//printf("     Shape : %d x %d\n", arrayWidth, arrayHeight);
    //            printf("     Shape : \n        %f, %f, %f\n        %f, %f, %f\n        %f, %f, %f\n        %f, %f, %f\n",
    //                shape[0], shape[1], shape[2], shape[3], shape[4], shape[5], shape[6], shape[7], shape[8], shape[9], shape[10], shape[11]);
                break;

            case 6: // Temperature
                // copies the number of elements in the array
                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                fieldOffset += 4;

                // Allocates/reallocates array if necessary
                if (temperatureArrayLength != arrayLength)
                {
                    temperatureArrayLength = arrayLength;
                    if (temperature != nullptr)
                        delete[] temperature;
                    if (arrayLength != 0)
                        temperature = new float_t[arrayLength];
                    else
                        temperature = nullptr;
                }

                memcpy(temperature, &recvbuf[fieldOffset], 4 * temperatureArrayLength);
                fieldOffset += 4 * temperatureArrayLength;

                //printf("     Temperature : %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", temperature[0], temperature[1], temperature[2], temperature[3], temperature[4], temperature[5], temperature[6], temperature[7], temperature[8], temperature[9], temperature[10], temperature[11], temperature[11]);
                break;

            case 7: // spectra field
                for (int i = 0; i < 4 ; i++)
                {
                    uint32_t spectraFieldLength;
                    memcpy(&spectraFieldLength, &recvbuf[fieldOffset], 4);

                    uint32_t fieldEnd = fieldOffset + spectraFieldLength;
                    fieldOffset += 4;

                    uint8_t channel;
                    memcpy(&channel, &recvbuf[fieldOffset], 1);
                    fieldOffset += 1;

                    //printf("Length : %d    Channel : %d\n", spectraFieldLength, channel);
                    printf("channel %d\n", i);

                    while (fieldOffset < fieldEnd) // reads the sub-fields
                    {
                        uint16_t spectraID;
                        memcpy(&fieldLength, &recvbuf[fieldOffset], 4);
                        fieldOffset += 4;

                        memcpy(&spectraID, &recvbuf[fieldOffset], 2);
                        fieldOffset += 2;

                        //printf("ID = %d      Length = %d\n", spectraID, fieldLength);

                        switch (spectraID)
                        {
                            default:
                                return 2;
                                break;    

                            case 0: // Timestamp
                                memcpy(&allCoresData[i].timestamp, &recvbuf[fieldOffset], 8);
                                fieldOffset += 8;
                                printf("    Timestamp : %" PRIu64 "\n", allCoresData[channel].timestamp);
                                break;

                            case 1: // Sample Number
                                memcpy(&allCoresData[i].sampleNumber, &recvbuf[fieldOffset], 8);
                                fieldOffset += 8;
                                printf("    Sample number : %" PRIu64 "\n", allCoresData[channel].sampleNumber);
                                break;

                            case 2: // Error Status
                                memcpy(&allCoresData[i].errorStatus, &recvbuf[fieldOffset], 2);
                                fieldOffset += 2;
                                printf("    Error status : %d\n", allCoresData[channel].errorStatus);
                                break;

                            case 3: // Spectrum WL
                                // Reads the number of values in the array
                                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                                fieldOffset += 4;

                                 //printf("    Spectrum length : %d\n", arrayLength);

                                // Allocates/reallocates array if necessary
                                if (spectrumWLSize[i] != arrayLength)
                                {
                                    spectrumWLSize[i] = arrayLength;
                                    if (allCoresData[i].spectrumWL != nullptr)
                                        delete[] allCoresData[i].spectrumWL;
                                    if (spectrumWLSize[i] != 0)
                                        allCoresData[i].spectrumWL = new uint32_t[spectrumWLSize[i]];
                                    else
                                        allCoresData[i].spectrumWL = nullptr;
                                    //printf("Pointer : %" PRIu32 , allCoresData[i].spectrumWL);
                                }

                                // fills array
                                memcpy(allCoresData[i].spectrumWL, &recvbuf[fieldOffset], spectrumWLSize[i] * 4);
                                fieldOffset += spectrumWLSize[i] * 4;

                                printf("    Spectrum Wl : %d, %d, %d, %d, %d, %d\n", allCoresData[i].spectrumWL[0], allCoresData[i].spectrumWL[1], allCoresData[i].spectrumWL[2], allCoresData[i].spectrumWL[3], allCoresData[i].spectrumWL[4], allCoresData[i].spectrumWL[5]);
                                break;

                            case 4: // Spectrum Powers
                                // Reads the number of values in the array
                                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                                fieldOffset += 4;

                                // Allocates/reallocates array if necessary
                                if (spectrumPowerSize[i] != arrayLength)
                                {
                                    spectrumPowerSize[i] = arrayLength;
                                    if (allCoresData[i].spectrumPower != nullptr)
                                        delete[] allCoresData[i].spectrumPower;
                                    if (spectrumPowerSize[i] != 0)
                                        allCoresData[i].spectrumPower = new uint16_t[spectrumPowerSize[i]];
                                    else
                                        allCoresData[i].spectrumPower = nullptr;
                                }

                                // fills array
                                memcpy(allCoresData[i].spectrumPower, &recvbuf[fieldOffset], spectrumPowerSize[i] * 2);
                                fieldOffset += spectrumPowerSize[i] * 2;

                                printf("    Spectrum power : %d, %d, %d, %d, %d, %d\n", allCoresData[i].spectrumPower[0], allCoresData[i].spectrumPower[1], allCoresData[i].spectrumPower[2], allCoresData[i].spectrumPower[3], allCoresData[i].spectrumPower[4], allCoresData[i].spectrumPower[5]);
                                break;

                            case 5: // Peaks WL
                                // Reads the number of values in the array
                                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                                fieldOffset += 4;

                                // Allocates/reallocates array if necessary
                                if (peaksWLSize[i] != arrayLength)
                                {
                                    peaksWLSize[i] = arrayLength;
                                    if (allCoresData[i].peaksWL != nullptr)
                                        delete[] allCoresData[i].peaksWL;
                                    if (peaksWLSize != 0)
                                        allCoresData[i].peaksWL = new uint32_t[peaksWLSize[i]];
                                    else
                                        allCoresData[i].peaksWL = nullptr;
                                }

                                // fills array
                                memcpy(allCoresData[i].peaksWL, &recvbuf[fieldOffset], peaksWLSize[i] * 4);
                                fieldOffset += peaksWLSize[i] * 4;

                                printf("    Peaks WL : %d, %d, %d, %d, %d, %d\n", allCoresData[i].peaksWL[0], allCoresData[i].peaksWL[1], allCoresData[i].peaksWL[2], allCoresData[i].peaksWL[3], allCoresData[i].peaksWL[4], allCoresData[i].peaksWL[5]);
                                break;

                            case 6: // Peaks power
                                // Reads the number of values in the array
                                memcpy(&arrayLength, &recvbuf[fieldOffset], 4);
                                fieldOffset += 4;

                                // Allocates/reallocates array if necessary
                                if (peaksPowerSize[i] != arrayLength)
                                {
                                    peaksPowerSize[i] = arrayLength;
                                    if (allCoresData[i].peaksPower != nullptr)
                                        delete[] allCoresData[i].peaksPower;
                                    if (arrayLength != 0)
                                        allCoresData[i].peaksPower = new uint16_t[arrayLength];
                                    else
                                        allCoresData[i].peaksPower = nullptr;
                                }

                                // fills array
                                memcpy(allCoresData[i].peaksPower, &recvbuf[fieldOffset], peaksPowerSize[i] * 2);
                                fieldOffset += peaksPowerSize[i] * 2;

                                printf("    Peaks power : %d, %d, %d, %d, %d, %d\n", allCoresData[i].peaksPower[0], allCoresData[i].peaksPower[1], allCoresData[i].peaksPower[2], allCoresData[i].peaksPower[3], allCoresData[i].peaksPower[4], allCoresData[i].peaksPower[5]);
                                break;

                            case 7: // Spectrometer Temperature
                                memcpy(&allCoresData[i].spectroTemperature, &recvbuf[fieldOffset], 4);
                                fieldOffset += 4;
                                break;

                        }
                    }
                }
                break;
            }
        }
        
        /**
         * @brief Wall-clock timestamp in seconds (with fractional sub-second precision).
         *
         * This variable holds the current system (wall-clock) time expressed as a floating-point
         * number of seconds since the POSIX/system epoch. Use std::chrono::system_clock to obtain
         * a timestamp anchored to the system epoch and std::chrono::duration<double> for portable
         * floating-point seconds representation.
         *
         * Recommended construction (example):
         *   std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
         *
         * Notes:
         * - Prefer std::chrono::system_clock over std::chrono::high_resolution_clock when you need
         *   an epoch-based timestamp; high_resolution_clock may not be epoch-based on all platforms.
         * - Use std::chrono::duration<double> instead of double_t for portability and clarity.
         * - The returned value is affected by system clock adjustments; if you need a monotonic
         *   interval measurement, use std::chrono::steady_clock.
         */
        double_t machine_timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::tuple<double_t, float*, float*, float*> dataPoint; 
        dataPoint = std::make_tuple(machine_timestamp, curvature, angle, shape);
        dataToSave.push_back(dataPoint);

        return 0;
    }
    else if (iResult == 0)
    {
        isConnected = NOT_CONNECTED;
        return 1;
    }
    else
    {
        return WSAGetLastError();
    }
    return 0;
}

int StreamClient::closeStream()
{
    delete[] recvbuf;

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

    isConnected = NOT_CONNECTED;

    return 0;
}