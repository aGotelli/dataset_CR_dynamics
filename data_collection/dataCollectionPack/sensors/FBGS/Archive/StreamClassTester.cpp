// StreamClassTester.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "StreamClient.h"
#include <string>
#include <inttypes.h>
#include <stdexcept>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
int main(int argc, char* argv[])
{
	if (argc != 3)
		throw std::invalid_argument("Usage: StreamClassTester <filename> <runtime>");
	double runtime = std::stod(argv[1]);
	std::string filename = argv[2];
	
	StreamClient streamClient("localhost", "50012", runtime);
	printf("Client instantiated\n");
	printf("Connection :%d\n", streamClient.connectStream());
    printf("FBGS: Reading samples for %.2f seconds and saving to %s\n", runtime, filename.c_str());

	auto start = std::chrono::high_resolution_clock::now();
	
	while (streamClient.isConnected && 
	       std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count() < runtime){
		streamClient.readStream();
	}

	streamClient.closeStream();

	printf("FBGS: Stream closed, saving data to %s\n", filename.c_str());

	// Save to file (explicitly overwrite existing file)
	std::ofstream outFile(filename, std::ios::out | std::ios::trunc);
	if (!outFile) {
		throw std::runtime_error("Failed to open file");
	}
	
	// Set fixed-point notation to avoid scientific format
	outFile << std::fixed;

	// Write CSV headers
	outFile << "Timestamp";
	for (int i = 0; i < streamClient.curvatureArrayLength; ++i) {
		outFile << ",Curvature_" << i;
	}
	for (int i = 0; i < streamClient.angleArrayLength; ++i) {
		outFile << ",Angle_" << i;
	}
	for (int i = 0; i < streamClient.shapeArrayLength; ++i) {
		outFile << ",Shape_" << i;
	}
	outFile << "\n";

	std::vector<std::tuple<double_t, std::vector<float>, std::vector<float>, std::vector<float>>> data;
	// std::vector<std::tuple<double_t, float*, float*, float*>> data;
	streamClient.getData(data);
	for (const auto& entry : data) {
		double_t timestamp = std::get<0>(entry);
		const std::vector<float>& curvature = std::get<1>(entry);
		// float* curvature = std::get<1>(entry);
		// float* angle = std::get<2>(entry);
		// float* shape = std::get<3>(entry);
		const std::vector<float>& angle = std::get<2>(entry);
		const std::vector<float>& shape = std::get<3>(entry);


		outFile << timestamp;
		for (size_t i = 0; i < curvature.size(); ++i) {       	//for (int i = 0; i < streamClient.curvatureArrayLength; ++i) {
			outFile << "," << curvature[i];
		}
		for (size_t i = 0; i < angle.size(); ++i) { 			//for (int i = 0; i < streamClient.angleArrayLength; ++i) {
			outFile << "," << angle[i];
		}
		for (size_t i = 0; i < shape.size(); ++i) {    			//for (int i = 0; i < streamClient.shapeArrayLength; ++i) {
			outFile << "," << shape[i];
		}
		outFile << "\n";
	}

size_t row_index = data.size();
double duration_sec = runtime;
std::cout << "FBGS: data saved to " << filename << " (" << row_index << " rows)\n";
if (duration_sec > 0.0) {
	std::cout << "Actual sampling rate: " << std::fixed << std::setprecision(1)
			  << (static_cast<double>(row_index) / duration_sec) << " samples/second\n";
} else {
	std::cout << "Actual sampling rate: N/A (duration <= 0)\n";
}
return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
