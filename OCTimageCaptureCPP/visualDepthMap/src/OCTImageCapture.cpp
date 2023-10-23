#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <conio.h>
#include <SpectralRadar.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <fstream>
// #include <matplotlibcpp.h>

#define PI 3.14159265358979323846
#define DEBUG

// #ifdef DEBUG
// namespace plt = matplotlibcpp;  
// #endif

using namespace cv;
using namespace std;


template<typename T>
void hash_combine(std::size_t& seed, T const& key) {
	std::hash<T> hasher;
	seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
	template<typename T1, typename T2>
	struct hash<std::pair<T1, T2>> {
		std::size_t operator()(const std::pair<T1, T2>& p) const {
			std::size_t seed = 0;
			hash_combine(seed, p.first);
			hash_combine(seed, p.second);
			return seed;
		}
	};
}



float estimateSurface(const std::vector<std::vector<float>>& ascanMatrix) {
	// Calculate the average of each column (average A-scan)
	std::vector<float> avg_data(ascanMatrix[0].size(), 0);
	for (const auto& scan : ascanMatrix) {
		for (size_t i = 0; i < scan.size(); ++i) {
			avg_data[i] += scan[i];
		}
	}

	for (auto& val : avg_data) {
		val /= ascanMatrix.size();
	}

	// Generate corresponding depth values based on given SpacingZ
	const float SpacingZ = 0.003455f;
	std::vector<float> depth_values(avg_data.size());
	std::iota(depth_values.begin(), depth_values.end(), 0);
	for (auto& val : depth_values) {
		val *= SpacingZ;
	}

	// Gaussian smoothing, assuming std_dev = 5
	const int std_dev = 5;
	cv::Mat avg_data_mat(avg_data.size(), 1, CV_32F, avg_data.data());  // Convert avg_data to cv::Mat
	cv::Mat smoothed_data_mat;
	cv::GaussianBlur(avg_data_mat, smoothed_data_mat, cv::Size(0, 0), std_dev, std_dev);

	// Convert smoothed_data_mat back to std::vector<float>
	std::vector<float> smoothed_data(smoothed_data_mat.begin<float>(), smoothed_data_mat.end<float>());

	// Find peak intensity and corresponding depth value
	auto max_element_iter = std::max_element(smoothed_data.begin(), smoothed_data.end());
	float peak_intensity = *max_element_iter;
	int peak_index = std::distance(smoothed_data.begin(), max_element_iter);
	float depth = depth_values[peak_index];

	/** WILL LINK THI SETUP LATER
	#ifdef DEBUG
	// Plotting and saving plots, requires matplotlibcpp
	plt::figure();
	plt::plot(depth_values, avg_data, "b-", { {"label", "Average Intensity"} });
	plt::plot(depth_values, smoothed_data, "r-", { {"label", "Gaussian Smoothing"} });
	plt::scatter(std::vector<float>{depth}, std::vector<float>{peak_intensity});
	plt::legend();
	plt::save("./depth_vs_intensity.png");
	#endif
	**/

	return depth;
}


void logDataProperties(DataHandle AScanDH) {
#ifdef DEBUG
	int dataDimensions = getDataPropertyInt(AScanDH, Data_Dimensions);
	int dataSize1 = getDataPropertyInt(AScanDH, Data_Size1);
	int dataSize2 = getDataPropertyInt(AScanDH, Data_Size2);
	int dataSize3 = getDataPropertyInt(AScanDH, Data_Size3);
	int dataNumberOfElements = getDataPropertyInt(AScanDH, Data_NumberOfElements);
	int dataSizeInBytes = getDataPropertyInt(AScanDH, Data_SizeInBytes);
	int dataBytesPerElement = getDataPropertyInt(AScanDH, Data_BytesPerElement);

	cout << "Data Properties: Dimensions=" << dataDimensions
		<< ", Size1=" << dataSize1
		<< ", Size2=" << dataSize2
		<< ", Size3=" << dataSize3
		<< ", NumberOfElements=" << dataNumberOfElements
		<< ", SizeInBytes=" << dataSizeInBytes
		<< ", BytesPerElement=" << dataBytesPerElement << endl;
#endif
}


std::unordered_map<std::pair<double, double>, float> AScanMultiLoc(
	const std::vector<std::pair<double, double>>& scanLocations,
	double NumOfAScan
) {
	char message[1024];
	// Create an unordered_map to store the surface values
	std::unordered_map<std::pair<double, double>, float> surfaceValues;

	OCTDeviceHandle Dev = initDevice();
	ProbeHandle Probe = initProbe(Dev, "Probe_Standard_OCTG_LSM04.ini");

	// Error handling
	if (getError(message, 1024)) {
		cout << "ERROR: " << message << endl;
	}

	ProcessingHandle Proc = createProcessingForDevice(Dev);
	setDevicePreset(Dev, 0, Probe, Proc, 0);

	for (const auto& location : scanLocations) {
		std::vector<std::vector<float>> ascanMatrix;

		double PosX = location.first;
		double PosY = location.second;


		RawDataHandle Raw = createRawData();
		DataHandle AScanDH = createData();

		ScanPatternHandle Pattern = createAScanPattern(Probe, NumOfAScan, PosX, PosY);

		std::this_thread::sleep_for(std::chrono::milliseconds(250));; // Before measuement wait for 250 ms . This is to prevent Galvo scanners from daamged.
		startMeasurement(Dev, Pattern, Acquisition_AsyncFinite);
		getRawData(Dev, Raw);
		setProcessedDataOutput(Proc, AScanDH);
		executeProcessing(Proc, Raw);
		stopMeasurement(Dev);


		// Retrieve dimensions and sizes
		int AScanSize = getDataPropertyInt(AScanDH, Data_Size1);  // Should be 1024
		int numAScan = getDataPropertyInt(AScanDH, Data_Size2);  // Should be 5

		// Check retrieved sizes
		if (AScanSize != 1024 || numAScan != 5) {
			std::cerr << "Data dimensions do not match expected values. Exiting." << std::endl;
		}

		logDataProperties(AScanDH);

		// Retrieve and store processed data
		float* processedData = getDataPtr(AScanDH);
		for (int i = 0; i < numAScan; ++i) {
			std::vector<float> singleAScan(processedData + i * AScanSize, processedData + (i + 1) * AScanSize);
			ascanMatrix.push_back(singleAScan);
		}

		float surfaceValue = estimateSurface(ascanMatrix);

		// Store the surface value in the map with the scan location as the key
		surfaceValues[location] = surfaceValue;

		clearScanPattern(Pattern);
		clearData(AScanDH);
		clearRawData(Raw);

		

	}

	clearProcessing(Proc);
	closeProbe(Probe);
	closeDevice(Dev);

	return surfaceValues;
}


void writeSurfaceValuesToCSV(
	const std::unordered_map<std::pair<double, double>, float>& surfaceValues,
	const std::string& directory,
	const std::string& filename
) {
	// Ensure the directory string ends with a backslash
	std::string fullPath = directory;
	if (fullPath.back() != '\\') {
		fullPath += '\\';
	}
	fullPath += filename;  // Concatenate the directory and filename

	std::ofstream outFile(fullPath);  // Open a file stream for writing

	if (!outFile.is_open()) {  // Check if the file opened successfully
		std::cerr << "Failed to open the file for writing." << std::endl;
		return;
	}

	outFile << "PosX,PosY,SurfaceValue\n";  // Write the header row to the CSV file

	for (const auto& entry : surfaceValues) {  // Iterate through the map
		outFile << entry.first.first << "," << entry.first.second << "," << entry.second << "\n";  // Write data to the CSV file
	}

	outFile.close();  // Close the file stream when done
}

int main(int argc, char* argv[]) {
	std::string directory = "C:\\Ajay_OCT\\OCTAssistedSurgicalLaserbot\\data\\3rdYeraReport\\";;
	double NumOfAScan = 5;

	std::vector<std::pair<double, double>> csUniformRasterTwoLines = {
	{-5.0, 5.0},
	{-4.375, 5.0},
	{-3.75, 5.0},
	{-3.1250000000000004, 5.0},
	{-2.5, 5.0},
	{-1.875, 5.0},
	{-1.2500000000000004, 5.0},
	{-0.6249999999999994, 5.0},
	{0.0, 5.0},
	{0.6249999999999994, 5.0},
	{1.2500000000000004, 5.0},
	{1.875, 5.0},
	{2.500000000000001, 5.0},
	{3.1250000000000004, 5.0},
	{3.75, 5.0},
	{4.375000000000001, 5.0},
	{5.0, 5.0},
	{-5.0, 4.375},
	{-4.375, 4.375},
	{-3.75, 4.375},
	{-3.1250000000000004, 4.375},
	{-2.5, 4.375},
	{-1.875, 4.375},
	{-1.2500000000000004, 4.375},
	{-0.6249999999999994, 4.375},
	{0.0, 4.375},
	{0.6249999999999994, 4.375},
	{1.2500000000000004, 4.375},
	{1.875, 4.375},
	{2.500000000000001, 4.375},
	{3.1250000000000004, 4.375},
	{3.75, 4.375},
	{4.375000000000001, 4.375},
	{5.0, 4.375}
	};


	std::vector<std::pair<double, double>> singlePoint = {
	{0.0, 0.0}
	};

	std::vector<std::pair<double, double>> scanLocations = csUniformRasterTwoLines;

	auto surfaceValues = AScanMultiLoc(scanLocations, NumOfAScan);
	writeSurfaceValuesToCSV(surfaceValues, directory, "csUniformRasterTwoLines.csv");  


	std::this_thread::sleep_for(std::chrono::seconds(1));
}