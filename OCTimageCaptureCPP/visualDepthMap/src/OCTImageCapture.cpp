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



void ExportAScanMultiLoc(const std::vector<std::pair<double, double>>& scanLocations, double NumOfAScan, const string& filepath) {
	char message[1024];

	OCTDeviceHandle Dev = initDevice();
	ProbeHandle Probe = initProbe(Dev, "Probe_Standard_OCTG_LSM04.ini");

	// Error handling
	if (getError(message, 1024)) {
		cout << "ERROR: " << message << endl;
		return;
	}

	ProcessingHandle Proc = createProcessingForDevice(Dev);
	setDevicePreset(Dev, 0, Probe, Proc, 0);

	for (const auto& location : scanLocations) {
		double PosX = location.first;
		double PosY = location.second;

		
		RawDataHandle Raw = createRawData();
		DataHandle AScanDH = createData();

		ScanPatternHandle Pattern = createAScanPattern(Probe, NumOfAScan, PosX, PosY);
		startMeasurement(Dev, Pattern, Acquisition_AsyncFinite);
		getRawData(Dev, Raw);
		setProcessedDataOutput(Proc, AScanDH);
		executeProcessing(Proc, Raw);
		stopMeasurement(Dev);


		// Verifying that each A-scan has 1024 data points
		int aScanSize = getDataPropertyInt(AScanDH, Data_Size1);
		if (aScanSize != 1024) {
			cout << "Unexpected A-scan size: " << aScanSize << endl;
			return;
		}
		logDataProperties(AScanDH);

		int posXInt = static_cast<int>(PosX);
		int posYInt = static_cast<int>(PosY);
		std::string fileName = "oct";
		fileName += (posXInt < 0) ? "_" : "";
		fileName += std::to_string(std::abs(posXInt));
		fileName += (posYInt < 0) ? "_" : "";
		fileName += std::to_string(std::abs(posYInt));
		string csvPath = filepath + fileName + ".csv";
		const char* cstrCSV = csvPath.c_str();

		exportData(AScanDH, DataExport_CSV, cstrCSV);
		clearScanPattern(Pattern);
		clearData(AScanDH);
		clearRawData(Raw);
		
	}

	clearProcessing(Proc);
	closeProbe(Probe);
	closeDevice(Dev);
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
		return;
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
			return;
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



	/**
	visualizeScanPatternOnDevice(Dev, Probe, Pattern, TRUE);

	getCameraImage(Dev, VideoImg);
	visualizeScanPatternOnImage(Probe, Pattern, VideoImg);

	copyColoredData(VideoImg, VideoImgCopy);
	unsigned long* data = getColoredDataPtr(VideoImgCopy);

	int width = 648, height = 484;
	cv::Mat videoImagecv = cv::Mat(height, width, CV_8UC3);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			unsigned long pixelValue = data[i * width + j];
			cv::Vec3b& pixel = videoImagecv.at<cv::Vec3b>(i, j);
			pixel[0] = (pixelValue >> 16) & 0xFF; // Blue channel
			pixel[1] = (pixelValue >> 8) & 0xFF;  // Green channel
			pixel[2] = pixelValue & 0xFF;         // Red channel
		}
	}
	data = nullptr;


	string jpgPath = filepath + "scan_pattern" + fileName + ".jpg";
	const char* cstrJPG = jpgPath.c_str();
	cv::imwrite(cstrJPG, videoImagecv);
	**/

void ExportBScanImage(string n, double BScanRangeMM, double ShiftX, double ShiftY, double Angle_rad, const string& filepath) {
	char message[1024];

	OCTDeviceHandle Dev = initDevice();
	ProbeHandle Probe = initProbe(Dev, "Probe_Standard_OCTG_LSM04.ini");
	ProcessingHandle Proc = createProcessingForDevice(Dev);

	RawDataHandle Raw = createRawData();
	DataHandle BScanDH = createData();
	ColoredDataHandle VideoImg = createColoredData();
	ColoredDataHandle VideoImgCopy = createColoredData();

	if (getError(message, 1024)) {
		cout << "ERROR: " << message << endl;
		(void)getchar();
		return;
	}

	getNumberOfDevicePresetCategories(Dev);
	// The scan speed of SD-OCT systems can be changed. A better image quality can be obtained with a longer integration time and therefore lower scan speed.
	// Preset 0 is the default scan speed followed by the highest. Please note to adjust the reference intensity on your scanner manually.
	// The number of available device presets can be obtained with #getNumberOfDevicePresets and the description of each preset with #getDevicePresetDescription
	int NumberOfDevicePresets = getNumberOfDevicePresets(Dev, 0);
	cout << getDevicePresetDescription(Dev, 0, 0) << endl;
	setDevicePreset(Dev, 0, Probe, Proc, 0);

	ScanPatternHandle Pattern = createBScanPattern(Probe, BScanRangeMM , 1024);
	shiftScanPattern(Pattern, ShiftX, ShiftY);
	rotateScanPattern(Pattern, Angle_rad);

	startMeasurement(Dev, Pattern, Acquisition_AsyncFinite);
	getRawData(Dev, Raw);
	setProcessedDataOutput(Proc, BScanDH);
	executeProcessing(Proc, Raw);
	stopMeasurement(Dev);

	ColoringHandle Coloring = createColoring32Bit(ColorScheme_BlackAndWhite, Coloring_RGBA);
	// set the boundaries for the colormap, 0.0 as lower and 70.0 as upper boundary are a good choice normally.
	setColoringBoundaries(Coloring, 0.0, 70.0);
	// Exports the processed data to an image with the specified slice normal direction since this will result in 2D-images.
	// To get the B-scan in one image with depth and scan field as axes for a single B-scan #Direction_3 is chosen.

	string fullpath = filepath + "oct" + n + ".jpg";
	const char* cstr = fullpath.c_str();
	exportDataAsImage(BScanDH, Coloring, ColoredDataExport_JPG, Direction_3, cstr, ExportOption_DrawScaleBar | ExportOption_DrawMarkers | ExportOption_UsePhysicalAspectRatio);
	//cv::Mat OCTimage = cv::imread("C:/Ajay_OCT/visualDepthMap/data/oct.jpg", cv::IMREAD_COLOR);
	//std::thread t(ProcessAndSaveImage, Dev, Probe, Pattern, n);
	//t.detach();
	//ColoredDataHandle VideoImg = createColoredData();
	//ColoredDataHandle VideoImgCopy = createColoredData();

	while (getColoredDataPtr(VideoImg) == nullptr) {
		getCameraImage(Dev, VideoImg);
	}
	visualizeScanPatternOnImage(Probe, Pattern, VideoImg);

	copyColoredData(VideoImg, VideoImgCopy);
	unsigned long* data = getColoredDataPtr(VideoImgCopy);

	int width = 648, height = 484;
	cv::Mat videoImagecv = cv::Mat(height, width, CV_8UC3);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			unsigned long pixelValue = data[i * width + j];
			cv::Vec3b& pixel = videoImagecv.at<cv::Vec3b>(i, j);
			pixel[0] = (pixelValue >> 16) & 0xFF; // Blue channel
			pixel[1] = (pixelValue >> 8) & 0xFF;  // Green channel
			pixel[2] = pixelValue & 0xFF;         // Red channel
		}
	}
	data = nullptr;

	fullpath = filepath + "scan_pattern" + n + ".jpg";
	cstr = fullpath.c_str();
	cv::imwrite(cstr, videoImagecv);


	clearScanPattern(Pattern);
	clearData(BScanDH);
	clearRawData(Raw);
	clearColoredData(VideoImg);
	clearProcessing(Proc);
	closeProbe(Probe);
	closeDevice(Dev);
}

void writeSurfaceValuesToCSV(
	const std::unordered_map<std::pair<double, double>, float>& surfaceValues,
	const std::string& filename
) {
	std::ofstream outFile(filename);  // Open a file stream for writing

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
	string filepath = "C:\\Ajay_OCT\\OCTAssistedSurgicalLaserbot\\data\\cs\\ablated_plaster\\1\\";
	double NumOfAScan = 5;

	std::vector<std::pair<double, double>> horizontalLine = {
		{7.0, 0.0},
		{6.0, 0.0},
		{5.0, 0.0},
		{4.0, 0.0},
		{3.0, 0.0},
		{2.0, 0.0},
		{1.0, 0.0},
		{0.0, 0.0},
		{-1.0, 0.0},
		{-2.0, 0.0},
		{-3.0, 0.0},
		{-4.0, 0.0},
		{-5.0, 0.0},
		{-6.0, 0.0},
		{-7.0, 0.0}
	};

	std::vector<std::pair<double, double>> verticalline = {
	{-1.0, 7.0},
	{-1.0, 6.0},
	{-1.0, 5.0},
	{-1.0, 4.0},
	{-1.0, 3.0},
	{-1.0, 2.0},
	{-1.0, 1.0},
	{-1.0, 0.0},
	{-1.0, -1.0},
	{-1.0, -2.0},
	{-1.0, -3.0},
	{-1.0, -4.0},
	{-1.0, -5.0},
	{-1.0, -6.0},
	{-1.0, -7.0}
	};


	std::vector<std::pair<double, double>> singlePoint = {
	{0.0, 0.0}
	};

	std::vector<std::pair<double, double>> scanLocations = singlePoint;

	auto surfaceValues = AScanMultiLoc(singlePoint, NumOfAScan);
	writeSurfaceValuesToCSV(surfaceValues, "surfaceValues.csv");  // Call the function to write data to CSV


	std::this_thread::sleep_for(std::chrono::seconds(1));
}
