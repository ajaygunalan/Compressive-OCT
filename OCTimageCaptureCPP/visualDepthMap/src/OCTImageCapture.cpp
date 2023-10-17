#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <conio.h>
#include <SpectralRadar.h>
#include <chrono>
#include <thread>
#include <iostream>
#define PI 3.14159265358979323846
#define DEBUG 

using namespace cv;
using namespace std;


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

// Return type should match the type of surfaceValue in your main function
float estimateSurface(const std::vector<std::vector<float>>& ascanMatrix) {
	// Declare variables you'll use for computation
	float estimatedSurfaceValue;

	// Insert the algorithm to estimate the surface value from ascanMatrix
	// ...

	return estimatedSurfaceValue;
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


void AScanMultiLoc(const std::vector<std::pair<double, double>>& scanLocations, double NumOfAScan, const string& filepath) {
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

		clearScanPattern(Pattern);
		clearData(AScanDH);
		clearRawData(Raw);

	}

	clearProcessing(Proc);
	closeProbe(Probe);
	closeDevice(Dev);
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

	ExportAScanMultiLoc(singlePoint, NumOfAScan, filepath);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	ExportAScanMultiLoc(singlePoint, NumOfAScan, filepath);
	std::this_thread::sleep_for(std::chrono::seconds(1));
}
