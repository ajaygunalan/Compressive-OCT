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

using namespace cv;
using namespace std;



void ExportAScanImage(string n, double NumOfAScan, double PosX, double PosY, const string& filepath) {
	char message[1024];

	OCTDeviceHandle Dev = initDevice();
	ProbeHandle Probe = initProbe(Dev, "Probe_Standard_OCTG_LSM04.ini");
	ProcessingHandle Proc = createProcessingForDevice(Dev);

	RawDataHandle Raw = createRawData();
	DataHandle AScanDH = createData();

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

	ScanPatternHandle Pattern = createAScanPattern(Probe, NumOfAScan, PosX, PosY);

	startMeasurement(Dev, Pattern, Acquisition_AsyncFinite);
	getRawData(Dev, Raw);
	setProcessedDataOutput(Proc, AScanDH);
	executeProcessing(Proc, Raw);
	stopMeasurement(Dev);

	// Convert PosX and PosY to integer to truncate any decimal portions
	int posXInt = static_cast<int>(PosX);
	int posYInt = static_cast<int>(PosY);

	// Construct the filename with the naming convention
	std::string namePrefix = "oct";
	if (posXInt < 0 || posYInt < 0) {
		namePrefix += "_";
	}
	std::string fileName = namePrefix + std::to_string(std::abs(posXInt)) + std::to_string(std::abs(posYInt));

	// Save .csv file
	string csvPath = filepath + fileName + ".csv";
	const char* cstrCSV = csvPath.c_str();
	exportData(AScanDH, DataExport_CSV, cstrCSV);


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


	cv::destroyAllWindows();

	clearScanPattern(Pattern);
	clearData(AScanDH);
	clearRawData(Raw);
	clearProcessing(Proc);
	closeProbe(Probe);
	closeDevice(Dev);
}

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
	string filepath = "C:\\Ajay_OCT\\OCTAssistedSurgicalLaserbot\\data\\cs\\ablated_plaster\\";
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

	std::vector<std::pair<double, double>> scanLocations = horizontalLine;

	for (const auto& location : scanLocations) {
		double PosX = location.first;
		double PosY = location.second;
		string n = std::to_string(static_cast<int>(PosX));
		ExportAScanImage(n, NumOfAScan, PosX, PosY, filepath);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
