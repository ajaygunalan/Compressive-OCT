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
#include <filesystem>
#include <iomanip>
namespace fs = std::filesystem;

// #include <matplotlibcpp.h>

#define PI 3.14159265358979323846
#define DEBUG
#define PRESET_DEFAULT_76kHz 0
#define PRESET_HIGH_SPEED_146kHz 1
#define PRESET_MEDIUM_SENSITIVITY_76kHz 2
#define PRESET_VIDEO_RATE_28kHz 3
#define PRESET_HIGH_SENSITIVITY_10kHz 4
#define CATEGORY_SPEED_SENSITIVITY 0

// #ifdef DEBUG
// namespace plt = matplotlibcpp;  
// #endif

using namespace cv;
using namespace std;


struct ScanResult {
    DataHandle surface;
    double actualTime;
    double expectedTime;
    int numOfLostBScan;
    double BscanCompressionRatio;
    double CscanCompressionRatio;
    int count;
    int numAScansPerBScan;
    int numBScansPerVolume;
};


void processScanData(const ScanResult& result, const std::string& folderLocation) {

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "ScanNum_" <<result.count << "_CR_BScan_" << result.BscanCompressionRatio
        << "_CScan_" << result.CscanCompressionRatio;
    std::string fileName = oss.str();

    // Exporting surface data
    std::string surfaceFileName = folderLocation + fileName + "_surface.csv";
    if (result.surface) {
        exportData(result.surface, DataExport_CSV, surfaceFileName.c_str());
    }
    else {
        std::cerr << "No surface data to export.\n";
    }

    // Writing metadata to a file
    std::string metaFileName = folderLocation + fileName + "_meta.csv";
    std::ofstream metaFile(metaFileName);
    if (metaFile.is_open()) {
        metaFile << "Number of BScans Per Volume: " << result.numBScansPerVolume << "\n";
        metaFile << "Number of AScans Per BScan: " << result.numAScansPerBScan << "\n";
        metaFile << "Actual Time: " << result.actualTime << "\n";
        metaFile << "Expected Time: " << result.expectedTime << "\n";
        metaFile << "Number of Lost B-Scans: " << result.numOfLostBScan << "\n";
        metaFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing metadata: " << metaFileName << "\n";
    }
}

void getSurfaceFrom3DScan(const std::string& folderLocation, int NumAScansPerBScanReference, double LengthOfBScan, int NumBScansPerVolumeReference, double WidthOfVolume) {

    char message[1024];
    OCTDeviceHandle Dev = initDevice();
    ProbeHandle Probe = initProbe(Dev, "Probe_Standard_OCTG_LSM04.ini");
    ProcessingHandle Proc = createProcessingForDevice(Dev);

    if (getError(message, 1024)) {
        std::cout << "ERROR: " << message << std::endl;
        _getch();
    }

    setDevicePreset(Dev, CATEGORY_SPEED_SENSITIVITY, Probe, Proc, PRESET_HIGH_SPEED_146kHz);
    int AScanAveraging = 3;
    setProbeParameterInt(Probe, Probe_Oversampling, AScanAveraging); // this results in a repetition of each scan point in the B-scan
    setProcessingParameterInt(Proc, Processing_AScanAveraging, AScanAveraging);


    int count = 1;
    std::vector<std::tuple<double, double>> compressionPairs = {
    {1.0, 1.0}, {0.5, 0.5},
                {1.0, 0.9}, {1.0, 0.8}, {1.0, 0.5},
    {0.9, 1.0}, {0.9, 0.9}, {0.9, 0.8}, {0.9, 0.5},
    {0.8, 1.0}, {0.8, 0.9}, {0.8, 0.8}, {0.8, 0.5},
    {0.5, 1.0}, {0.5, 0.9}, {0.5, 0.8}, 
    };

    for (const auto& [BscanCompressionRatio, CscanCompressionRatio] : compressionPairs) {
        int numAScansPerBScan = static_cast<int>(NumAScansPerBScanReference * BscanCompressionRatio);
        int numBScansPerVolume = static_cast<int>(NumBScansPerVolumeReference * CscanCompressionRatio);

        RawDataHandle RawVolume = createRawData();
        DataHandle Volume = createData();
        DataHandle Surface = createData();
        ScanPatternHandle Pattern = createVolumePattern(Probe, LengthOfBScan, numAScansPerBScan, WidthOfVolume, numBScansPerVolume, ScanPattern_ApoOneForAll, ScanPattern_AcqOrderAll);

        auto start = std::chrono::high_resolution_clock::now();
        startMeasurement(Dev, Pattern, Acquisition_AsyncContinuous);
        getRawData(Dev, RawVolume);
        setProcessedDataOutput(Proc, Volume);
        executeProcessing(Proc, RawVolume);
        stopMeasurement(Dev);
        auto stop = std::chrono::high_resolution_clock::now();


        int numOfLostBScan = getRawDataPropertyInt(RawVolume, RawData_LostFrames);
        std::chrono::duration<double> actualTimeDuration = stop - start;
        double actualTime = actualTimeDuration.count();
        double expectedTime = expectedAcquisitionTime_s(Pattern, Dev);

        determineSurface(Volume, Surface);

        ScanResult result;
        result.surface = Surface;
        result.actualTime = actualTimeDuration.count();
        result.expectedTime = expectedTime;
        result.numOfLostBScan = numOfLostBScan;
        result.BscanCompressionRatio = BscanCompressionRatio;
        result.CscanCompressionRatio = CscanCompressionRatio;
        result.numAScansPerBScan = numAScansPerBScan;
        result.numBScansPerVolume = numBScansPerVolume;
        result.count = count;

        processScanData(result, folderLocation);  // Process the data from this iteration

        // Clean up
        clearScanPattern(Pattern);
        clearRawData(RawVolume);
        clearData(Volume);
        clearData(Surface);
        count++;
    }
    clearProcessing(Proc);
    closeProbe(Probe);
    closeDevice(Dev);
}


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " trialNum\n";
        return -1;
    }
    std::string trialNum = argv[1];
    std::string baseFolder = "C:\\Ajay_OCT\\OCT-Guided-AutoCALM\\data\\getDepthFromSparse3Doct\\";
    std::string folderLocation = baseFolder + trialNum + "\\";
    if (!fs::exists(folderLocation)) {
        fs::create_directories(folderLocation);
    }

    double LengthOfBScan = 10.0; // mm
    double WidthOfVolume = 10.0; // mm
    int NumAScansPerBScanReference = 300;
    int NumBScansPerVolumeReference = 300;

    getSurfaceFrom3DScan(folderLocation, NumAScansPerBScanReference, LengthOfBScan, NumBScansPerVolumeReference, WidthOfVolume);
    return 0;
}