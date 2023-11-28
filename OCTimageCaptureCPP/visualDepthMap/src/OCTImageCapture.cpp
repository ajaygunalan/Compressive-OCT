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
};


ScanResult getSurfaceFrom3DScan(int AScansPerBScan, double LengthOfBScan, int BScansPerVolume, double WidthOfVolume) {
    char message[1024];
    OCTDeviceHandle Dev = initDevice();
    ProbeHandle Probe = initProbe(Dev, "Probe_Standard_OCTG_LSM04.ini");
    ProcessingHandle Proc = createProcessingForDevice(Dev);

    RawDataHandle RawVolume = createRawData();
    DataHandle Volume = createData();
    DataHandle Surface = createData();

    if (getError(message, 1024)) {
        std::cout << "ERROR: " << message << std::endl;
        _getch();
        return ScanResult{ nullptr, -1.0, -1.0 };  // Indicate error with negative times
    }

    setDevicePreset(Dev, CATEGORY_SPEED_SENSITIVITY, Probe, Proc, PRESET_HIGH_SPEED_146kHz);
    int AScanAveraging = 3;
    setProbeParameterInt(Probe, Probe_Oversampling, AScanAveraging); // this results in a repetition of each scan point in the B-scan
    setProcessingParameterInt(Proc, Processing_AScanAveraging, AScanAveraging);

    ScanPatternHandle Pattern = createVolumePattern(Probe, LengthOfBScan, AScansPerBScan, WidthOfVolume, BScansPerVolume, ScanPattern_ApoOneForAll, ScanPattern_AcqOrderAll);
    

    // the apodization spectra are acquired now
    //measureApodizationSpectra(Dev, Probe, Proc);

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

    // Clean up
    clearScanPattern(Pattern);
    clearData(Volume);
    clearRawData(RawVolume);
    clearProcessing(Proc);
    closeProbe(Probe);
    closeDevice(Dev);

    return ScanResult{ Surface, actualTime, expectedTime, numOfLostBScan};
}


void performScanAndExport(const std::string& folderLocation, std::string fileName,
    int numAScans, int numBScans, double LengthOfBScan, double WidthOfVolume,
    double BscanCompressionRatio, double CscanCompressionRatio) {
    ScanResult result = getSurfaceFrom3DScan(numAScans, LengthOfBScan, numBScans, WidthOfVolume);
    if (result.surface) {
        exportData(result.surface, DataExport_CSV, (folderLocation + fileName + ".csv").c_str());
    }
    std::ofstream metaFile(folderLocation + fileName + "_meta.csv");
    if (metaFile.is_open()) {
        metaFile << numBScans << "\n";
        metaFile << numAScans << "\n";
        metaFile << BscanCompressionRatio << "\n";
        metaFile << CscanCompressionRatio << "\n";
        metaFile << result.actualTime << "\n";
        metaFile << LengthOfBScan << "\n";
        metaFile << WidthOfVolume << "\n";
        metaFile << result.numOfLostBScan << "\n";
        metaFile << result.expectedTime;
        metaFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing metadata\n";
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " trialNum\n";
        return -1;
    }
    std::string trialNum = argv[1];
    double LengthOfBScan = 10.0; // mm
    double WidthOfVolume = 10.0; // mm
    std::string baseFolder = "C:\\Ajay_OCT\\OCT-Guided-AutoCALM\\data\\getDepthFromSparse3Doct\\";
    std::string folderLocation = baseFolder + trialNum + "\\";

    // Create the directory if it does not exist
    if (!fs::exists(folderLocation)) {
        fs::create_directories(folderLocation);
    }

    int NumAScansPerBScanReference = 256;
    int NumBScansPerVolumeReference = 100;

    for (double BscanCompressionRatio = 1.0; BscanCompressionRatio >= 0.05; BscanCompressionRatio -= 0.05) {
        for (double CscanCompressionRatio = 1.0; CscanCompressionRatio >= 0.05; CscanCompressionRatio -= 0.05) {
            int numAScansPerBScan = static_cast<int>(NumAScansPerBScanReference * BscanCompressionRatio);
            int numBScansPerVolume = static_cast<int>(NumBScansPerVolumeReference * CscanCompressionRatio);

            std::string fileName = "CompressionRatio_Bscan" + to_string(BscanCompressionRatio) +
                                   "_AScan" + to_string(CscanCompressionRatio);

            performScanAndExport(folderLocation, fileName, numAScansPerBScan, numBScansPerVolume, LengthOfBScan, WidthOfVolume, BscanCompressionRatio, CscanCompressionRatio);
        }
    }

    return 0;
}