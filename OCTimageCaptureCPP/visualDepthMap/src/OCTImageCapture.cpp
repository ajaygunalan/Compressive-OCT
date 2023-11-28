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
    int BscanCompressionRatio;
    int CscanCompressionRatio;
};


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


    for (double BscanCompressionRatio = 0.25; BscanCompressionRatio >= 0.05; BscanCompressionRatio -= 0.05) {
        for (double CscanCompressionRatio = 1.0; CscanCompressionRatio >= 0.05; CscanCompressionRatio -= 0.05) {
            int numAScansPerBScan = static_cast<int>(NumAScansPerBScanReference * BscanCompressionRatio);
            int numBScansPerVolume = static_cast<int>(NumBScansPerVolumeReference * CscanCompressionRatio);

            RawDataHandle RawVolume = createRawData();
            DataHandle Volume = createData();
            DataHandle Surface = createData();
            ScanPatternHandle Pattern = createVolumePattern(Probe, LengthOfBScan, numAScansPerBScan, WidthOfVolume, numBScansPerVolume, ScanPattern_ApoOneForAll, ScanPattern_AcqOrderAll);


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

            ScanResult result;
            result.surface = Surface;
            result.actualTime = actualTimeDuration.count();
            result.expectedTime = expectedTime;
            result.numOfLostBScan = numOfLostBScan;
            result.BscanCompressionRatio = BscanCompressionRatio;
            result.CscanCompressionRatio = CscanCompressionRatio;

            processScanData(result, folderLocation);  // Process the data from this iteration

            // Clean up
            clearScanPattern(Pattern);
            clearData(Volume);
            clearRawData(RawVolume);

        }
    }
    clearProcessing(Proc);
    closeProbe(Probe);
    closeDevice(Dev);
}

void processScanData(const ScanResult& result, const std::string& folderLocation) {
    std::string fileName = "CompressionRatio_Bscan" + to_string(result.BscanCompressionRatio) +
        "_AScan" + to_string(result.CscanCompressionRatio);

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
        metaFile << "B-Scan Compression Ratio: " << result.BscanCompressionRatio << "\n";
        metaFile << "C-Scan Compression Ratio: " << result.CscanCompressionRatio << "\n";
        metaFile << "Actual Time: " << result.actualTime << "\n";
        metaFile << "Expected Time: " << result.expectedTime << "\n";
        metaFile << "Number of Lost B-Scans: " << result.numOfLostBScan << "\n";
        metaFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing metadata: " << metaFileName << "\n";
    }
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
    int NumAScansPerBScanReference = 256;
    int NumBScansPerVolumeReference = 100;


    getSurfaceFrom3DScan(folderLocation, NumAScansPerBScanReference, NumBScansPerVolumeReference, LengthOfBScan, WidthOfVolume);

    return 0;
}

