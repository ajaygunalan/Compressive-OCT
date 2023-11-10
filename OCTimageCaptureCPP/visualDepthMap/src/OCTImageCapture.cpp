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


struct ScanResult {
    DataHandle surface;
    double actualTime;
    double expectedTime;
};


void logDataProperties(DataHandle DH) {
#ifdef DEBUG
	int dataDimensions = getDataPropertyInt(DH, Data_Dimensions);
	int dataSize1 = getDataPropertyInt(DH, Data_Size1);
	int dataSize2 = getDataPropertyInt(DH, Data_Size2);
	int dataSize3 = getDataPropertyInt(DH, Data_Size3);
	int dataNumberOfElements = getDataPropertyInt(DH, Data_NumberOfElements);
	int dataSizeInBytes = getDataPropertyInt(DH, Data_SizeInBytes);
	int dataBytesPerElement = getDataPropertyInt(DH, Data_BytesPerElement);

	cout << "Data Properties: Dimensions=" << dataDimensions
		<< ", Size1=" << dataSize1
		<< ", Size2=" << dataSize2
		<< ", Size3=" << dataSize3
		<< ", NumberOfElements=" << dataNumberOfElements
		<< ", SizeInBytes=" << dataSizeInBytes
		<< ", BytesPerElement=" << dataBytesPerElement << endl;
#endif
}

std::pair<double, double> VolumeScanPattern(int AScansPerBScan, double LengthOfBScan, int BScansPerVolume, double WidthOfVolume) {
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
        return std::make_pair(-1.0, -1.0);  // Indicate error with negative times
    }

    // Set up the scan pattern
    ScanPatternHandle Pattern = createVolumePattern(Probe, LengthOfBScan, AScansPerBScan, WidthOfVolume, BScansPerVolume, ScanPattern_ApoEachBScan, ScanPattern_AcqOrderAll);

    auto start = std::chrono::high_resolution_clock::now();

    startMeasurement(Dev, Pattern, Acquisition_AsyncFinite);
    getRawData(Dev, RawVolume);
    setProcessedDataOutput(Proc, Volume);
    executeProcessing(Proc, RawVolume);
    stopMeasurement(Dev);

    auto stop = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> actualTime = stop - start;
    double expectedTime = expectedAcquisitionTime_s(Pattern, Dev); // Assuming this function returns expected time in seconds



    determineSurface(Volume, Surface);
    exportData(Surface, DataExport_CSV,  "C:\\Ajay_OCT\\OCTAssistedSurgicalLaserbot\\data\\getDepthFromSparse3Doct\\surface.csv" );

    // Clean up
    clearScanPattern(Pattern);
    clearData(Volume);
    clearRawData(RawVolume);
    clearProcessing(Proc);
    closeProbe(Probe);
    closeDevice(Dev);

    _getch();

    return std::make_pair(actualTime.count(), expectedTime);
}



int main() {
    // Parameters for the volume pattern
    int AScansPerBScan = 128;
    double LengthOfBScan = 2.0;
    int BScansPerVolume = 128;
    double WidthOfVolume = 2.0;

    //  Do volume Scan
    auto times = VolumeScanPattern(AScansPerBScan, LengthOfBScan, BScansPerVolume, WidthOfVolume);

    double actualTime = times.first;
    double expectedTime = times.second;
    double timeDifference = actualTime - expectedTime;
    std::cout << "Actual time taken: " << actualTime << " seconds" << std::endl;
    std::cout << "Expected time: " << expectedTime << " seconds" << std::endl;
    std::cout << "Time difference: " << timeDifference << " seconds" << std::endl;

    return 0;
}
