#pragma once

#include <chrono>
#include <string>
#include <cmath>
#include <cassert>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>

using namespace std;


class MeasurementData
{
    public:
        MeasurementData(std::string message_,
                        chrono::high_resolution_clock::time_point sentTime_);

        MeasurementData(std::string message_,
                        chrono::high_resolution_clock::time_point sentTime_,
                        chrono::high_resolution_clock::time_point receivedTime_);

        void setReceived(chrono::high_resolution_clock::time_point receivedTime_);

        std::string getMessage() { return message; }
        chrono::high_resolution_clock::time_point getSentTime() { return sentTime; }
        chrono::high_resolution_clock::time_point getReceivedTime() { return receivedTime; }
        bool isReceived() { return received; }
        double getDelayMs();

    private:
        std::string message;
        chrono::high_resolution_clock::time_point sentTime;
        chrono::high_resolution_clock::time_point receivedTime;
        bool received;
};


class Measurement
{
    public:
        Measurement() {}

        void addData(MeasurementData dataPoint);
        void setReceivedStatus(size_t id, chrono::high_resolution_clock::time_point receivedTime);
        MeasurementData getData(size_t id) { return data[id]; }
        size_t getDataNum() { return data.size(); }
        size_t getReceivedNum();
        double getLostRate();
        double getMaxDelay();
        double getMinDelay();
        double getAvgDelay();
        double getDelayStdDev();

        void writeRawDataToFile(std::string filename);
        void writeStatisticsToFile(std::string filename);
        void printStatistics();

    private:
        std::vector<MeasurementData> data;
};