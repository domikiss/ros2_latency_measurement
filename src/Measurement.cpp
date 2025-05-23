#include "Measurement.h"


MeasurementData::MeasurementData(std::string message_,
                                 chrono::high_resolution_clock::time_point sentTime_)
                                : message(message_)
                                , sentTime(sentTime_)
                                , received(false)
{
}

MeasurementData::MeasurementData(std::string message_,
                                 chrono::high_resolution_clock::time_point sentTime_,
                                 chrono::high_resolution_clock::time_point receivedTime_)
                                : message(message_)
                                , sentTime(sentTime_)
                                , receivedTime(receivedTime_)
                                , received(true)
{
}

void MeasurementData::setReceived(chrono::high_resolution_clock::time_point receivedTime_)
{
    receivedTime = receivedTime_;
    received = true;
}

double MeasurementData::getDelayMs()
{
    if (!isReceived())
    {
        return std::numeric_limits<double>::infinity();
    }

    std::chrono::duration<double, std::milli> delayMs = receivedTime - sentTime;

    return delayMs.count();
}


/***********************************************************************************/


void Measurement::addData(MeasurementData dataPoint)
{
    data.push_back(dataPoint);
}

size_t Measurement::getReceivedNum()
{
    size_t validNum = 0;

    for (auto d : data)
    {
        if (d.isReceived())
        {
            validNum++;
        }
    }
    return validNum;
}

void Measurement::setReceivedStatus(size_t id, chrono::high_resolution_clock::time_point receivedTime)
{
    assert(id <= data.size());
    data[id].setReceived(receivedTime);
}

double Measurement::getLostRate()
{
    size_t lostNum = 0;

    for (auto d : data)
    {
        if (!d.isReceived()) lostNum++;
    }

    return ((double)lostNum)/((double)data.size());
}

double Measurement::getMaxDelay()
{
    double maxDelay = 0;

    for (auto d : data)
    {
        if (d.isReceived())
        {
            if (d.getDelayMs() > maxDelay) maxDelay = d.getDelayMs();
        }
    }

    return maxDelay;
}

double Measurement::getMinDelay()
{
    double minDelay = std::numeric_limits<double>::infinity();;

    for (auto d : data)
    {
        if (d.isReceived())
        {
            if (d.getDelayMs() < minDelay) minDelay = d.getDelayMs();
        }
    }

    return minDelay;
}

double Measurement::getAvgDelay()
{
    double delaySum = 0;
    size_t validNum = 0;

    for (auto d : data)
    {
        if (d.isReceived())
        {
            delaySum += d.getDelayMs();
            validNum++;
        }
    }

    return delaySum/validNum;
}

double Measurement::getDelayStdDev()
{
    double mean = this->getAvgDelay();
    double meanDevSum = 0;
    size_t validNum = 0;

    for (auto d : data)
    {
        if (d.isReceived())
        {
            double meanDev = d.getDelayMs() - mean;
            meanDevSum += meanDev * meanDev;
            validNum++;
        }
    }

    return sqrt(meanDevSum / validNum);
}

void Measurement::writeRawDataToFile(std::string filename)
{
    auto logFile = make_unique<std::ofstream>(filename);
    chrono::high_resolution_clock::time_point timeReference;
    bool referenceSet = false;

    *logFile << "message,timestamp (ms),delay (ms)" << std::endl;

    for (auto d : data)
    {
        if (d.isReceived())
        {
            if (!referenceSet)
            {
                timeReference = d.getReceivedTime();
                referenceSet = true;
            }
            std::chrono::duration<double, std::milli> timestamp = d.getReceivedTime() - timeReference;
            *logFile << d.getMessage() << "," << timestamp.count() << "," << d.getDelayMs() << std::endl;
        }
    }
}

void Measurement::writeStatisticsToFile(std::string filename)
{
    auto logFile = make_unique<std::ofstream>(filename);

    *logFile << "Messages sent:     " << getDataNum() << std::endl;
    *logFile << "Messages received: " << getReceivedNum() << std::endl;
    *logFile << "Message loss rate: " << getLostRate()*100 << " %" << std::endl;
    *logFile << "Roundtrip delay:" << std::endl;
    *logFile << "      - average:   " << getAvgDelay() << " ms" << std::endl;
    *logFile << "      - std. dev.: " << getDelayStdDev() << " ms" << std::endl;
    *logFile << "      - min:       " << getMinDelay() << " ms" << std::endl;
    *logFile << "      - max:       " << getMaxDelay() << " ms" << std::endl;
}

void Measurement::printStatistics()
{
    std::cout << "Messages sent:     " << getDataNum() << std::endl;
    std::cout << "Messages received: " << getReceivedNum() << std::endl;
    std::cout << "Message loss rate: " << getLostRate()*100 << " %" << std::endl;
    std::cout << "Roundtrip delay:" << std::endl;
    std::cout << "      - average:   " << getAvgDelay() << " ms" << std::endl;
    std::cout << "      - std. dev.: " << getDelayStdDev() << " ms" << std::endl;
    std::cout << "      - min:       " << getMinDelay() << " ms" << std::endl;
    std::cout << "      - max:       " << getMaxDelay() << " ms" << std::endl;
}
