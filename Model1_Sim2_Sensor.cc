/*
 * my_simulation_network.cc
 *
 *  Created on: July 1, 2024
 *      Author: Pramita
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <omnetpp.h>
#include "SensorNodeMsg_m.h" // Include the generated message header
#include "FeedbackThresholdMsg_m.h"

using namespace omnetpp;


class SensorNode : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void checkThreshold();
    virtual void handleConfigEnergyMessage(cMessage *msg);
    virtual void adjustThreshold(double feedback);
    virtual void forwardMessage(SensorNodeMsg *msg);

private:
    int nodeId;
    double eps; // Threshold
    int aboveThresholdCount;

    std::vector<double> sensorReadings;
    int currentReadingIndex;
    double lastSample;

    double Δ_N;
    double Δ_D;
    double Δ_M;  // Timer period for mandatory threshold update

    double previousMean;
    int batchSize = 60;
    std::vector<double> currentBatch;
    double tol_eps; // Tolerance for threshold adjustment

    int batchCounterSinceLastComm; // Counter to track time since last communication

    bool configReceived; // Flag to track receipt of ConfigEnergy message

    int hubNodeId; // ID of the Hub node to which this sensor node communicates

    double minReading; // Minimum value of the sensor readings
    double maxReading; // Maximum value of the sensor readings

public:
    SensorNode() : currentReadingIndex(0), lastSample(0), previousMean(0), batchSize(10), tol_eps(0.1), batchCounterSinceLastComm(0), configReceived(false), minReading(0), maxReading(0) {}
};

Define_Module(SensorNode);

void SensorNode::initialize() {
    Δ_N = par("Δ_N").doubleValue();
    Δ_D = par("Δ_D").doubleValue();
    eps = par("eps").doubleValue(); // Initialize threshold as eps
    Δ_M = par("Δ_M").doubleValue(); // Initialize timer period for mandatory threshold update
    batchSize = par("batchSize").intValue(); // Initialize batch size

    aboveThresholdCount = 0;
    currentReadingIndex = 0;
    lastSample = 0;

    int columnIndex = par("columnIndex").intValue();

    // Read the CSV file
    const char *fileName = par("fileName").stringValue();
    FILE *file = fopen(fileName, "r");
    if (!file) {
        EV << "Error opening file " << fileName << "\n";
        return;
    }

    // Skip the header row
    char buffer[1024];
    fgets(buffer, sizeof(buffer), file);

    // Read each line of the CSV file and store the values in sensorReadings
    double value;
    while (fgets(buffer, sizeof(buffer), file)) {
        char *token = strtok(buffer, ",");
        for (int i = 0; i < columnIndex; ++i) {
            token = strtok(nullptr, ",");
        }
        if (token) {
            sscanf(token, "%lf", &value);
            sensorReadings.push_back(value);
        }
    }

    fclose(file);
    EV << "Finished reading file. Total readings: " << sensorReadings.size() << "\n";
    EV << "Batchsize: " << batchSize << "\n";
    EV << "Total number of mean values: " << static_cast<double>(sensorReadings.size()) / batchSize << "\n";



    // Calculate min and max values for normalization
    if (!sensorReadings.empty()) {
        minReading = *std::min_element(sensorReadings.begin(), sensorReadings.end());
        maxReading = *std::max_element(sensorReadings.begin(), sensorReadings.end());
    }

    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    // Determine hubNodeId based on nodeId
    if (nodeId == 10 || nodeId == 11) {
        hubNodeId = 5; // Hub node ID for Node 10 and 11
    } else if (nodeId == 8 || nodeId == 9) {
        hubNodeId = 4; // Hub node ID for Node 8 and 9
    } else if (nodeId == 6 || nodeId == 7) {
        hubNodeId = 3; // Hub node ID for Node 6 and 7
    } else {
        hubNodeId = -1; // Invalid hubNodeId if nodeId does not match expected values
    }

    EV << "Initialization complete for sensor node: " << nodeId << " communicating with Hub_Node" << hubNodeId << "\n";

    // Start the sensing process immediately
    checkThreshold();
}

void SensorNode::handleMessage(cMessage *msg) {
    if (strcmp(msg->getName(), "ThresholdFeedback") == 0) {
         FeedbackThresholdMsg *feedbackMsg = check_and_cast<FeedbackThresholdMsg *>(msg);
         int receivedNodeId = feedbackMsg->getSourceNodeId();
         if (receivedNodeId == nodeId) {
             double newThreshold = feedbackMsg->getThreshold();
             adjustThreshold(newThreshold);
             EV << "Node " << nodeId << " received ThresholdFeedback from Hub_Node" << hubNodeId << " and updated threshold to: " << newThreshold << "\n";
         }
         delete feedbackMsg; // Free the memory after processing the message
    } else if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
        handleConfigEnergyMessage(msg);
        if (!configReceived) {
            configReceived = true;
            EV << "Config and energy message received, initializing timers.\n";
            //checkThreshold();

        }
    } else if (strcmp(msg->getName(), "StartNode") == 0) {
        EV << "Received start signal, starting sensing process.\n";
        // Start the sensing process
        checkThreshold();
    }

    // Include additional logging for message reception
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";
}

void SensorNode::checkThreshold() {
    if (currentReadingIndex < sensorReadings.size()) {
        double reading = sensorReadings[currentReadingIndex];

        // Normalize the reading
        double normalizedReading = (reading - minReading) / (maxReading - minReading);

        currentBatch.push_back(normalizedReading);
        currentReadingIndex++;

        if (currentBatch.size() == batchSize) {
            double currentMean = std::accumulate(currentBatch.begin(), currentBatch.end(), 0.0) / currentBatch.size();
            double meanDiff = currentMean - previousMean;

            if (meanDiff >= eps) {
                SensorNodeMsg *regCommMsg = new SensorNodeMsg("RegComm");
                regCommMsg->setMean(currentMean);
                regCommMsg->setSourceNodeId(nodeId); // Indicate the source nodeId
                regCommMsg->setDestinationNodeId(hubNodeId); // Indicate the destination nodeId

                EV << "Node " << nodeId << " prepared regular communication with mean: " << currentMean << "\n";

                forwardMessage(regCommMsg); // Use the new method to send the packet
                previousMean = currentMean;
                batchCounterSinceLastComm = 0; // Reset the counter when regular communication happens
            }

            currentBatch.clear();
        }
    }
    batchCounterSinceLastComm++;

    if (batchCounterSinceLastComm >= Δ_M) {
        if (!currentBatch.empty()) {
            double currentMean = std::accumulate(currentBatch.begin(), currentBatch.end(), 0.0) / currentBatch.size();
            SensorNodeMsg *mandCommMsg = new SensorNodeMsg("MandComm");
            mandCommMsg->setMean(currentMean);
            mandCommMsg->setSourceNodeId(nodeId); // Indicate the source nodeId
            EV << "Node " << nodeId << " prepared mandatory communication with mean: " << currentMean << "\n";

            forwardMessage(mandCommMsg); // Use the new method to send the packet
            batchCounterSinceLastComm = 0; // Reset the counter after mandatory communication
        }
    }
}

void SensorNode::forwardMessage(SensorNodeMsg *msg) {
    // Determine the Hub module name based on hubNodeId
    std::string hubModuleName;
    if (hubNodeId == 3) {
        hubModuleName = "Hub_1";
    } else if (hubNodeId == 4) {
        hubModuleName = "Hub_2";
    } else if (hubNodeId == 5) {
        hubModuleName = "Hub_3";
    } else {
        EV << "Error: Invalid hubNodeId " << hubNodeId << ". No corresponding Hub module.\n";
        delete msg; // Clean up the message if the hubNodeId is invalid
        return;
    }

    cModule *hubModule = getParentModule()->findModuleByPath(hubModuleName.c_str());

    if (hubModule) {
        send(msg, "out"); // No need to specify index for non-vector gate
        EV << "Message being forwarded to " << hubModuleName << "\n";
        bubble("Message Sent!");
    } else {
        EV << "Error: Hub module " << hubModuleName << " not found.\n";
        delete msg; // Clean up the message if the destination is not found
    }
}


void SensorNode::adjustThreshold(double feedback) {
    eps = feedback; // Update the threshold value
    EV << "Node " << nodeId << " adjusted threshold to: " << eps << "\n";
}

void SensorNode::handleConfigEnergyMessage(cMessage *msg) {
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // You can implement any additional logic for handling config and energy messages here
}


// Define Node11
class node11 : public SensorNode {
protected:
    virtual void initialize() override {
        SensorNode::initialize();
    }
};

Define_Module(node11);

// Define Node12
class node12 : public SensorNode {
protected:
    virtual void initialize() override {
        SensorNode::initialize();
    }
};

Define_Module(node12);



// Define Node21
class node21 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();
    }
};

Define_Module(node21);

// Define Node22
class node22 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();

    }
};

Define_Module(node22);

// Define Node31
class node31 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();
    }
};

Define_Module(node31);

// Define Node32
class node32 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();
    }
};

Define_Module(node32);

