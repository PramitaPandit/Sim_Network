/*
 * my_simulation_network.cc
 *
 *  Created on: Feb 1, 2024
 *      Author: pramita
 */

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include "KalmanFilter.h"
#include "SensorNodeMsg_m.h" // Include the generated message
#include "DataAggregationMessage_m.h" // Include the generated DataAggregationMessage

using namespace omnetpp;

// Assuming the KalmanFilter class has a constructor that takes matrices
const std::vector<std::vector<double>> F = {{1.0, 0.0}, {0.0, 1.0}}; // Example state transition matrix
const std::vector<std::vector<double>> H = {{1.0, 0.0}}; // Example observation matrix
const std::vector<std::vector<double>> Q = {{0.1, 0.0}, {0.0, 0.1}}; // Process noise covariance
const std::vector<std::vector<double>> R = {{0.1}}; // Measurement noise covariance
const std::vector<std::vector<double>> P0 = {{1.0, 0.0}, {0.0, 1.0}}; // Initial state covariance
const std::vector<double> x0 = {0.0, 0.0}; // Initial state

class OBN_node : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    //virtual void handleAggregatedData(cMessage *msg);
    void handleAggregatedData(DataAggregationMessage *dataAggregationMsg);
    virtual void handleSensorData(int sourceNodeId, double receivedValue, KalmanFilter &kf);
    virtual void sendConfigAndEnergy();
    void processData(double HR_Node11, double HR_Node12, double HR_Node21, double HR_Node22, double HR_Node31, double HR_Node32);

    int nodeId;
//    double slotDuration = 0.01;
  //  bool channelBusy = false;

    int numDataAggregationMessagesReceived = 0;
    int expectedNumDataAggregationMessages = 10579; // Adjust based on your network topology
    bool simulationEnded = false;

    KalmanFilter kf_hub1_node11;
    KalmanFilter kf_hub1_node12;
    KalmanFilter kf_hub2_node21;
    KalmanFilter kf_hub2_node22;
    KalmanFilter kf_hub3_node31;
    KalmanFilter kf_hub3_node32;

    double HR_Node11;
    double HR_Node12;
    double HR_Node21;
    double HR_Node22;
    double HR_Node31;
    double HR_Node32;

    cMessage *periodicTimer = nullptr;
    double timerInterval = 5.0 * 3600.0; // 5 hours in seconds
    simtime_t lastDataAggregationTime;

    double Δ_M = 5.0; // Interval for receiving combined data
    double Δ_E = 0.0; // Interval for sending config and energy, once in 24 hours

public:
    OBN_node()
        : kf_hub1_node11(F, H, Q, R, P0, x0),
          kf_hub1_node12(F, H, Q, R, P0, x0),
          kf_hub2_node21(F, H, Q, R, P0, x0),
          kf_hub2_node22(F, H, Q, R, P0, x0),
          kf_hub3_node31(F, H, Q, R, P0, x0),
          kf_hub3_node32(F, H, Q, R, P0, x0),
          HR_Node11(0),
          HR_Node12(0),
          HR_Node21(0),
          HR_Node22(0),
          HR_Node31(0),
          HR_Node32(0) {
        lastDataAggregationTime = SIMTIME_ZERO;
    }
};

Define_Module(OBN_node);

void OBN_node::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    // Schedule the first data aggregation event
    //scheduleAt(simTime() + Δ_M, new DataAggregationMessage("AggregatedData"));
    //scheduleAt(simTime() + Δ_E, new cMessage("TimeInterruptSendConfigEnergy"));

    // Send initial config and energy message
    sendConfigAndEnergy();

    periodicTimer = new cMessage("PeriodicTimer");
    scheduleAt(simTime() + timerInterval, periodicTimer);
}

void OBN_node::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";
  //  if (strcmp(msg->getName(), "TimeInterruptSendConfigEnergy") == 0) {
    //    sendConfigAndEnergy();
      //  scheduleAt(simTime() + Δ_E, new cMessage("TimeInterruptSendConfigEnergy"));
    if (msg == periodicTimer) {
         simtime_t now = simTime();
         if (now - lastDataAggregationTime >= timerInterval) {
             EV << "No DataAggregationMessage received in the last 5 hours.";
             // Handle the case where no message was received
         } else {
             EV << "DataAggregationMessage received within the last 5 hours.";
             // Handle the case where a message was received
         }

         // Reschedule the timer
         scheduleAt(simTime() + timerInterval, periodicTimer);

    } else if (strcmp(msg->getName(), "DataAggregation") == 0) {
  //  if (strcmp(msg->getName(), "AggregatedData") == 0) {

        EV << "Receiving Aggregated data from hub nodes at " << simTime() << "\n" ;

        DataAggregationMessage *dataAggregationMsg = dynamic_cast<DataAggregationMessage *>(msg);
        EV << "DataAggregationMessage.";
        if (dataAggregationMsg) {
            handleAggregatedData(dataAggregationMsg);
            numDataAggregationMessagesReceived++;
            if (numDataAggregationMessagesReceived == expectedNumDataAggregationMessages) {
                simulationEnded = true;
                // Perform any necessary cleanup or final calculations
            }
        } else {
            EV << "Received message is not of type DataAggregationMessage.\n";
        }

        delete dataAggregationMsg; // Clean up the message after processing

        // Schedule the next data aggregation event
        //scheduleAt(simTime() + Δ_M, new DataAggregationMessage("AggregatedData"));
    } else {
        EV << "Unhandled message type: " << msg->getName() << " from " << msg->getSenderModule()->getName() << " at time " << simTime() << "\n";
    }

 //   EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";
}

void OBN_node::handleAggregatedData(DataAggregationMessage *dataAggregationMsg) {
    double receivedValue_node11 = dataAggregationMsg->hasPar("aggregatedValueNode11") ? dataAggregationMsg->par("aggregatedValueNode11").doubleValue() : 0.0;
    EV << " Received Value from Node11: " << receivedValue_node11 << "\n";
    handleSensorData(6, receivedValue_node11, kf_hub1_node11);
    bubble("Message Received from Hub_Node1 (Node11)");

    double receivedValue_node12 = dataAggregationMsg->hasPar("aggregatedValueNode12") ? dataAggregationMsg->par("aggregatedValueNode12").doubleValue() : 0.0;
    handleSensorData(7, receivedValue_node12, kf_hub1_node12);
    EV << " Received Value from Node12: " << receivedValue_node12 << "\n";
    bubble("Message Received from Hub_Node1 (Node12)");

    double receivedValue_node21 = dataAggregationMsg->hasPar("aggregatedValueNode21") ? dataAggregationMsg->par("aggregatedValueNode21").doubleValue() : 0.0;
    handleSensorData(8, receivedValue_node21, kf_hub2_node21);
    bubble("Message Received from Hub_Node2 (Node21)");

    double receivedValue_node22 = dataAggregationMsg->hasPar("aggregatedValueNode22") ? dataAggregationMsg->par("aggregatedValueNode22").doubleValue() : 0.0;
    handleSensorData(9, receivedValue_node22, kf_hub2_node22);
    bubble("Message Received from Hub_Node2 (Node22)");

    double receivedValue_node31 = dataAggregationMsg->hasPar("aggregatedValueNode31") ? dataAggregationMsg->par("aggregatedValueNode31").doubleValue() : 0.0;
    handleSensorData(10, receivedValue_node31, kf_hub3_node31);
    bubble("Message Received from Hub_Node3 (Node31)");

    double receivedValue_node32 = dataAggregationMsg->hasPar("aggregatedValueNode32") ? dataAggregationMsg->par("aggregatedValueNode32").doubleValue() : 0.0;
    handleSensorData(11, receivedValue_node32, kf_hub3_node32);
    bubble("Message Received from Hub_Node3 (Node32)");

    processData(HR_Node11, HR_Node12, HR_Node21, HR_Node22, HR_Node31, HR_Node32);
}

void OBN_node::handleSensorData(int sourceNodeId, double receivedValue, KalmanFilter &kf) {
    EV << "Handling data reception for Node " << sourceNodeId << "\n";

    double predictedValue, filteredValue, PredError;

    kf.predict();
    predictedValue = kf.getState(); // Access the predicted state

    filteredValue = kf.update(receivedValue);
    EV << "Filtered value after update: " << filteredValue << "\n";

    EV << "Predicted state: " << predictedValue << "\n";
    EV << "Received value from Node " << sourceNodeId << ": " << receivedValue << ", Predicted value: " << predictedValue << ", Filtered value: " << filteredValue << "\n";

    PredError = receivedValue - predictedValue; // Calculate the error
    EV << "Prediction Error: " << PredError << "\n";
  //  adjustThresholdValue(error, predictedValue, sourceNodeId); // Pass error and predicted value to adjustThresholdValue

    if (sourceNodeId == 6) {
        HR_Node11 = filteredValue;
    } else if (sourceNodeId == 7) {
        HR_Node12 = filteredValue;
    } else if (sourceNodeId == 8) {
        HR_Node21 = filteredValue;
    } else if (sourceNodeId == 9) {
        HR_Node22 = filteredValue;
    } else if (sourceNodeId == 10) {
        HR_Node31 = filteredValue;
    } else if (sourceNodeId == 11) {
        HR_Node32 = filteredValue;
    }
}

void OBN_node::processData(double HR_Node11, double HR_Node12, double HR_Node21, double HR_Node22, double HR_Node31, double HR_Node32) {
    double averageHR = (HR_Node11 + HR_Node12 + HR_Node21 + HR_Node22 + HR_Node31 + HR_Node32) / 6.0;
    const double normalHRRangeMin = 60.0;
    const double normalHRRangeMax = 100.0;

    if (averageHR >= normalHRRangeMin && averageHR <= normalHRRangeMax) {
        EV << "Average HR: " << averageHR << " is within the normal range.\n";
        bubble("Average HR is within the normal range.");
    } else {
        EV << "Average HR: " << averageHR << " is outside the normal range.\n";
        bubble("Average HR is outside the normal range.");
    }
}

void OBN_node::sendConfigAndEnergy() {
    EV << "Sending configuration parameters and energy to hub nodes and sensor nodes at " << simTime() << "\n";

    cMessage *configEnergyMsg = new cMessage("ConfigEnergy");
    configEnergyMsg->addPar("configParam1") = 42;
    configEnergyMsg->addPar("energyLevel") = 100.0;

    for (int i = 0; i < gateSize("out"); ++i) {
        send(configEnergyMsg->dup(), "out", i);
    }

    delete configEnergyMsg;
}


