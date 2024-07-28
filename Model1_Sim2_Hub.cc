/*
 * my_simulation_network2.cc
 *
 *  Created on: July 8, 2024
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
#include "FeedbackThresholdMsg_m.h"
#include "DataAggregationMessage_m.h" // Include the generated DataAggregationMessage


using namespace omnetpp;

// Assuming the KalmanFilter class has a constructor t  hat takes matrices

const std::vector<std::vector<double>> F = {{1.0, 0.0}, {0.0, 1.0}}; // 2x2 state transition matrix
const std::vector<std::vector<double>> H = {{1.0, 0.0}}; // 1x2 observation matrix
const std::vector<std::vector<double>> Q = {{0.1, 0.0}, {0.0, 0.1}}; // 2x2 Process noise covariance
const std::vector<std::vector<double>> R = {{0.1}}; // 1x1 Measurement noise covariance
const std::vector<std::vector<double>> P0 = {{1.0, 0.0}, {0.0, 1.0}}; // 2x2 Initial state covariance
const std::vector<double> x0 = {0.0, 0.0}; // 2x1 // Initial State



class Hub_Node1 : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void handleSensorDataReception(cMessage *msg);
    virtual void handleDataAggregation(double observationSum_node11, double observationSum_node12);
    virtual void adjustThresholdValue(double error, double filteredValue, int nodeId);
    virtual void sendFeedbackThresholdValue(double threshold, int nodeId);
    virtual void handleConfigEnergyMessage(cMessage *msg);

    int nodeId;
    double Δ_D, Δ_M;
    double eps; // Threshold
    double tol_KF; // Tolerance for Kalman filter
    double tol_eps; // Tolerance for threshold adjustment
    double min_eps;
    double max_eps;
    double K;

    KalmanFilter kf_node11; // Kalman Filter for node11
    KalmanFilter kf_node12; // Kalman Filter for node12
    std::vector<double> data_values_node11;
    std::vector<double> data_values_node12;

    int observationCount;
    double observationSum_node11;
    double observationSum_node12;

    bool configReceived = false; // Flag to track receipt of ConfigEnergy message
    bool firstSensorMessageReceived = false; // Flag to track receipt of first sensor message

public:
    Hub_Node1()
        : kf_node11(F, H, Q, R, P0, x0),
          kf_node12(F, H, Q, R, P0, x0),
          observationCount(0),
          observationSum_node11(0.0),
          observationSum_node12(0.0) {}
};

Define_Module(Hub_Node1);

void Hub_Node1::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    Δ_D = par("Δ_D").doubleValue();
    Δ_M = par("Δ_M").doubleValue();
    eps = par("eps").doubleValue();
    tol_KF = par("tol_KF").doubleValue();
    tol_eps = par("tol_eps").doubleValue();
    min_eps = par("min_eps").doubleValue();
    max_eps = par("max_eps").doubleValue();
    K = par("K").doubleValue();


    for (int i = 0; i < gateSize("in"); i++) {
        gate("in", i)->setDeliverImmediately(true); // Ensure packets are delivered at the start of reception
    }
}

void Hub_Node1::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (!configReceived) {
        if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
            handleConfigEnergyMessage(msg);
            configReceived = true;
            EV << "Config and energy message received at Hub_Node1, initializing timers.\n";
            // Schedule the first data aggregation event
        } else {
            EV << "Waiting for ConfigEnergy message.\n";
            delete msg; // Discard any other messages until config is received
        }
        return;
    } else if (strcmp(msg->getName(), "RegComm") == 0 || strcmp(msg->getName(), "MandComm") == 0) {
        firstSensorMessageReceived = true;
        handleSensorDataReception(msg);
    } else {
        EV << "Unhandled message type: " << msg->getName() << "\n";
    }
}

void Hub_Node1::handleSensorDataReception(cMessage *msg) {
    EV << "Handling RegComm/MandComm reception.\n";

    // Type cast the message to SensorNodeMsg for direct access
    SensorNodeMsg *sensorMsg = dynamic_cast<SensorNodeMsg*>(msg);
    if (!sensorMsg) {
        EV << "Received message is not of type SensorNodeMsg.\n";
        delete msg;
        return;
    }

    double receivedValue = sensorMsg->getMean();
    int sourceNodeId = sensorMsg->getSourceNodeId();

    double filteredValue;
    double predictedValue;

    if (sourceNodeId == 6) {
        // Handle node11
        EV << "Received value from Node_11: " << receivedValue << "\n";

        kf_node11.predict();
        predictedValue = kf_node11.getState(); // Access the predicted state

        filteredValue = kf_node11.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node11.push_back(filteredValue);
        EV << "Predicted state: " << filteredValue << "\n";
        EV << "Received value from Node_11: " << receivedValue << ", Predicted value: " << filteredValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node11!");

        double error = receivedValue - predictedValue; // Calculate the error
        adjustThresholdValue(error, predictedValue, 6); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node11 += receivedValue;
        } else {
            observationSum_node11 += filteredValue;
        }
        observationCount++;

    } else if (sourceNodeId == 7) {
        // Handle node12
        EV << "Received value from Node_12: " << receivedValue << "\n";

        kf_node12.predict();
        predictedValue = kf_node12.getState(); // Access the predicted state

        filteredValue = kf_node12.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node12.push_back(filteredValue);
        EV << "Predicted state: " << filteredValue << "\n";
        EV << "Received value from Node_12: " << receivedValue << ", Predicted value: " << filteredValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node12!");

        double error = receivedValue - filteredValue; // Calculate the error
        adjustThresholdValue(error, filteredValue, 7); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node12 += receivedValue;
        } else {
            observationSum_node12 += filteredValue;
        }
        observationCount++;

    } else {
        EV << "Unknown sourceNodeId for sensor data reception: " << sourceNodeId << "\n";
    }

        if (observationCount == 2) {
            handleDataAggregation(observationSum_node11, observationSum_node12);
            observationCount = 0;
            observationSum_node11 = 0.0;
            observationSum_node12 = 0.0;
        }

delete msg; // Clean up message
}

void Hub_Node1::adjustThresholdValue(double error, double filteredValue, int nodeId) {

    EV << "AdjustThresholdValue: Initial threshold: " << eps
       << ", Error: " << error << ", Predicted Value (State): " << filteredValue << "\n";

    if (fabs(error) > tol_KF) {
        double eps_new = eps * (1 + K * error);
        EV << "Value of K: " << K << "\n";
        EV << "Value of tol_KF: " << tol_KF << "\n";
        EV << "Value of error: " << error << "\n";
        eps_new = std::max(min_eps, std::min(eps_new, max_eps));
        EV << "Value of min_eps: " << min_eps << "\n";
        EV << "Value of max_eps: " << max_eps << "\n";
        EV << "Value of eps_new: " << eps_new << "\n";
        double diff = std::abs(eps_new - eps);
        EV << "Value of diff: " << diff << "\n";
        if (diff <= tol_eps) {
            eps = eps_new;
            EV << "Threshold adjusted to: " << eps << "\n";
            sendFeedbackThresholdValue(eps, nodeId); // Send the adjusted threshold value back to the sensor node
            EV << "AdjustThresholdValue: New threshold: " << eps << "\n";
            EV << "Feedback message sent.\n";
        } else {
            EV << "Threshold adjustment skipped. Difference (" << diff << ") exceeds tolerance (" << tol_eps << ").\n";
            EV << "AdjustThresholdValue: No change in threshold value. Error (" << error << ") is within tolerance (" << tol_KF << ").\n";
        }
    }
}

void Hub_Node1::sendFeedbackThresholdValue(double threshold, int nodeId) {
    EV << "Sending feedback threshold value: " << threshold << " to Node " << nodeId << "\n";
    FeedbackThresholdMsg *feedbackMsg = new FeedbackThresholdMsg("FeedbackThreshold");
    feedbackMsg->setThreshold(threshold);
    feedbackMsg->setSourceNodeId(getId());
    feedbackMsg->setDestinationNodeId(nodeId);

    if (nodeId == 6) {
        send(feedbackMsg, "out", 0); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << threshold << " to node: " << nodeId << "\n";
    } else if (nodeId == 7) {
        send(feedbackMsg, "out", 1); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << threshold << " to node: " << nodeId << "\n";
    } else {
        EV << "Unknown nodeId for feedback message: " << nodeId << "\n";
        delete feedbackMsg;
    }
}


void Hub_Node1::handleDataAggregation(double observationSum_node11, double observationSum_node12) {
    double aggregatedValue_node11 = observationSum_node11 ;
    double aggregatedValue_node12 = observationSum_node12 ;

    EV << "Aggregated value for Node 11: " << aggregatedValue_node11 << "\n";
    EV << "Aggregated value for Node 12: " << aggregatedValue_node12 << "\n";


    // Create a new DataAggregationMessage object
    DataAggregationMessage *dataAggregationMsg = new DataAggregationMessage("DataAggregation");
    dataAggregationMsg->setSourceNodeId(getId());
    // dataAggregationMsg->setDestinationNodeId(hubNodeId); // Indicate the destination nodeId
    dataAggregationMsg->setAggregatedValueNode11(aggregatedValue_node11);
    dataAggregationMsg->setAggregatedValueNode12(aggregatedValue_node12);

    send(dataAggregationMsg, "out", 2); // Send the aggregated data message to the OBN node
    EV << "DataAggregation message sent with aggregated values to the OBN node from Hub_Node1.\n";
    bubble("AggregationMessage Sent from Hub_Node1!");
}


void Hub_Node1::handleConfigEnergyMessage(cMessage *msg) {
    EV << "Handling config and energy message in Hub_Node1.\n";
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param 1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // Handle configuration and energy information as needed
    // For example, store values for later use, update internal state, etc.
}

class Hub_Node2 : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void handleSensorDataReception(cMessage *msg);
    virtual void handleDataAggregation();
    virtual void adjustThresholdValue(double value, int nodeId);
    virtual void sendFeedbackThresholdValue(double threshold, int nodeId);
    virtual void handleConfigEnergyMessage(cMessage *msg);

    int nodeId;
    double Δ_D, Δ_M;
    double eps; // Threshold
    double tol_KF; // Tolerance for Kalman filter
    double tol_eps; // Tolerance for threshold adjustment
    double min_eps;
    double max_eps;
    double K;

    KalmanFilter kf_node21; // Kalman Filter for node21
    KalmanFilter kf_node22; // Kalman Filter for node22
    std::vector<double> data_values_node21;
    std::vector<double> data_values_node22;

    cMessage *DataAggregationMessage = nullptr;

    bool configReceived = false; // Flag to track receipt of ConfigEnergy message
    bool firstSensorMessageReceived = false; // Flag to track receipt of first sensor message

public:
    Hub_Node2()
        : kf_node21(F, H, Q, R, P0, x0),
          kf_node22(F, H, Q, R, P0, x0) {}
};

Define_Module(Hub_Node2);

void Hub_Node2::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    Δ_D = par("Δ_D").doubleValue();
    Δ_M = par("Δ_M").doubleValue();
    eps = par("eps").doubleValue();
    tol_KF = par("tol_KF").doubleValue();
    tol_eps = par("tol_eps").doubleValue();
    min_eps = par("min_eps").doubleValue();
    max_eps = par("max_eps").doubleValue();
    K = par("K").doubleValue();

    DataAggregationMessage = new cMessage("DataAggregation");

    for (int i = 0; i < gateSize("in"); i++) {
        gate("in", i)->setDeliverImmediately(true); // Ensure packets are delivered at the start of reception
    }
}

void Hub_Node2::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (!configReceived) {
        if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
            handleConfigEnergyMessage(msg);
            configReceived = true;
            EV << "Config and energy message received at Hub_Node2, initializing timers.\n";
            // Schedule the first data aggregation event
            scheduleAt(simTime() + Δ_D, DataAggregationMessage);
        } else {
            EV << "Waiting for ConfigEnergy message.\n";
            delete msg; // Discard any other messages until config is received
        }
        return;
    } else if (strcmp(msg->getName(), "RegComm") == 0 || strcmp(msg->getName(), "MandComm") == 0) {
        firstSensorMessageReceived = true;
        handleSensorDataReception(msg);
    } else {
        EV << "Unhandled message type: " << msg->getName() << "\n";
    }
}

void Hub_Node2::handleSensorDataReception(cMessage *msg) {
    EV << "Handling RegComm reception.\n";

    // Type cast the message to SensorNodeMsg for direct access
    SensorNodeMsg *sensorMsg = dynamic_cast<SensorNodeMsg*>(msg);
    if (!sensorMsg) {
        EV << "Received message is not of type SensorNodeMsg.\n";
        delete msg;
        return;
    }

    double receivedValue = sensorMsg->getMean();
    int sourceNodeId = sensorMsg->getSourceNodeId();

    double filteredValue;

    if (sourceNodeId == 8) {
        // Handle node21 data
        filteredValue = kf_node21.updateEstimate(receivedValue); // Ensure updateEstimate() is implemented in KalmanFilter
        data_values_node21.push_back(filteredValue);
        EV << "Received value from Node_21: " << receivedValue << ", Predicted value: " << filteredValue << "\n";
        bubble("Message Received from Node21!");
        adjustThresholdValue(filteredValue, 8);
    } else if (sourceNodeId == 9) {
        // Handle node22 data
        filteredValue = kf_node22.updateEstimate(receivedValue); // Ensure updateEstimate() is implemented in KalmanFilter
        data_values_node22.push_back(filteredValue);
        EV << "Received value from Node_22: " << receivedValue << ", Predicted value: " << filteredValue << "\n";
        adjustThresholdValue(filteredValue, 9);
        bubble("Message Received from Node22!");
    } else {
        EV << "Unknown sourceNodeId for sensor data reception: " << sourceNodeId << "\n";
    }

    delete msg; // Clean up message
}

void Hub_Node2::adjustThresholdValue(double value, int nodeId) {
    KalmanFilter *kf;
    std::vector<double> *data_values;

    if (nodeId == 8) {
        kf = &kf_node21;
        data_values = &data_values_node21;
    } else if (nodeId == 9) {
        kf = &kf_node22;
        data_values = &data_values_node22;
    } else {
        EV << "Unknown nodeId for threshold adjustment: " << nodeId << "\n";
        return;
    }

    kf->predict();
    double state = kf->getState(); // Ensure getState() is implemented in KalmanFilter
    double err = state - value;

    EV << "AdjustThresholdValue: Initial threshold: " << eps
       << ", Received value: " << value << ", Predicted Value (State): " << state
       << ", Error: " << err << "\n";

    if (fabs(err) > tol_KF) {
        double eps_new = eps * (1 + K * err);
        eps_new = std::max(min_eps, std::min(eps_new, max_eps));
        double diff = std::abs(eps_new - eps);
        if (diff <= tol_eps) {
            eps = eps_new;
            EV << "Threshold adjusted to: " << eps << "\n";
            sendFeedbackThresholdValue(eps, nodeId); // Send the adjusted threshold value back to the sensor node
        } else {
            EV << "Threshold adjustment skipped. Difference (" << diff << ") exceeds tolerance (" << tol_eps << ").\n";
        }
        EV << "AdjustThresholdValue: New threshold: " << eps << "\n";
    } else {
        EV << "AdjustThresholdValue: No change in threshold value. Error (" << err << ") is within tolerance (" << tol_KF << ").\n";
    }
}

void Hub_Node2::sendFeedbackThresholdValue(double threshold, int nodeId) {
    EV << "Sending feedback threshold value: " << threshold << " to Node " << nodeId << "\n";
    FeedbackThresholdMsg *feedbackMsg = new FeedbackThresholdMsg("ThresholdFeedback");
    feedbackMsg->setThreshold(threshold);
    feedbackMsg->setSourceNodeId(getId()); // Assuming getId() gives the ID of the current node
    feedbackMsg->setDestinationNodeId(nodeId);
    if (nodeId == 8) {
        send(feedbackMsg, "out", 0); // Adjust the gate index as needed
    } else if (nodeId == 9) {
        send(feedbackMsg, "out", 1); // Adjust the gate index as needed
    } else {
        EV << "Unknown nodeId for feedback message: " << nodeId << "\n";
        delete feedbackMsg;
    }
}

void Hub_Node2::handleDataAggregation() {
    EV << "Handling data aggregation.\n";
    double sum_node21 = 0.0;
    int count_node21 = 0;
    double sum_node22 = 0.0;
    int count_node22 = 0;
    for (double value : data_values_node21) {
        sum_node21 += value;
        count_node21++;
    }
    for (double value : data_values_node22) {
        sum_node22 += value;
        count_node22++;
    }
    if (count_node21 == 0 && count_node22 == 0) {
        EV << "No data received from either node for aggregation.\n";
        return;
    }
    double meanValue_node21 = (count_node21 > 0) ? (sum_node21 / count_node21) : 0.0;
    double meanValue_node22 = (count_node22 > 0) ? (sum_node22 / count_node22) : 0.0;
    EV << "Aggregated data mean value for Node_21: " << meanValue_node21 << "\n";
    EV << "Aggregated data mean value for Node_22: " << meanValue_node22 << "\n";
    cMessage *aggregationMsg= new cMessage("AggregatedData");
    aggregationMsg->addPar("mean_node21") = meanValue_node21;
    aggregationMsg->addPar("mean_node22") = meanValue_node22;
    send(aggregationMsg, "out", 2); // Assuming 'outGate' is defined and connected
    bubble("AggregationMessage Sent from Hub_Node2!");
    }

    void Hub_Node2::handleConfigEnergyMessage(cMessage *msg) {
    EV << "Handling config and energy message in Hub_Node2.\n";
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param 1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // Handle configuration and energy information as needed
    // For example, store values for later use, update internal state, etc.
}


class Hub_Node3 : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void handleSensorDataReception(cMessage *msg);
    virtual void handleDataAggregation();
    virtual void adjustThresholdValue(double value, int nodeId);
    virtual void sendFeedbackThresholdValue(double threshold, int nodeId);
    virtual void handleConfigEnergyMessage(cMessage *msg);

    int nodeId;
    double Δ_D, Δ_M;
    double eps; // Threshold
    double tol_KF; // Tolerance for Kalman filter
    double tol_eps; // Tolerance for threshold adjustment
    double min_eps;
    double max_eps;
    double K;

    KalmanFilter kf_node31; // Kalman Filter for node31
    KalmanFilter kf_node32; // Kalman Filter for node32
    std::vector<double> data_values_node31;
    std::vector<double> data_values_node32;

    cMessage *DataAggregationMessage = nullptr;

    bool configReceived = false; // Flag to track receipt of ConfigEnergy message
    bool firstSensorMessageReceived = false; // Flag to track receipt of first sensor message

public:
    Hub_Node3()
        : kf_node31(F, H, Q, R, P0, x0),
          kf_node32(F, H, Q, R, P0, x0) {}
};

Define_Module(Hub_Node3);

void Hub_Node3::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    Δ_D = par("Δ_D").doubleValue();
    Δ_M = par("Δ_M").doubleValue();
    eps = par("eps").doubleValue();
    tol_KF = par("tol_KF").doubleValue();
    tol_eps = par("tol_eps").doubleValue();
    min_eps = par("min_eps").doubleValue();
    max_eps = par("max_eps").doubleValue();
    K = par("K").doubleValue();

    DataAggregationMessage = new cMessage("DataAggregation");

    for (int i = 0; i < gateSize("in"); i++) {
        gate("in", i)->setDeliverImmediately(true); // Ensure packets are delivered at the start of reception
    }
}

void Hub_Node3::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (!configReceived) {
        if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
            handleConfigEnergyMessage(msg);
            configReceived = true;
            EV << "Config and energy message received at Hub_Node3, initializing timers.\n";
            // Schedule the first data aggregation event
            scheduleAt(simTime() + Δ_D, DataAggregationMessage);
        } else {
            EV << "Waiting for ConfigEnergy message.\n";
            delete msg; // Discard any other messages until config is received
        }
        return;

    } else if (strcmp(msg->getName(), "RegComm") == 0 || strcmp(msg->getName(), "MandComm") == 0) {
        firstSensorMessageReceived = true;
        handleSensorDataReception(msg);
    } else {
        EV << "Unhandled message type: " << msg->getName() << "\n";
    }
}

void Hub_Node3::handleSensorDataReception(cMessage *msg) {
    EV << "Handling RegComm reception.\n";

    // Type cast the message to SensorNodeMsg for direct access
    SensorNodeMsg *sensorMsg = dynamic_cast<SensorNodeMsg*>(msg);
    if (!sensorMsg) {
        EV << "Received message is not of type SensorNodeMsg.\n";
        delete msg;
        return;
    }

    double receivedValue = sensorMsg->getMean();
    int sourceNodeId = sensorMsg->getSourceNodeId();

    double filteredValue;

    if (sourceNodeId == 10) {
        // Handle node31 data
        filteredValue = kf_node31.updateEstimate(receivedValue); // Ensure updateEstimate() is implemented in KalmanFilter
        data_values_node31.push_back(filteredValue);
        EV << "Received value from Node_31: " << receivedValue << ", Predicted value: " << filteredValue << "\n";
        bubble("Message Received from Node31!");
        adjustThresholdValue(filteredValue, 10);
    } else if (sourceNodeId == 11) {
        // Handle node32 data
        filteredValue = kf_node32.updateEstimate(receivedValue); // Ensure updateEstimate() is implemented in KalmanFilter
        data_values_node32.push_back(filteredValue);
        EV << "Received value from Node_32: " << receivedValue << ", Predicted value: " << filteredValue << "\n";
        adjustThresholdValue(filteredValue, 11);
        bubble("Message Received from Node32!");
    } else {
        EV << "Unknown sourceNodeId for sensor data reception: " << sourceNodeId << "\n";
    }

    delete msg; // Clean up message
}

void Hub_Node3::adjustThresholdValue(double value, int nodeId) {
    KalmanFilter *kf;
    std::vector<double> *data_values;

    if (nodeId == 10) {
        kf = &kf_node31;
        data_values = &data_values_node31;
    } else if (nodeId == 11) {
        kf = &kf_node32;
        data_values = &data_values_node32;
    } else {
        EV << "Unknown nodeId for threshold adjustment: " << nodeId << "\n";
        return;
    }

    kf->predict();
    double state = kf->getState(); // Ensure getState() is implemented in KalmanFilter
    double err = state - value;

    EV << "AdjustThresholdValue: Initial threshold: " << eps
       << ", Received value: " << value << ", Predicted Value (State): " << state
       << ", Error: " << err << "\n";

    if (fabs(err) > tol_KF) {
        double eps_new = eps * (1 + K * err);
        eps_new = std::max(min_eps, std::min(eps_new, max_eps));
        double diff = std::abs(eps_new - eps);
        if (diff <= tol_eps) {
            eps = eps_new;
            EV << "Threshold adjusted to: " << eps << "\n";
            sendFeedbackThresholdValue(eps, nodeId); // Send the adjusted threshold value back to the sensor node
        } else {
            EV << "Threshold adjustment skipped. Difference (" << diff << ") exceeds tolerance (" << tol_eps << ").\n";
        }
        EV << "AdjustThresholdValue: New threshold: " << eps << "\n";
    } else {
        EV << "AdjustThresholdValue: No change in threshold value. Error (" << err << ") is within tolerance (" << tol_KF << ").\n";
    }
}

void Hub_Node3::sendFeedbackThresholdValue(double threshold, int nodeId) {
    EV << "Sending feedback threshold value: " << threshold << " to Node " << nodeId << "\n";
    FeedbackThresholdMsg *feedbackMsg = new FeedbackThresholdMsg("ThresholdFeedback");
    feedbackMsg->setThreshold(threshold);
    feedbackMsg->setSourceNodeId(getId()); // Assuming getId() gives the ID of the current node
    feedbackMsg->setDestinationNodeId(nodeId);
    if (nodeId == 10) {
        send(feedbackMsg, "out", 0); // Adjust the gate index as needed
    } else if (nodeId == 11) {
        send(feedbackMsg, "out", 1); // Adjust the gate index as needed
    } else {
        EV << "Unknown nodeId for feedback message: " << nodeId << "\n";
        delete feedbackMsg;
    }
}

void Hub_Node3::handleDataAggregation() {
    EV << "Handling data aggregation.\n";
    double sum_node31 = 0.0;
    int count_node31 = 0;
    double sum_node32 = 0.0;
    int count_node32 = 0;
    for (double value : data_values_node31) {
        sum_node31 += value;
        count_node31++;
    }
    for (double value : data_values_node32) {
        sum_node32 += value;
        count_node32++;
    }
    if (count_node31 == 0 && count_node32 == 0) {
        EV << "No data received from either node for aggregation.\n";
        return;
    }
    double meanValue_node31 = (count_node31 > 0) ? (sum_node31 / count_node31) : 0.0;
    double meanValue_node32 = (count_node32 > 0) ? (sum_node32 / count_node32) : 0.0;
    EV << "Aggregated data mean value for Node_31: " << meanValue_node31 << "\n";
    EV << "Aggregated data mean value for Node_32: " << meanValue_node32 << "\n";
    cMessage *aggregationMsg = new cMessage("AggregatedData");
    aggregationMsg->addPar("mean_node31") = meanValue_node31;
    aggregationMsg->addPar("mean_node32") = meanValue_node32;
    send(aggregationMsg, "out", 2); // Assuming 'outGate' is defined and connected
    bubble("AggregationMessage Sent from Hub_Node3!");
}

void Hub_Node3::handleConfigEnergyMessage(cMessage *msg) {
    EV << "Handling config and energy message in Hub_Node3.\n";
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param 1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // Initialize the sensors
     scheduleAt(simTime() + Δ_M, new cMessage("TimeInterruptReceiveMandatoryData"));

    // Handle configuration and energy information as needed
    // For example, store values for later use, update internal state, etc.
}


