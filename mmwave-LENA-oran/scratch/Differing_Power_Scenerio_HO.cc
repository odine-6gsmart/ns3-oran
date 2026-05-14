/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/* *
 * Copyright (c) 2024 Orange Innovation Poland
 * Copyright (c) 2024 Orange Innovation Egypt
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/point-to-point-helper.h"
 #include <ns3/lte-ue-net-device.h>
 #include "ns3/mmwave-helper.h"
 #include "ns3/epc-helper.h"
 #include "ns3/mmwave-point-to-point-epc-helper.h"
 #include "../src/mmwave/model/node-container-manager.h"
 #include "ns3/lte-helper.h"
 #include <sys/time.h>
 #include <ctime>
 #include <sys/types.h>
 #include <iostream>
 #include <stdlib.h>
 #include <list>
 #include <random>
 #include <chrono>
 #include <cmath>
 #include <fstream>
 #include <algorithm>
 #include <iomanip>
 #include <sstream>
 #include "ns3/basic-energy-source-helper.h"
 #include "ns3/mmwave-radio-energy-model-enb-helper.h"
 #include "ns3/isotropic-antenna-model.h"
 #include "ns3/propagation-module.h"
 #include "ns3/antenna-module.h"
 #include "ns3/spectrum-module.h"
 #include <ns3/two-ray-spectrum-propagation-loss-model.h>
 #include <ns3/channel-condition-model.h>
#include "ns3/lte-enb-rrc.h"
#include "ns3/mmwave-ue-net-device.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-enb-phy.h"
#include "ns3/mmwave-spectrum-phy.h"
 #include "ns3/uniform-planar-array.h"
 #include <cstdio>
 
 using namespace ns3;
 using namespace mmwave;

 std::map<uint64_t, uint16_t> imsi_cellid;
 std::map<uint16_t, std::set<uint64_t>> imsi_list;
 std::map<uint16_t, Ptr < Node>> cellid_node;
 std::map<uint32_t, uint16_t> ue_cellid_usinghandover;
 std::map<uint64_t, uint32_t> ueimsi_nodeid;
 std::map<uint64_t, int> ue_assoc_list;
 double maxXAxis;
 double maxYAxis;
 double totalnewEnergyConsumption_storage[10] = {0};
 double totaloldEnergyConsumption_storage[10] = {0};
 double current_energy_consumption[10] = {0};
 
// Global file stream for UE position export
static std::ofstream g_uePositionFile;
// Runtime flag (derived from GlobalValue g_exportUEPositionsFlag)
static bool g_exportUEPositionsEnabled = false;

// Handover logging file
static std::ofstream g_handoverLogFile;

// Global file stream for Network Configuration export
static std::ofstream g_networkConfigFile;

// *** Handover Mechanism Globals ***
// These are populated after device installation and synced with runtime control
NetDeviceContainer g_mmWaveEnbDevs;  // Store mmWave eNB devices globally
NetDeviceContainer g_ueDevs;          // Store UE devices globally
Vector g_bs1Pos, g_bs2Pos;            // BS positions (set after installation)
double g_txPower[2] = {38.0, 38.0};   // Current TxPower per cell (sync with runtime control)
double g_currentTilt[2] = {10.0, 10.0}; // Current Tilt per cell (sync with runtime control)
double g_a3Offset[2] = {0.0, 0.0};    // Per-cell A3 offset (cell handover aggressiveness)
Ptr<MmWaveHelper> g_mmwaveHelper;     // Helper pointer for accessing propagation model
long g_handoverCount = 0;             // Track handover events
uint16_t g_cellId1 = 0;               // Cell ID for first eNB (for mapping)
uint16_t g_cellId2 = 0;               // Cell ID for second eNB (for mapping)

// *** TTT and Freeze Timer Configuration ***
const double HO_TTT = 0.320;          // Time-to-Trigger (seconds)
const double HO_FREEZE = 2.000;       // Handover freeze timer (seconds)
std::map<uint32_t, double> g_lastHoTime;       // Last HO time per UE NodeId
std::map<uint32_t, double> g_hoTriggerStart;   // TTT start time per UE NodeId

NS_LOG_COMPONENT_DEFINE ("DifferingPowerScenario");
 
 void
 EnergyConsumptionUpdate (int nodeIndex, std::string filename, double totaloldEnergyConsumption,
                          double totalnewEnergyConsumption)
 {
   Time currentTime = Simulator::Now ();
   std::ofstream outFile;
   outFile.open (filename, std::ios_base::out | std::ios_base::app);
   outFile << currentTime.GetSeconds () << "," << totalnewEnergyConsumption << ","
           << (totalnewEnergyConsumption - totaloldEnergyConsumption) << std::endl;
   totalnewEnergyConsumption_storage[nodeIndex] = totalnewEnergyConsumption;
 }
 
 void
 EnergyConsumptionPrint (int nodeIndex)
 {
   NS_LOG_UNCOND ("Total energy consumption for mmWave cell "
                  << nodeIndex + 2 << ": " << totalnewEnergyConsumption_storage[nodeIndex] << "J"
                  << " at time " << Simulator::Now ().GetSeconds ()
                  << ", diff from last measurement is: "
                  << (totalnewEnergyConsumption_storage[nodeIndex] -
                      totaloldEnergyConsumption_storage[nodeIndex])
                  << "J");
   totalnewEnergyConsumption_storage[nodeIndex] = totalnewEnergyConsumption_storage[nodeIndex];
   current_energy_consumption[nodeIndex] =
       totalnewEnergyConsumption_storage[nodeIndex] - totaloldEnergyConsumption_storage[nodeIndex];
  totaloldEnergyConsumption_storage[nodeIndex] = totalnewEnergyConsumption_storage[nodeIndex];
}

// *** Handover Mechanism Functions ***

/**
 * Calculate RSRP using realistic propagation model
 * Uses the actual propagation model from mmwaveHelper (Hybrid/TwoRay)
 * @param uePos UE position
 * @param bsPos BS position
 * @param bsIndex BS index (0 or 1)
 * @param enbDev eNB device pointer
 * @return RSRP in dBm
 */
double CalculateRSRPRealistic(Vector uePos, Vector bsPos, int bsIndex,
                               Ptr<MmWaveEnbNetDevice> enbDev)
{
  // 1. Get distance
  double dx = uePos.x - bsPos.x;
  double dy = uePos.y - bsPos.y;
  double dz = uePos.z - bsPos.z;
  double dist = sqrt(dx*dx + dy*dy + dz*dz);
  if (dist < 1.0) dist = 1.0;
  
  // 2. Get propagation model from helper
  Ptr<PropagationLossModel> pathlossModel = nullptr;
  if (g_mmwaveHelper)
  {
    pathlossModel = g_mmwaveHelper->GetPathLossModel(0);
  }
  
  double pathlossDb = 0.0;
  
  if (pathlossModel)
  {
    // Use realistic propagation model to calculate pathloss
    // Create temporary mobility models for calculation
    Ptr<Node> tempEnbNode = enbDev->GetNode();
    Ptr<MobilityModel> tempEnbMob = tempEnbNode->GetObject<MobilityModel>();
    
    Ptr<Node> tempUeNode = CreateObject<Node>();
    Ptr<MobilityModel> tempUeMob = CreateObject<ConstantPositionMobilityModel>();
    tempUeMob->SetPosition(uePos);
    tempUeNode->AggregateObject(tempUeMob);
    
    // Calculate pathloss using realistic model
    double txPowerDbm = g_txPower[bsIndex];
    double rxPowerDbm = pathlossModel->CalcRxPower(txPowerDbm, tempEnbMob, tempUeMob);
    pathlossDb = txPowerDbm - rxPowerDbm;
  }
  else
  {
    // Fallback to Friis if propagation model not available
    double freq = 28e9; // mmWave frequency
    double lambda = 299792458.0 / freq;
    pathlossDb = -20 * log10(lambda / (4 * M_PI * dist));
    NS_LOG_DEBUG("CalculateRSRPRealistic: Using Friis fallback (propagation model not available)");
  }
  
  // 3. Calculate antenna gain based on elevation angle and tilt
  double dist3d = dist;
  double theta_rad = acos(dz / dist3d); // 0=Up, 90=Horizon
  double theta_deg = theta_rad * 180.0 / M_PI;
  
  double tilt = g_currentTilt[bsIndex];
  double boresight = 90.0 + tilt; // Boresight is 90 deg + tilt (downward)
  double hpbw = 10.0; // Approximate HPBW for UniformPlanarArray
  
  // Antenna gain approximation (parabolic model, matches UniformPlanarArray behavior)
  double gain = 18.0 - 12.0 * pow((theta_deg - boresight) / hpbw, 2);
  if (gain < -30.0) gain = -30.0; // Minimum gain limit
  
  // 4. Calculate RSRP
  double txPower = g_txPower[bsIndex];
  double rsrp = txPower + gain - pathlossDb;
  
  return rsrp;
}

/**
 * Execute handover by updating PHY, MAC, and RRC layers
 * Updates RRC's m_ueMap to ensure E2 reports (DU, CU-CP, CU-UP) reflect the handover
 * @param ueDev UE device
 * @param targetEnb Target eNB device
 */
void HandoverTo(Ptr<MmWaveUeNetDevice> ueDev, Ptr<MmWaveEnbNetDevice> targetEnb)
{
  if (!ueDev || !targetEnb)
  {
    NS_LOG_WARN("HandoverTo: Invalid device pointer(s)");
    return;
  }

  Ptr<MmWaveEnbNetDevice> sourceEnb = ueDev->GetTargetEnb();
  if (!sourceEnb || sourceEnb == targetEnb)
  {
    return;
  }

  uint16_t rnti = ueDev->GetRrc()->GetRnti();
  uint16_t targetCellId = targetEnb->GetCellId();
  uint64_t imsi = ueDev->GetImsi();

  NS_LOG_UNCOND("Triggering X2 Handover: UE (IMSI " << imsi << ") from Cell "
                                                    << sourceEnb->GetCellId() << " to Cell "
                                                    << targetCellId);

  // Trigger the Standard X2 Handover via RRC
  sourceEnb->GetRrc()->SendHandoverRequest(rnti, targetCellId);

  // Update handover count and logs
  g_handoverCount++;
  double timestamp = Simulator::Now().GetSeconds();
  if (g_handoverLogFile.is_open())
  {
    g_handoverLogFile << timestamp << ",HO," << imsi << "," << sourceEnb->GetCellId() << "," << targetCellId << "," << rnti << std::endl;
    g_handoverLogFile.flush();
  }
}

/**
 * Periodic handover checking function
 * Compares RSRP for both cells and triggers handover if threshold is met
 * Runs every 0.1 seconds
 */
void CheckHandover()
{
  if (g_ueDevs.GetN() == 0 || g_mmWaveEnbDevs.GetN() < 2)
  {
    // Not ready yet, reschedule
    Simulator::Schedule(Seconds(0.1), &CheckHandover);
    return;
  }
  
  // Get eNB devices
  Ptr<MmWaveEnbNetDevice> enb1 = DynamicCast<MmWaveEnbNetDevice>(g_mmWaveEnbDevs.Get(0));
  Ptr<MmWaveEnbNetDevice> enb2 = DynamicCast<MmWaveEnbNetDevice>(g_mmWaveEnbDevs.Get(1));
  
  if (!enb1 || !enb2)
  {
    NS_LOG_WARN("CheckHandover: Failed to get eNB devices");
    Simulator::Schedule(Seconds(0.1), &CheckHandover);
    return;
  }
  
  // Check each UE
  for (uint32_t i = 0; i < g_ueDevs.GetN(); ++i)
  {
    Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice>(g_ueDevs.Get(i));
    if (!ueDev) continue;
    
    // Get UE position
    Ptr<Node> ueNode = ueDev->GetNode();
    if (!ueNode) continue;
    
    Ptr<MobilityModel> ueMob = ueNode->GetObject<MobilityModel>();
    if (!ueMob) continue;
    
    Vector uePos = ueMob->GetPosition();
    
    // Calculate RSRP for both cells using realistic propagation model
    double rsrp1 = CalculateRSRPRealistic(uePos, g_bs1Pos, 0, enb1);
    double rsrp2 = CalculateRSRPRealistic(uePos, g_bs2Pos, 1, enb2);
    
    // Get current serving cell
    Ptr<MmWaveEnbNetDevice> currentEnb = ueDev->GetTargetEnb();
    if (!currentEnb) continue;
    
    uint32_t nodeId = ueNode->GetId();
    double now = Simulator::Now().GetSeconds();

    // Freeze timer check: prevent rapid handovers (ping-pong)
    if (g_lastHoTime.count(nodeId) && (now - g_lastHoTime[nodeId] < HO_FREEZE))
    {
      continue;
    }

    // Determine current and target cell indices
    int currentIndex = (currentEnb == enb1) ? 0 : 1;
    
    // Handover decision: check if neighbor RSRP exceeds serving RSRP + offset
    bool shouldTrigger = false;
    if (currentIndex == 0)
    {
      if (rsrp2 > rsrp1 + g_a3Offset[0])  // Use Cell 1's A3 offset
      {
        shouldTrigger = true;
      }
    }
    else
    {
      if (rsrp1 > rsrp2 + g_a3Offset[1])  // Use Cell 2's A3 offset
      {
        shouldTrigger = true;
      }
    }
    
    // TTT Logic: Condition must persist for HO_TTT seconds
    if (shouldTrigger)
    {
      if (!g_hoTriggerStart.count(nodeId))
      {
        g_hoTriggerStart[nodeId] = now; // Start TTT timer
      }
      else if (now - g_hoTriggerStart[nodeId] >= HO_TTT)
      {
        // TTT expired, execute handover
        int targetIndex = (currentIndex == 0) ? 1 : 0;
        Ptr<MmWaveEnbNetDevice> targetEnb = (targetIndex == 0) ? enb1 : enb2;
        
        HandoverTo(ueDev, targetEnb);
        
        g_lastHoTime[nodeId] = now;
        g_hoTriggerStart.erase(nodeId);
        
        NS_LOG_UNCOND("[" << now << "s] Handover: UE (Node " << nodeId 
                          << ", IMSI " << ueDev->GetImsi() << ") Cell " << (currentIndex+1) 
                          << " -> Cell " << (targetIndex+1) 
                          << " (RSRP " << std::fixed << std::setprecision(2) << rsrp1 
                          << " vs " << rsrp2 << " dBm, A3=" << g_a3Offset[currentIndex] << "dB)");
      }
    }
    else
    {
      // Condition not met or no longer met, reset TTT timer
      g_hoTriggerStart.erase(nodeId);
    }
  }
  
  // Schedule next check (every 0.1 seconds)
  Simulator::Schedule(Seconds(0.1), &CheckHandover);
}

// Function to log UE and BS positions every 100ms
 void
 LogUEPositions (NodeContainer& ueNodes, NetDeviceContainer& ueDevs,
                 NodeContainer& enbNodes, NetDeviceContainer& enbDevs)
 {
   if (!g_exportUEPositionsEnabled || !g_uePositionFile.is_open())
   {
     return;
   }
 
   double currentTime = Simulator::Now().GetSeconds();
   
   // Log BS positions (static, but log once per frame for visualization)
   for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
   {
     Ptr<Node> enbNode = enbNodes.Get(i);
     Ptr<MmWaveEnbNetDevice> enbDev = enbDevs.Get(i)->GetObject<MmWaveEnbNetDevice>();
     if (!enbDev)
     {
       continue;
     }
     
     Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
     if (!enbMobility)
     {
       continue;
     }
     
     Vector bsPos = enbMobility->GetPosition();
     uint16_t cellId = enbDev->GetCellId();
     
     g_uePositionFile << std::fixed << std::setprecision(3)
                      << currentTime << ",BS," << cellId << ","
                      << bsPos.x << "," << bsPos.y << "," << bsPos.z << "," << std::endl;
   }
   
   // Log all UE positions
   for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
   {
     Ptr<Node> ueNode = ueNodes.Get(u);
     Ptr<NetDevice> ueDev = ueDevs.Get(u);
     Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
     
     if (!ueMobility)
     {
       continue;
     }
     
     Vector uePos = ueMobility->GetPosition();
     
     // Get serving cell ID and IMSI
     uint16_t servingCellId = 0;
     uint64_t imsi = 0;
     
     Ptr<MmWaveUeNetDevice> mmWaveUeDev = DynamicCast<MmWaveUeNetDevice>(ueDev);
     if (mmWaveUeDev)
     {
       imsi = mmWaveUeDev->GetImsi();
       Ptr<MmWaveEnbNetDevice> targetEnb = mmWaveUeDev->GetTargetEnb();
       if (targetEnb)
       {
         servingCellId = targetEnb->GetCellId();
       }
     }
     
     g_uePositionFile << std::fixed << std::setprecision(3)
                      << currentTime << ",UE," << imsi << ","
                      << uePos.x << "," << uePos.y << "," << uePos.z << ","
                      << servingCellId << std::endl;
   }
   
   // Schedule next logging (100ms = 0.1s)
   Simulator::Schedule(Seconds(0.1), &LogUEPositions, 
                       ueNodes, ueDevs, enbNodes, enbDevs);
 }
 
 /**
  * Log Network Configurations (TxPower, Tilt, A3 Offset) every 100ms
  */
 void LogNetworkConfigurations()
 {
   if (!g_networkConfigFile.is_open())
   {
     return;
   }
 
   double currentTime = Simulator::Now().GetSeconds();
   
   // Format: Time,Cell1_TxPower,Cell1_Tilt,Cell1_A3,Cell2_TxPower,Cell2_Tilt,Cell2_A3
   g_networkConfigFile << std::fixed << std::setprecision(3)
                       << currentTime << ","
                       << g_txPower[0] << "," << g_currentTilt[0] << "," << g_a3Offset[0] << ","
                       << g_txPower[1] << "," << g_currentTilt[1] << "," << g_a3Offset[1] << std::endl;
 
   // Schedule next logging (100ms = 0.1s)
   Simulator::Schedule(Seconds(0.1), &LogNetworkConfigurations);
 }
 
 // *** Runtime Control Helper Functions ***
 
 /**
  * Change TxPower of a specific cell at runtime
  * @param enbNetDevice Pointer to MmWaveEnbNetDevice
  * @param newPower New TxPower in dBm
  * @param cellId Cell ID for logging
  * @param minPower Minimum allowed power (for validation)
  * @param maxPower Maximum allowed power (for validation)
  */
 void ChangeCellTxPower(Ptr<MmWaveEnbNetDevice> enbNetDevice, double newPower, uint16_t cellId,
                        double minPower, double maxPower)
 {
   if (!enbNetDevice)
   {
     NS_LOG_ERROR("ChangeCellTxPower: Invalid device pointer");
     return;
   }
   
   // Validate power range
   if (newPower < minPower || newPower > maxPower)
   {
     NS_LOG_WARN("ChangeCellTxPower: Power " << newPower << " dBm out of range [" 
                 << minPower << ", " << maxPower << "] for Cell " << cellId);
     return;
   }
   
   Ptr<MmWaveEnbPhy> phy = enbNetDevice->GetPhy();
   if (!phy)
   {
     NS_LOG_ERROR("ChangeCellTxPower: Failed to get PHY for Cell " << cellId);
     return;
   }
   
   // Set the new power value
   phy->SetTxPower(newPower);
   
   // CRITICAL: Update the Power Spectral Density (PSD) that's actually used for transmission
   // SetTxPower only updates the internal variable, but doesn't update the PSD
   Ptr<MmWaveSpectrumPhy> spectrumPhy = phy->GetDlSpectrumPhy();
   if (spectrumPhy)
   {
     // Create new PSD with updated power (uses current subchannels)
     Ptr<SpectrumValue> txPsd = phy->CreateTxPowerSpectralDensity();
     if (txPsd)
     {
      spectrumPhy->SetTxPowerSpectralDensity(txPsd);
      NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Cell " << cellId 
                        << " TxPower changed to " << newPower << " dBm (PSD updated)");
      
      // *** Sync global variable for handover mechanism ***
      if (cellId == g_cellId1)
      {
        g_txPower[0] = newPower;
      }
      else if (cellId == g_cellId2)
      {
        g_txPower[1] = newPower;
      }
    }
    else
    {
      NS_LOG_WARN("ChangeCellTxPower: Failed to create PSD for Cell " << cellId);
    }
  }
  else
  {
    NS_LOG_WARN("ChangeCellTxPower: Failed to get SpectrumPhy for Cell " << cellId 
                << " (power set but PSD not updated)");
  }
}
 
 /**
  * Change E-Tilt (DowntiltAngle) of a specific cell at runtime
  * @param enbNetDevice Pointer to MmWaveEnbNetDevice
  * @param newTiltDegrees New tilt angle in degrees
  * @param cellId Cell ID for logging
  * @param minTilt Minimum allowed tilt (for validation)
  * @param maxTilt Maximum allowed tilt (for validation)
  */
 void ChangeCellTilt(Ptr<MmWaveEnbNetDevice> enbNetDevice, double newTiltDegrees, uint16_t cellId,
                     double minTilt, double maxTilt)
 {
   if (!enbNetDevice)
   {
     NS_LOG_ERROR("ChangeCellTilt: Invalid device pointer");
     return;
   }
   
   // Validate tilt range
   if (newTiltDegrees < minTilt || newTiltDegrees > maxTilt)
   {
     NS_LOG_WARN("ChangeCellTilt: Tilt " << newTiltDegrees << " degrees out of range [" 
                 << minTilt << ", " << maxTilt << "] for Cell " << cellId);
     return;
   }
   
   Ptr<MmWaveEnbPhy> phy = enbNetDevice->GetPhy();
   if (!phy)
   {
     NS_LOG_ERROR("ChangeCellTilt: Failed to get PHY for Cell " << cellId);
     return;
   }
   
   Ptr<MmWaveSpectrumPhy> spectrumPhy = phy->GetDlSpectrumPhy();
   if (!spectrumPhy)
   {
     NS_LOG_ERROR("ChangeCellTilt: Failed to get SpectrumPhy for Cell " << cellId);
     return;
   }
   
   Ptr<MmWaveBeamformingModel> bfModel = spectrumPhy->GetBeamformingModel();
   if (!bfModel)
   {
     NS_LOG_ERROR("ChangeCellTilt: Failed to get BeamformingModel for Cell " << cellId);
     return;
   }
   
   Ptr<PhasedArrayModel> antenna = bfModel->GetAntenna();
   if (!antenna)
   {
     NS_LOG_ERROR("ChangeCellTilt: Failed to get Antenna for Cell " << cellId);
     return;
   }
   
   Ptr<UniformPlanarArray> upa = DynamicCast<UniformPlanarArray>(antenna);
   if (!upa)
   {
     NS_LOG_ERROR("ChangeCellTilt: Antenna is not UniformPlanarArray for Cell " << cellId);
     return;
   }
   
  double newTiltRadians = newTiltDegrees * M_PI / 180.0;
  upa->SetAttribute("DowntiltAngle", DoubleValue(newTiltRadians));
  NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Cell " << cellId 
                    << " E-Tilt changed to " << newTiltDegrees << " degrees (" 
                    << newTiltRadians << " radians)");
  
  // *** Sync global variable for handover mechanism ***
  if (cellId == g_cellId1)
  {
    g_currentTilt[0] = newTiltDegrees;
  }
  else if (cellId == g_cellId2)
  {
    g_currentTilt[1] = newTiltDegrees;
  }
}

/**
 * Change A3 offset (handover trigger threshold) of a specific cell at runtime
 * @param cellId Cell ID for logging
 * @param newA3OffsetDb New A3 offset in dB
 * @param minA3 Minimum allowed A3 offset
 * @param maxA3 Maximum allowed A3 offset
 */
void ChangeCellA3Offset(uint16_t cellId, double newA3OffsetDb,
                        double minA3, double maxA3)
{
  // Validate A3 offset range
  if (newA3OffsetDb < minA3 || newA3OffsetDb >  maxA3)
  {
    NS_LOG_WARN("ChangeCellA3Offset: A3 offset " << newA3OffsetDb 
                << " dB out of range [" << minA3 << ", " << maxA3 << "] for Cell " << cellId);
    return;
  }
  
  // Update global array based on cell ID
  if (cellId == g_cellId1)
  {
    g_a3Offset[0] = newA3OffsetDb;
    NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Cell " << cellId 
                  << " A3 offset changed to " << newA3OffsetDb << " dB");
  }
  else if (cellId == g_cellId2)
  {
    g_a3Offset[1] = newA3OffsetDb;
    NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Cell " << cellId 
                  << " A3 offset changed to " << newA3OffsetDb << " dB");
  }
  else
  {
    NS_LOG_WARN("ChangeCellA3Offset: Invalid cell ID " << cellId);
  }
}
 
 /**
  * Check control file for external commands
  * Format: <COMMAND> <CELL_ID> <VALUE>
  * 
  * Commands:
  *   POWER <cellId> <power_dBm>     - Change TxPower
  *   TILT <cellId> <tilt_degrees>   - Change E-Tilt
  * 
  * The file is deleted after reading to avoid re-execution.
  */
 void CheckControlFile(Ptr<MmWaveEnbNetDevice> enbDev1, Ptr<MmWaveEnbNetDevice> enbDev2,
                       double txPowerMin, double txPowerMax, double tiltMin, double tiltMax,
                       double a3Min, double a3Max, uint32_t pollIntervalMs)
 {
   std::string controlFile = "runtime_control.txt";
   std::ifstream file(controlFile);
   
   if (file.good())
   {
     std::string command;
     int cellId;
     double value;
     
     while (file >> command >> cellId >> value)
     {
       // Validate cell ID
       if (cellId != 1 && cellId != 2)
       {
         NS_LOG_WARN("Invalid cell ID: " << cellId << " (must be 1 or 2)");
         continue;
       }
       
       // Get target device
       Ptr<MmWaveEnbNetDevice> targetDev = (cellId == 1) ? enbDev1 : enbDev2;
       if (!targetDev)
       {
         NS_LOG_ERROR("Invalid device pointer for Cell " << cellId);
         continue;
       }
       
       uint16_t actualCellId = targetDev->GetCellId();
       
       // Execute command
       if (command == "POWER" || command == "power" || command == "P")
       {
         ChangeCellTxPower(targetDev, value, actualCellId, txPowerMin, txPowerMax);
       }
       else if (command == "TILT" || command == "tilt" || command == "T")
       {
         ChangeCellTilt(targetDev, value, actualCellId, tiltMin, tiltMax);
       }
       else if (command == "A3" || command == "a3")
       {
         ChangeCellA3Offset(actualCellId, value, a3Min, a3Max);
       }
       else
       {
         NS_LOG_WARN("Unknown command: " << command << " (valid: POWER, TILT, A3)");
       }
     }

     
     file.close();
     
     // Delete file after reading to avoid re-execution
     if (std::remove(controlFile.c_str()) != 0)
     {
       NS_LOG_WARN("Failed to delete control file: " << controlFile);
     }
   }
   
   // Schedule next check (configurable interval)
   Simulator::Schedule(MilliSeconds(pollIntervalMs), &CheckControlFile, enbDev1, enbDev2,
                      txPowerMin, txPowerMax, tiltMin, tiltMax, a3Min, a3Max, pollIntervalMs);
 }
 
 // Global Values
 static ns3::GlobalValue g_bufferSize("bufferSize", "RLC tx buffer size (MB)",
                                       ns3::UintegerValue(10),
                                       ns3::MakeUintegerChecker<uint32_t>());
 
 static ns3::GlobalValue g_enableTraces("enableTraces", "If true, generate ns-3 traces",
                                         ns3::BooleanValue(true), ns3::MakeBooleanChecker());

static ns3::GlobalValue g_enableEnergyMonitoring("enableEnergyMonitoring",
                                                   "If true, enable energy consumption monitoring and CSV output",
                                                   ns3::BooleanValue(false),
                                                   ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_e2lteEnabled("e2lteEnabled", "If true, send LTE E2 reports",
                                         ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_e2nrEnabled("e2nrEnabled", "If true, send NR E2 reports",
                                        ns3::BooleanValue(true), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_e2du("e2du", "If true, send DU reports", ns3::BooleanValue(true),
                                 ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_e2cuUp("e2cuUp", "If true, send CU-UP reports", ns3::BooleanValue(true),
                                   ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_e2cuCp("e2cuCp", "If true, send CU-CP reports", ns3::BooleanValue(true),
                                   ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_reducedPmValues("reducedPmValues", "If true, use a subset of the the pm containers",
                                           ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue
     g_hoSinrDifference("hoSinrDifference",
                         "The value for which an handover between MmWave eNB is triggered",
                         ns3::DoubleValue(3), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue
     g_indicationPeriodicity("indicationPeriodicity",
                              "E2 Indication Periodicity reports (value in seconds)",
                              ns3::DoubleValue(0.1), ns3::MakeDoubleChecker<double>(0.01, 2.0));
 
 static ns3::GlobalValue g_simTime("simTime", "Simulation time in seconds", ns3::DoubleValue(1000),
                                    ns3::MakeDoubleChecker<double>(0.1, 100000.0));
 
 static ns3::GlobalValue g_outageThreshold("outageThreshold",
                                            "SNR threshold for outage events [dB]", // use -1000.0 with NoAuto
                                            ns3::DoubleValue(-5.0),
                                            ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_numberOfRaPreambles(
     "numberOfRaPreambles",
     "how many random access preambles are available for the contention based RACH process",
     ns3::UintegerValue(40), // Indicated for TS use case, 52 is default
     ns3::MakeUintegerChecker<uint8_t>());
 
 static ns3::GlobalValue
     g_handoverMode("handoverMode",
                     "HO euristic to be used,"
                     "can be only \"NoAuto\", \"FixedTtt\", \"DynamicTtt\",   \"Threshold\"",
                     ns3::StringValue("DynamicTtt"), ns3::MakeStringChecker());
 
 static ns3::GlobalValue g_e2TermIp("e2TermIp", "The IP address of the RIC E2 termination",
                                     ns3::StringValue("127.0.0.1"), ns3::MakeStringChecker());
 
 static ns3::GlobalValue
         g_enableE2FileLogging("enableE2FileLogging",
                               "If true, generate offline file logging instead of connecting to RIC",
                               ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 static ns3::GlobalValue g_e2_func_id("KPM_E2functionID", "Function ID to subscribe",
                                       ns3::DoubleValue(2),
                                       ns3::MakeDoubleChecker<double>());
 static ns3::GlobalValue g_rc_e2_func_id("RC_E2functionID", "Function ID to subscribe",
                                          ns3::DoubleValue(3),
                                          ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_e2andLogging("e2andLogging", "If true, both RIC connection and file logging",
                                       ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_controlFileName("controlFileName",
                                            "The path to the control file (can be absolute)",
                                            ns3::StringValue(""),
                                            ns3::MakeStringChecker());
 
 static ns3::GlobalValue lteEnb_nodes ("N_LteEnbNodes", "Number of LteEnbNodes",
                                       ns3::UintegerValue (0),
                                       ns3::MakeUintegerChecker<uint8_t> ());
 
 static ns3::GlobalValue ue_s ("N_Ues", "Number of User Equipments",
                               ns3::UintegerValue (20),
                               ns3::MakeUintegerChecker<uint32_t> ());
 
 static ns3::GlobalValue center_freq ("CenterFrequency", "Center Frequency Value",
                                      ns3::DoubleValue (3.5e9),
                                      ns3::MakeDoubleChecker<double> ());
 
 static ns3::GlobalValue bandwidth_value ("Bandwidth", "Bandwidth Value",
                                          ns3::DoubleValue (20e6),
                                          ns3::MakeDoubleChecker<double> ());
 
 static ns3::GlobalValue num_antennas_McUe ("N_AntennasMcUe", "Number of Antenna as McUe",
                                       ns3::UintegerValue (1),
                                       ns3::MakeUintegerChecker<uint32_t> ());
 
 static ns3::GlobalValue num_antennas_MmWave ("N_AntennasMmWave", "Number of Antenna as MmWave",
                                       ns3::UintegerValue (1),
                                       ns3::MakeUintegerChecker<uint32_t> ());
 
 static ns3::GlobalValue interside_distance_value_ue ("IntersideDistanceUEs", "Interside Distance Value",
                                       ns3::DoubleValue (100),
                                       ns3::MakeDoubleChecker<double> ());
 static ns3::GlobalValue interside_distance_value_cell ("IntersideDistanceCells", "Interside Distance Value",
                                                   ns3::DoubleValue (500),
                                                   ns3::MakeDoubleChecker<double> ());
 
 // *** NEW: Differential TxPower Parameters ***
 static ns3::GlobalValue g_txPower1("TxPower1", "Transmission Power for Cell 1 in dBm",
                                   ns3::DoubleValue(46.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_txPower2("TxPower2", "Transmission Power for Cell 2 in dBm",
                                   ns3::DoubleValue(46.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_tilt("Tilt", "Antenna Downtilt in degrees",
                                ns3::DoubleValue(0.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_useFriis("useFriis", "Use Friis Propagation Model (Faster) instead of 3GPP",
                                    ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_useHybrid("useHybrid", "Use Hybrid Propagation Model (LogDistance + Random shadowing) with 3GPP Antenna",
                                     ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_enableTiltTwoRay("enableTiltTwoRay", 
                                            "If true, enable TwoRaySpectrumPropagationLossModel for E-Tilt support in Hybrid mode (default: false for speed)",
                                            ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_tReorderingTimer("tReorderingTimer", "RLC UM t-Reordering timer in milliseconds",
                                            ns3::DoubleValue(35.0), ns3::MakeDoubleChecker<double>());
 
 
 static ns3::GlobalValue g_mobility("Mobility", "Enable UE Mobility (RandomWalk2d)",
                                    ns3::BooleanValue(false), ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_rngRun("RngRun", "Rng Run ID",
                                  ns3::UintegerValue(1), ns3::MakeUintegerChecker<uint32_t>());
 
 // Control flag (CLI / GlobalValue) for UE position export
 static ns3::GlobalValue g_exportUEPositionsFlag(
     "exportUEPositions",
     "If true, export UE positions every 100ms to UEPosition.txt (in build directory)",
     ns3::BooleanValue(false),
     ns3::MakeBooleanChecker());
 
 // *** Runtime Control: Configurable Action Spaces ***
 static ns3::GlobalValue g_enableRuntimeControl("enableRuntimeControl",
                                                "If true, enable external runtime control via runtime_control.txt",
                                                ns3::BooleanValue(true),
                                                ns3::MakeBooleanChecker());
 
 static ns3::GlobalValue g_txPowerMin("TxPowerMin", "Minimum TxPower for action space (dBm)",
                                      ns3::DoubleValue(0.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_txPowerMax("TxPowerMax", "Maximum TxPower for action space (dBm)",
                                      ns3::DoubleValue(50.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_tiltMin("TiltMin", "Minimum E-Tilt for action space (degrees)",
                                   ns3::DoubleValue(-45.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_tiltMax("TiltMax", "Maximum E-Tilt for action space (degrees)",
                                   ns3::DoubleValue(45.0), ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_controlPollInterval("ControlPollInterval",
                                                "Control file polling interval in milliseconds (simulation time)",
                                                ns3::UintegerValue(10),  // Default: 10ms for better responsiveness
                                                ns3::MakeUintegerChecker<uint32_t>());
 
 static ns3::GlobalValue g_a3OffsetMin("A3OffsetMin", 
                                       "Minimum A3 offset for action space (dB)",
                                       ns3::DoubleValue(-10.0), 
                                       ns3::MakeDoubleChecker<double>());
 
 static ns3::GlobalValue g_a3OffsetMax("A3OffsetMax", 
                                       "Maximum A3 offset for action space (dB)",
                                       ns3::DoubleValue(15.0), 
                                       ns3::MakeDoubleChecker<double>());
 
 int
 main(int argc, char *argv[]) {
   LogComponentEnableAll(LOG_PREFIX_ALL);
 
   maxXAxis = 2000;
   maxYAxis = 2000;
 
   CommandLine cmd;
   cmd.Parse(argc, argv);
 
   bool harqEnabled = true;
 
   UintegerValue uintegerValue;
   BooleanValue booleanValue;
   StringValue stringValue;
   DoubleValue doubleValue;
 
   GlobalValue::GetValueByName("RngRun", uintegerValue);
   uint32_t rngRun = uintegerValue.Get();
   RngSeedManager::SetSeed(1);
   RngSeedManager::SetRun(rngRun);
 
   GlobalValue::GetValueByName("hoSinrDifference", doubleValue);
   double hoSinrDifference = doubleValue.Get();
   GlobalValue::GetValueByName("bufferSize", uintegerValue);
   uint32_t bufferSize = uintegerValue.Get();
   GlobalValue::GetValueByName("enableTraces", booleanValue);
   bool enableTraces = booleanValue.Get();
   GlobalValue::GetValueByName("outageThreshold", doubleValue);
   double outageThreshold = doubleValue.Get();
   GlobalValue::GetValueByName("handoverMode", stringValue);
   std::string handoverMode = stringValue.Get();
   GlobalValue::GetValueByName("e2TermIp", stringValue);
   std::string e2TermIp = stringValue.Get();
   GlobalValue::GetValueByName("enableE2FileLogging", booleanValue);
   bool enableE2FileLogging = booleanValue.Get();
   GlobalValue::GetValueByName("KPM_E2functionID", doubleValue);
   double g_e2_func_id = doubleValue.Get();
   GlobalValue::GetValueByName("RC_E2functionID", doubleValue);
   double g_rc_e2_func_id = doubleValue.Get();
   GlobalValue::GetValueByName("e2andLogging", booleanValue);
   bool e2andLogging = booleanValue.Get();
   GlobalValue::GetValueByName("useFriis", booleanValue);
   bool useFriis = booleanValue.Get();
   GlobalValue::GetValueByName("useHybrid", booleanValue);
   bool useHybrid = booleanValue.Get();
   GlobalValue::GetValueByName("exportUEPositions", booleanValue);
   bool exportUEPositions = booleanValue.Get();
   
   // Get runtime control configuration
   GlobalValue::GetValueByName("enableRuntimeControl", booleanValue);
   bool enableRuntimeControl = booleanValue.Get();
   GlobalValue::GetValueByName("TxPowerMin", doubleValue);
   double txPowerMin = doubleValue.Get();
   GlobalValue::GetValueByName("TxPowerMax", doubleValue);
   double txPowerMax = doubleValue.Get();
   GlobalValue::GetValueByName("TiltMin", doubleValue);
   double tiltMin = doubleValue.Get();
   GlobalValue::GetValueByName("TiltMax", doubleValue);
   double tiltMax = doubleValue.Get();
   GlobalValue::GetValueByName("ControlPollInterval", uintegerValue);
   uint32_t controlPollIntervalMs = uintegerValue.Get();
   
   GlobalValue::GetValueByName("A3OffsetMin", doubleValue);
   double a3Min = doubleValue.Get();
   GlobalValue::GetValueByName("A3OffsetMax", doubleValue);
   double a3Max = doubleValue.Get();
 
   GlobalValue::GetValueByName("numberOfRaPreambles", uintegerValue);
   uint8_t numberOfRaPreambles = uintegerValue.Get();
 
   GlobalValue::GetValueByName("e2lteEnabled", booleanValue);
   bool e2lteEnabled = booleanValue.Get();
   GlobalValue::GetValueByName("e2nrEnabled", booleanValue);
   bool e2nrEnabled = booleanValue.Get();
   GlobalValue::GetValueByName("e2du", booleanValue);
   bool e2du = booleanValue.Get();
   GlobalValue::GetValueByName("e2cuUp", booleanValue);
   bool e2cuUp = booleanValue.Get();
   GlobalValue::GetValueByName("e2cuCp", booleanValue);
   bool e2cuCp = booleanValue.Get();
 
   GlobalValue::GetValueByName("reducedPmValues", booleanValue);
   bool reducedPmValues = booleanValue.Get();

  GlobalValue::GetValueByName("enableEnergyMonitoring", booleanValue);
  bool enableEnergyMonitoring = booleanValue.Get();
 
   GlobalValue::GetValueByName("indicationPeriodicity", doubleValue);
   double indicationPeriodicity = doubleValue.Get();
   GlobalValue::GetValueByName("controlFileName", stringValue);
   std::string controlFilename = stringValue.Get();
 
   // Get TxPower1, TxPower2, and Tilt
   GlobalValue::GetValueByName("TxPower1", doubleValue);
   double txPower1 = doubleValue.Get();
   GlobalValue::GetValueByName("TxPower2", doubleValue);
   double txPower2 = doubleValue.Get();
 
   GlobalValue::GetValueByName("Tilt", doubleValue);
   double tilt = doubleValue.Get();
   GlobalValue::GetValueByName("tReorderingTimer", doubleValue);
   double tReorderingMs = doubleValue.Get();
 
   Config::SetDefault("ns3::LteEnbNetDevice::ControlFileName", StringValue(controlFilename));
   Config::SetDefault("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue(indicationPeriodicity));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::E2Periodicity",
                       DoubleValue(indicationPeriodicity));
 
   Config::SetDefault("ns3::MmWaveHelper::E2ModeLte", BooleanValue(e2lteEnabled));
   Config::SetDefault("ns3::MmWaveHelper::E2ModeNr", BooleanValue(e2nrEnabled));
 
   Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue(e2du));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue(e2cuUp));
   Config::SetDefault("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue(e2cuUp));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue(e2cuCp));
   Config::SetDefault("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue(e2cuCp));
 
   Config::SetDefault("ns3::MmWaveEnbNetDevice::ReducedPmValues", BooleanValue(reducedPmValues));
   Config::SetDefault("ns3::LteEnbNetDevice::ReducedPmValues", BooleanValue(reducedPmValues));
 
   Config::SetDefault("ns3::LteEnbNetDevice::EnableE2FileLogging",
                       BooleanValue(enableE2FileLogging));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableE2FileLogging",
                       BooleanValue(enableE2FileLogging));
 
   Config::SetDefault("ns3::LteEnbNetDevice::KPM_E2functionID",
                       DoubleValue(g_e2_func_id));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::KPM_E2functionID",
                       DoubleValue(g_e2_func_id));
 
   Config::SetDefault("ns3::LteEnbNetDevice::RC_E2functionID",
                       DoubleValue(g_rc_e2_func_id));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::RC_E2functionID",
                       DoubleValue(g_rc_e2_func_id));
 
   Config::SetDefault("ns3::LteEnbNetDevice::e2andLogging", BooleanValue(e2andLogging));
   Config::SetDefault("ns3::MmWaveEnbNetDevice::e2andLogging", BooleanValue(e2andLogging));
 
   Config::SetDefault("ns3::MmWaveEnbMac::NumberOfRaPreambles",
                       UintegerValue(numberOfRaPreambles));
 
   Config::SetDefault("ns3::MmWaveHelper::HarqEnabled", BooleanValue(harqEnabled));
   Config::SetDefault("ns3::MmWaveHelper::UseIdealRrc", BooleanValue(true));
   Config::SetDefault("ns3::MmWaveHelper::E2TermIp", StringValue(e2TermIp));
 
   Config::SetDefault("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue(harqEnabled));
   Config::SetDefault("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue(100));
 
   // Configure antenna element: ThreeGppAntennaModel for Hybrid mode, IsotropicAntennaModel otherwise
   // Note: ThreeGppAntennaModel has fixed 65° beamwidth (per 3GPP TR 38.901) - no settable attributes
   if (useHybrid)
   {
       Ptr<ThreeGppAntennaModel> threeGppAntenna = CreateObject<ThreeGppAntennaModel>();
       Config::SetDefault("ns3::PhasedArrayModel::AntennaElement", PointerValue(threeGppAntenna));
       NS_LOG_UNCOND("Using ThreeGppAntennaModel (65° vertical and horizontal beamwidth - fixed per 3GPP TR 38.901)");
   }
   else
   {
       Config::SetDefault("ns3::PhasedArrayModel::AntennaElement",
                          PointerValue(CreateObject<IsotropicAntennaModel>()));
   }
   Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
   Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
     TimeValue (MilliSeconds (100)));
 
   Config::SetDefault("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue(MilliSeconds(10.0)));
   Config::SetDefault("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                       TimeValue(MilliSeconds(10.0)));
   Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(bufferSize * 1024 * 1024));
   Config::SetDefault("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                       UintegerValue(bufferSize * 1024 * 1024));
   Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(bufferSize * 1024 * 1024));
 
   Config::SetDefault("ns3::LteRlcUm::ReorderingTimer", TimeValue(MilliSeconds(tReorderingMs)));
   NS_LOG_UNCOND("t-Reordering Timer set to: " << tReorderingMs << " ms");
 
   Config::SetDefault("ns3::LteEnbRrc::OutageThreshold", DoubleValue(outageThreshold));
   Config::SetDefault("ns3::LteEnbRrc::SecondaryCellHandoverMode", StringValue(handoverMode));
   Config::SetDefault("ns3::LteEnbRrc::HoSinrDifference", DoubleValue(hoSinrDifference));
   Config::SetDefault("ns3::ThreeGppPropagationLossModel::Frequency",DoubleValue(3.5e9));
   // Shadowing: enabled for 3GPP, disabled for Hybrid (handled by RandomPropagationLossModel)
   Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled",BooleanValue(!useHybrid));
   
   NS_LOG_UNCOND("Differing Power Scenario Parameters:");
   NS_LOG_UNCOND("  TxPower Cell 1: " << txPower1 << " dBm");
   NS_LOG_UNCOND("  TxPower Cell 2: " << txPower2 << " dBm");
   NS_LOG_UNCOND("  Tilt: " << tilt << " degrees");
   if (useHybrid)
   {
       NS_LOG_UNCOND("  Propagation: Hybrid (LogDistance + Random Shadowing)");
       NS_LOG_UNCOND("  Antenna: ThreeGppAntennaModel (65° beamwidth)");
       NS_LOG_UNCOND("  BS Height: 15.0 m");
   }
   else if (useFriis)
   {
       NS_LOG_UNCOND("  Propagation: Friis");
   }
   else
   {
       NS_LOG_UNCOND("  Propagation: 3GPP UMi Street Canyon");
   }
 
   GlobalValue::GetValueByName ("Bandwidth", doubleValue);
   double bandwidth = doubleValue.Get ();
   GlobalValue::GetValueByName ("CenterFrequency", doubleValue);
   double centerFrequency = doubleValue.Get ();
   // GlobalValue::GetValueByName ("IntersideDistanceUEs", doubleValue);
   // double isd_ue = doubleValue.Get (); 
   GlobalValue::GetValueByName ("IntersideDistanceCells", doubleValue);
   double isd_cell = doubleValue.Get (); 
 
   GlobalValue::GetValueByName ("N_AntennasMcUe", uintegerValue);
   int numAntennasMcUe = uintegerValue.Get();
   // GlobalValue::GetValueByName ("N_AntennasMmWave", uintegerValue);
   // int numAntennasMmWave = uintegerValue.Get();
 
   Config::SetDefault("ns3::McUeNetDevice::AntennaNum", UintegerValue(numAntennasMcUe));
   Config::SetDefault("ns3::MmWaveNetDevice::AntennaNum", UintegerValue(16)); // Increase antennas
   Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(bandwidth));
   Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(centerFrequency));
 
   Ptr <MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
   
   if (useHybrid)
   {
       BooleanValue enableTiltTwoRayValue;
       GlobalValue::GetValueByName("enableTiltTwoRay", enableTiltTwoRayValue);
       bool enableTiltTwoRay = enableTiltTwoRayValue.Get();
       
       if (enableTiltTwoRay)
       {
           NS_LOG_UNCOND("Using Hybrid Propagation Model (LogDistance + Random Shadowing + TwoRay for E-Tilt)");
           mmwaveHelper->SetPathlossModelType("ns3::LogDistancePropagationLossModel");
           
           // Configure LogDistance attributes (unchanged - TxPower still works via this)
           Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.8));
           Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(43.3));
           
           // Enable TwoRaySpectrumPropagationLossModel for phased array antenna gain (includes E-Tilt)
           // Set ChannelConditionModelType (mmwaveHelper will create it, but we still need to set it on TwoRay)
           mmwaveHelper->SetChannelConditionModelType("ns3::AlwaysLosChannelConditionModel");
           
           // Set model type FIRST, then attributes
           mmwaveHelper->SetChannelModelType("ns3::TwoRaySpectrumPropagationLossModel");
           
           // Set attributes on TwoRay model
           // Frequency: use centerFrequency from scenario (will be set by mmwaveHelper later, but we set a default)
           // Note: mmwaveHelper will override this with phyMacCommon->GetCenterFrequency() during initialization
           mmwaveHelper->SetChannelModelAttribute("Frequency", DoubleValue(centerFrequency));
           // Scenario: UMi-StreetCanyon (common urban scenario, pre-calibrated)
           mmwaveHelper->SetChannelModelAttribute("Scenario", StringValue("UMi-StreetCanyon"));
           
           // ChannelConditionModel: Create and set via factory attribute (smart pointer will keep it alive)
           // Note: mmwaveHelper's else branch doesn't automatically associate ChannelConditionModel to spectrum model
           // like it does for ThreeGpp, so we need to set it explicitly
           Ptr<ChannelConditionModel> losModel = CreateObject<AlwaysLosChannelConditionModel>();
           mmwaveHelper->SetChannelModelAttribute("ChannelConditionModel", PointerValue(losModel));
       }
       else
       {
           NS_LOG_UNCOND("Using Hybrid Propagation Model (LogDistance + Random Shadowing)");
           mmwaveHelper->SetPathlossModelType("ns3::LogDistancePropagationLossModel");
           mmwaveHelper->SetChannelModelType(""); // Disable heavy 3GPP spectrum model
           // Configure LogDistance attributes
           Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.8));
           Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(43.3));
       }
   }
   else if (useFriis)
   {
       NS_LOG_UNCOND("Using Friis Propagation Model");
       mmwaveHelper->SetPathlossModelType("ns3::FriisPropagationLossModel");
       mmwaveHelper->SetChannelModelType(""); // Disable spectrum model
   }
   else
   {
       NS_LOG_UNCOND("Using 3GPP UMi Street Canyon Propagation Model");
       mmwaveHelper->SetChannelConditionModelType("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");
   } 
   
   mmwaveHelper->SetBeamformingModelType("ns3::MmWaveDftBeamforming");
 
   mmwaveHelper->SetEnbPhasedArrayModelAttribute("NumRows", UintegerValue(16));
   mmwaveHelper->SetEnbPhasedArrayModelAttribute("NumColumns", UintegerValue(4));
   
   double tiltRadians = tilt * M_PI / 180.0;
   mmwaveHelper->SetEnbPhasedArrayModelAttribute("DowntiltAngle", DoubleValue(tiltRadians));
   
   mmwaveHelper->SetUePhasedArrayModelAttribute("NumRows", UintegerValue(2));
   mmwaveHelper->SetUePhasedArrayModelAttribute("NumColumns", UintegerValue(2));
 
   Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(100e6));
 
   Ptr <MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper>();
   mmwaveHelper->SetEpcHelper(epcHelper);
 
   // We enforce 2 nodes for this scenario for simplicity
   uint8_t nMmWaveEnbNodes = 2; 
 
   GlobalValue::GetValueByName ("N_LteEnbNodes", uintegerValue);
   uint8_t nLteEnbNodes = uintegerValue.Get();
   GlobalValue::GetValueByName ("N_Ues", uintegerValue);
   uint32_t ues = uintegerValue.Get ();
   uint8_t nUeNodes = ues;
 
   Ptr <Node> pgw = epcHelper->GetPgwNode();
   NodeContainer remoteHostContainer;
   remoteHostContainer.Create(1);
   Ptr <Node> remoteHost = remoteHostContainer.Get(0);
   InternetStackHelper internet;
   internet.Install(remoteHostContainer);
 
   PointToPointHelper p2ph;
   p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
   p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
   p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
   NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
   Ipv4AddressHelper ipv4h;
   ipv4h.SetBase("1.0.0.0", "255.0.0.0");
   Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
   Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
   Ipv4StaticRoutingHelper ipv4RoutingHelper;
   Ptr <Ipv4StaticRouting> remoteHostStaticRouting =
       ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
   remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
 
   NodeContainer ueNodes;
   NodeContainer mmWaveEnbNodes;
   NodeContainer lteEnbNodes;
   
   // Create nodes
   mmWaveEnbNodes.Create(nMmWaveEnbNodes);
   lteEnbNodes.Create(nLteEnbNodes);
   ueNodes.Create(nUeNodes);
 
   // Split into individual containers for separate configuration
   NodeContainer mmWaveEnbNode1 = NodeContainer(mmWaveEnbNodes.Get(0));
   NodeContainer mmWaveEnbNode2 = NodeContainer(mmWaveEnbNodes.Get(1));
 
   NodeContainer allEnbNodes;
   allEnbNodes.Add(lteEnbNodes);
   allEnbNodes.Add(mmWaveEnbNodes);
 
   NodeContainerManager::GetInstance().SetMmWaveEnbNodes(mmWaveEnbNodes);
 
   Vector centerPosition = Vector(maxXAxis / 2, maxYAxis / 2, 3);
 
   // Base station height: 15.0m for Hybrid mode (realistic), 3.0m otherwise
   double enbHeight = useHybrid ? 5.0 : 3.0;
   Ptr <ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
   enbPositionAlloc->Add(Vector(centerPosition.x - isd_cell/2, centerPosition.y, enbHeight));
   enbPositionAlloc->Add(Vector(centerPosition.x + isd_cell/2, centerPosition.y, enbHeight));
 
   MobilityHelper enbmobility;
   enbmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
   enbmobility.SetPositionAllocator(enbPositionAlloc);
   enbmobility.Install(allEnbNodes);
 
   // Install Mobility for UEs (uniformly distributed over the full mobility area)
   Ptr<ListPositionAllocator> uePositionAllocNew = CreateObject<ListPositionAllocator>();
   Ptr<UniformRandomVariable> xPos = CreateObject<UniformRandomVariable>();
   // Match initial placement to mobility bounds in X: [500, 1500]
   xPos->SetAttribute("Min", DoubleValue(500.0));
   xPos->SetAttribute("Max", DoubleValue(1500.0));
   Ptr<UniformRandomVariable> yPos = CreateObject<UniformRandomVariable>();
   // Match initial placement to mobility bounds in Y: [800, 1200]
   yPos->SetAttribute("Min", DoubleValue(800.0));
   yPos->SetAttribute("Max", DoubleValue(1200.0));
 
   for (uint32_t i = 0; i < ues; i++) {
       uePositionAllocNew->Add(Vector(xPos->GetValue(), yPos->GetValue(), 1.5));
   }
 
   GlobalValue::GetValueByName("Mobility", booleanValue);
   bool mobilityEnabled = booleanValue.Get();
 
   MobilityHelper uemobility;
   if (!mobilityEnabled)
     {
       // Legacy behavior: all UEs static at their initial positions
       uemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
       uemobility.SetPositionAllocator(uePositionAllocNew);
       uemobility.Install(ueNodes);
     }
   else
     {
       // Heterogeneous mobility profile for ML data collection:
       // Group 1: fast cars 30 m/s (50%) - long-distance traversal, change direction every 10s
       // Group 2: static (20%) - anchor points for baseline metrics
       // Group 3: medium speed 15 m/s (30%) - local topology variations, change direction every 5s
       // Distribution: 50/20/30 split for maximum state diversity
 
       // Common bounds for all mobile groups (same as RandomWalk2d used before)
       Rectangle bounds (centerPosition.x - isd_cell,
                         centerPosition.x + isd_cell,
                         centerPosition.y - 200,
                         centerPosition.y + 200);
 
       // Helpers for each group, all sharing the same initial position allocator
       MobilityHelper mobFast;     // Group 1: 30 m/s
       MobilityHelper mobStatic;   // Group 2: 0 m/s
       MobilityHelper mobMedium;   // Group 3: 15 m/s
 
       mobFast.SetPositionAllocator (uePositionAllocNew);
       mobStatic.SetPositionAllocator (uePositionAllocNew);
       mobMedium.SetPositionAllocator (uePositionAllocNew);
 
       // Group 1: Fast cars, 30 m/s, change direction every 10 s
       // Long intervals allow UEs to traverse entire deployment area multiple times
       mobFast.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                 "Mode", StringValue ("Time"),
                                 "Bounds", RectangleValue (bounds),
                                 "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30.0]"),
                                 "Time", TimeValue (Seconds (10.0)));
 
       // Group 2: Static UEs (0 m/s) - anchor points
       mobStatic.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
 
       // Group 3: Medium speed, 15 m/s, change direction every 5 s
       // Aligned with action intervals for fresh topology at each decision point
       mobMedium.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                   "Mode", StringValue ("Time"),
                                   "Bounds", RectangleValue (bounds),
                                   "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=15.0]"),
                                   "Time", TimeValue (Seconds (5.0)));
 
       // Partition UE nodes into 3 groups: 50% fast, 20% static, 30% medium
       uint32_t totalUes = ueNodes.GetN ();
       
       // Calculate group sizes (50%, 20%, 30% split)
       uint32_t size1 = (uint32_t)(totalUes * 0.50);  // 50% fast (e.g., 30 UEs for 60 total)
       uint32_t size2 = (uint32_t)(totalUes * 0.20);  // 20% static (e.g., 12 UEs for 60 total)
       uint32_t size3 = totalUes - size1 - size2;     // 30% medium (remainder, e.g., 18 UEs)
 
       uint32_t g1End = size1;              // Fast group: [0, g1End)
       uint32_t g2End = g1End + size2;      // Static group: [g1End, g2End)
       uint32_t g3End = totalUes;           // Medium group: [g2End, g3End)
 
       NodeContainer group1;  // Fast group
       NodeContainer group2;  // Static group
       NodeContainer group3;  // Medium group
 
       for (uint32_t i = 0; i < totalUes; ++i)
         {
           Ptr<Node> ueNode = ueNodes.Get (i);
           if (i < g1End)
             {
               group1.Add (ueNode);  // First 50% -> Fast (30 m/s)
             }
           else if (i < g2End)
             {
               group2.Add (ueNode);  // Next 20% -> Static (0 m/s)
             }
           else
             {
               group3.Add (ueNode);  // Last 30% -> Medium (15 m/s)
             }
         }
 
       // Install mobility models on each group
       if (group1.GetN () > 0)
         {
           mobFast.Install (group1);    // Group 1: Fast (30 m/s, 10s intervals)
         }
       if (group2.GetN () > 0)
         {
           mobStatic.Install (group2);  // Group 2: Static (0 m/s)
         }
       if (group3.GetN () > 0)
         {
           mobMedium.Install (group3);  // Group 3: Medium (15 m/s, 5s intervals)
         }
     }
 
   NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice(lteEnbNodes);
 
   // *** Differential Power Installation ***
   
   // Install Cell 1
   Config::SetDefault("ns3::MmWaveEnbPhy::TxPower", DoubleValue(txPower1));
   NetDeviceContainer mmWaveEnbDev1 = mmwaveHelper->InstallEnbDevice(mmWaveEnbNode1);
 
   // Install Cell 2
   Config::SetDefault("ns3::MmWaveEnbPhy::TxPower", DoubleValue(txPower2));
   NetDeviceContainer mmWaveEnbDev2 = mmwaveHelper->InstallEnbDevice(mmWaveEnbNode2);
 
   // Merge Device Containers for tracking
   NetDeviceContainer mmWaveEnbDevs;
   mmWaveEnbDevs.Add(mmWaveEnbDev1);
   mmWaveEnbDevs.Add(mmWaveEnbDev2);
   
   // *** Store device pointers for runtime control ***
   Ptr<MmWaveEnbNetDevice> enbDev1 = mmWaveEnbDev1.Get(0)->GetObject<MmWaveEnbNetDevice>();
   Ptr<MmWaveEnbNetDevice> enbDev2 = mmWaveEnbDev2.Get(0)->GetObject<MmWaveEnbNetDevice>();
   
   if (!enbDev1 || !enbDev2)
   {
     NS_FATAL_ERROR("Failed to get MmWaveEnbNetDevice pointers for runtime control");
   }
   
  uint16_t cellId1 = enbDev1->GetCellId();
  uint16_t cellId2 = enbDev2->GetCellId();
  
  // *** Store globally for handover mechanism ***
  g_mmWaveEnbDevs = mmWaveEnbDevs;
  g_mmwaveHelper = mmwaveHelper;
  g_cellId1 = cellId1;
  g_cellId2 = cellId2;
  
  // Store BS positions globally
  Ptr<MobilityModel> enb1Mob = mmWaveEnbNode1.Get(0)->GetObject<MobilityModel>();
  Ptr<MobilityModel> enb2Mob = mmWaveEnbNode2.Get(0)->GetObject<MobilityModel>();
  if (enb1Mob && enb2Mob)
  {
    g_bs1Pos = enb1Mob->GetPosition();
    g_bs2Pos = enb2Mob->GetPosition();
  }
  
  // Initialize globals from command line args
  g_txPower[0] = txPower1;
  g_txPower[1] = txPower2;
  // Note: Both cells start with the same tilt value (from --Tilt flag)
  // Individual cell tilts can be changed later via runtime control
  g_currentTilt[0] = tilt;
  g_currentTilt[1] = tilt;
  
  // Log handover configuration
  NS_LOG_UNCOND("=== Handover Mechanism Enabled ===");
  NS_LOG_UNCOND("  A3 Offset Cell 1: " << g_a3Offset[0] << " dB, Cell 2: " << g_a3Offset[1] << " dB");
  NS_LOG_UNCOND("  Handover checking interval: 0.1 seconds");
  NS_LOG_UNCOND("  Cell " << cellId1 << " position: (" << g_bs1Pos.x << ", " << g_bs1Pos.y << ", " << g_bs1Pos.z << ")");
  NS_LOG_UNCOND("  Cell " << cellId2 << " position: (" << g_bs2Pos.x << ", " << g_bs2Pos.y << ", " << g_bs2Pos.z << ")");
  
  // Log action space configuration
   if (enableRuntimeControl)
   {
     NS_LOG_UNCOND("=== Runtime Control Enabled ===");
     NS_LOG_UNCOND("  Control file: runtime_control.txt (polling every " << controlPollIntervalMs << "ms simulation time)");
     NS_LOG_UNCOND("  TxPower action space: [" << txPowerMin << ", " << txPowerMax << "] dBm");
     NS_LOG_UNCOND("  E-Tilt action space: [" << tiltMin << ", " << tiltMax << "] degrees");
     NS_LOG_UNCOND("  Cell " << cellId1 << " -> enbDev1");
     NS_LOG_UNCOND("  Cell " << cellId2 << " -> enbDev2");
     NS_LOG_UNCOND("  Command format: POWER <cellId> <value>  or  TILT <cellId> <value>");
   }
 
   // *** Hybrid Mode: Chain RandomPropagationLossModel for shadowing ***
   if (useHybrid)
   {
       NS_LOG_UNCOND("Chaining RandomPropagationLossModel for stochastic shadowing");
       // Create NormalRandomVariable for shadowing (Mean=0, Variance=49.0 -> StdDev=7.0 dB)
       Ptr<NormalRandomVariable> shadowingVar = CreateObject<NormalRandomVariable>();
       shadowingVar->SetAttribute("Mean", DoubleValue(0.0));
       shadowingVar->SetAttribute("Variance", DoubleValue(49.0));
       
       // Get the pathloss model for each component carrier (typically index 0)
       Ptr<PropagationLossModel> baseModel = mmwaveHelper->GetPathLossModel(0);
       if (baseModel)
       {
           // Create and configure RandomPropagationLossModel
           Ptr<RandomPropagationLossModel> shadowModel = CreateObject<RandomPropagationLossModel>();
           shadowModel->SetAttribute("Variable", PointerValue(shadowingVar));
           
           // Chain the shadowing model after the LogDistance model
           baseModel->SetNext(shadowModel);
           NS_LOG_UNCOND("Successfully chained RandomPropagationLossModel to LogDistancePropagationLossModel");
       }
       else
       {
           NS_LOG_ERROR("Failed to retrieve pathloss model for chaining - Hybrid mode may not work correctly");
       }
   }
 
   // Install UEs
   NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice(ueNodes);
  
  // *** Store UE devices globally for handover mechanism ***
  g_ueDevs = ueDevs;
 
   internet.Install(ueNodes);
   Ipv4InterfaceContainer ueIpIface;
   ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
   for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
       Ptr <Node> ueNode = ueNodes.Get(u);
       Ptr <Ipv4StaticRouting> ueStaticRouting =
           ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
       ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
     }
 
   // *** Setup X2 interfaces between mmWave eNBs for Standalone (SA) mode handover ***
   // This enables direct mmWave-to-mmWave SINR exchange and handover decisions
   mmwaveHelper->AddX2Interface(mmWaveEnbNodes);
   NS_LOG_UNCOND("X2 interfaces established between mmWave eNBs for SA mode handover");
 
   // Calculate Distance Helper
   auto CalculateDistance = [](Vector a, Vector b) {
       return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
   };
 
   // Custom Attachment: Attach to Best Server (Highest Estimated RxPower)
   // Approximated as: RxPower_dB = TxPower_dBm - 20*log10(Distance)
   for (uint32_t u = 0; u < ueDevs.GetN(); ++u) {
       Ptr<NetDevice> ueDev = ueDevs.Get(u);
       Ptr<Node> ueNode = ueNodes.Get(u);
       Vector uePos = ueNode->GetObject<MobilityModel>()->GetPosition();
 
       // Check Cell 1
       Ptr<Node> enb1Node = mmWaveEnbNode1.Get(0);
       Vector enb1Pos = enb1Node->GetObject<MobilityModel>()->GetPosition();
       double dist1 = CalculateDistance(uePos, enb1Pos);
       // Protect against dist=0
       double pathloss1 = (dist1 > 0) ? 20 * std::log10(dist1) : 0;
       double estRxPower1 = txPower1 - pathloss1;
 
       // Check Cell 2
       Ptr<Node> enb2Node = mmWaveEnbNode2.Get(0);
       Vector enb2Pos = enb2Node->GetObject<MobilityModel>()->GetPosition();
       double dist2 = CalculateDistance(uePos, enb2Pos);
       double pathloss2 = (dist2 > 0) ? 20 * std::log10(dist2) : 0;
       double estRxPower2 = txPower2 - pathloss2;
 
       if (estRxPower1 >= estRxPower2) {
           mmwaveHelper->AttachToEnbWithIndex(ueDev, mmWaveEnbDevs, 0);
       } else {
           mmwaveHelper->AttachToEnbWithIndex(ueDev, mmWaveEnbDevs, 1);
       }
   }
 
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  // Energy monitoring: only enable if explicitly requested (saves ~4.8 GB per 600s scenario)
  if (enableEnergyMonitoring)
    {
     BasicEnergySourceHelper basicEnergySourceHelper;
     basicEnergySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (1000000000000));
     basicEnergySourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (5.0));
     energy::EnergySourceContainer sources = basicEnergySourceHelper.Install (mmWaveEnbNodes);
     MmWaveRadioEnergyModelEnbHelper nrEnbHelper;
     energy::DeviceEnergyModelContainer deviceEModel = nrEnbHelper.Install (mmWaveEnbDevs, sources);
   
     int numPrints = simTime / 0.1;
   
     for (int x = 0; x < nMmWaveEnbNodes; ++x)
       {
         std::ostringstream filename;
         filename << "energyfilecell" << x + 2 << ".csv";
         deviceEModel.Get (x)->TraceConnectWithoutContext (
             "TotalEnergyConsumption",
             MakeBoundCallback (&EnergyConsumptionUpdate, x, filename.str ()));
         for (int i = 0; i < numPrints; i++)
           {
             Simulator::Schedule (Seconds (i * simTime / numPrints), &EnergyConsumptionPrint, x);
           }
       }
    }
  else
    {
      NS_LOG_UNCOND("Energy monitoring DISABLED - skipping energyfilecell*.csv generation (saves ~4.8 GB per 600s scenario)");
    }
 
   uint16_t portUdp = 60000;
   Address sinkLocalAddressUdp(InetSocketAddress(Ipv4Address::GetAny(), portUdp));
   PacketSinkHelper sinkHelperUdp("ns3::UdpSocketFactory", sinkLocalAddressUdp);
   AddressValue serverAddressUdp(InetSocketAddress(remoteHostAddr, portUdp));
 
   ApplicationContainer sinkApp;
   sinkApp.Add(sinkHelperUdp.Install(remoteHost));
 
   ApplicationContainer clientApp;
 
   for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
       PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), 1234));
       sinkApp.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));
       UdpClientHelper dlClient(ueIpIface.GetAddress(u), 1234);
       dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(1639)));
       dlClient.SetAttribute("MaxPackets", UintegerValue(UINT32_MAX));
       dlClient.SetAttribute("PacketSize", UintegerValue(1024)); 
       clientApp.Add(dlClient.Install(remoteHost));
     }
 
   sinkApp.Start (Seconds (0));
   clientApp.Start(MilliSeconds(100));
   clientApp.Stop(Seconds(simTime - 0.1));
 
  if (enableTraces)
  {
    mmwaveHelper->EnableRlcTraces();      
    mmwaveHelper->EnablePdcpTraces();     
    mmwaveHelper->EnableEnbSchedTrace();  
  }
  
  // CRITICAL: EnableDlPhyTrace() MUST be called unconditionally to connect RxPacketTraceUeCallback
  // This callback updates MAC statistics via UpdateTraces(), which is required for du-cell outputs
  // Without this, MAC stats remain zero even though packets are transmitted (SINR data exists)
  // This is separate from other traces (RLC/PDCP) and is needed for E2 reporting
  mmwaveHelper->EnableDlPhyTrace();
 
   // Setup UE position export if enabled
   if (exportUEPositions)
   {
     // Open file in build directory
     std::string positionFile = "UEPosition.txt";
     g_uePositionFile.open(positionFile, std::ios::out);
     
     if (g_uePositionFile.is_open())
     {
       g_exportUEPositionsEnabled = true;
       g_uePositionFile << "Time(s),Type,ID,X(m),Y(m),Z(m),CellID" << std::endl;
       NS_LOG_UNCOND("UE position export enabled. Writing to: " << positionFile);
       
       // Start logging from t=0.1s (first frame after initial setup)
       Simulator::Schedule(Seconds(0.1), &LogUEPositions, 
                           ueNodes, ueDevs, mmWaveEnbNodes, mmWaveEnbDevs);
       
       // Close file at end of simulation
       Simulator::ScheduleDestroy([]() { 
         if (g_uePositionFile.is_open()) {
    g_uePositionFile.close();
           NS_LOG_UNCOND("UE position trace file closed.");
         }
       });
     }
     else
     {
       NS_LOG_ERROR("Failed to open UE position file: " << positionFile);
     }
   }
 
   // Setup Network Configuration export
   std::string configFile = "NetworkConfigurations.txt";
   g_networkConfigFile.open(configFile, std::ios::out);
   if (g_networkConfigFile.is_open())
   {
     g_networkConfigFile << "Time(s),Cell1_TxPower,Cell1_Tilt,Cell1_A3,Cell2_TxPower,Cell2_Tilt,Cell2_A3" << std::endl;
     NS_LOG_UNCOND("Network configuration export enabled. Writing to: " << configFile);
     
     // Start logging from t=0.1s
     Simulator::Schedule(Seconds(0.1), &LogNetworkConfigurations);
     
     // Close file at end of simulation
     Simulator::ScheduleDestroy([]() { 
       if (g_networkConfigFile.is_open()) {
         g_networkConfigFile.close();
         NS_LOG_UNCOND("Network configuration trace file closed.");
       }
     });
   }
   else
   {
     NS_LOG_ERROR("Failed to open Network Configuration file: " << configFile);
   }
 
  // *** Start external runtime control file polling (if enabled) ***
  if (enableRuntimeControl)
  {
    Simulator::Schedule(Seconds(0.1), &CheckControlFile, enbDev1, enbDev2,
                        txPowerMin, txPowerMax, tiltMin, tiltMax, a3Min, a3Max, controlPollIntervalMs);
    NS_LOG_UNCOND("Runtime control polling started. Waiting for commands in runtime_control.txt");
  }
  
  // *** Start handover checking mechanism ***
  // This runs independently of runtime control and checks handover conditions every 0.1s
  Simulator::Schedule(Seconds(0.1), &CheckHandover);
  NS_LOG_UNCOND("Handover checking mechanism started. Monitoring RSRP every 0.1 seconds");
 
  // Open handover log file
  std::string handoverLogFile = "HandoverLog.txt";
  g_handoverLogFile.open(handoverLogFile, std::ios::out);
  if (g_handoverLogFile.is_open())
  {
    g_handoverLogFile << "Time(s),Type,IMSI,SourceCell,TargetCell,TargetRNTI" << std::endl;
    NS_LOG_UNCOND("Handover logging enabled. Writing to: " << handoverLogFile);
  }
  else
  {
    NS_LOG_ERROR("Failed to open handover log file: " << handoverLogFile);
  }

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  
  // Close handover log file
  if (g_handoverLogFile.is_open())
  {
    g_handoverLogFile.close();
    NS_LOG_UNCOND("Handover log file closed.");
  }
  
  Simulator::Destroy();
  return 0;
}
 