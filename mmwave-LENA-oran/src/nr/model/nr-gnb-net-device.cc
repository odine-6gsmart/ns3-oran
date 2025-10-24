// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-gnb-net-device.h"

#include "bandwidth-part-gnb.h"
#include "bwp-manager-gnb.h"
#include "nr-gnb-component-carrier-manager.h"
#include "nr-gnb-mac.h"
#include "nr-gnb-phy.h"
#include "nr-gnb-rrc.h"

#include <ns3/abort.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/log.h>
#include <ns3/object-map.h>
#include <ns3/pointer.h>
#include "encode_e2apv1.hpp"
#include <ns3/double.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbNetDevice");

NS_OBJECT_ENSURE_REGISTERED(NrGnbNetDevice);

TypeId
NrGnbNetDevice::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrGnbNetDevice")
            .SetParent<NrNetDevice>()
            .AddConstructor<NrGnbNetDevice>()
            .AddAttribute("NrGnbComponentCarrierManager",
                          "The component carrier manager associated to this GnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&NrGnbNetDevice::m_componentCarrierManager),
                          MakePointerChecker<NrGnbComponentCarrierManager>())
            .AddAttribute("BandwidthPartMap",
                          "List of Bandwidth Part container.",
                          ObjectMapValue(),
                          MakeObjectMapAccessor(&NrGnbNetDevice::m_ccMap),
                          MakeObjectMapChecker<BandwidthPartGnb>())
            .AddAttribute("NrGnbRrc",
                          "The RRC layer associated with the gNB",
                          PointerValue(),
                          MakePointerAccessor(&NrGnbNetDevice::m_rrc),
                          MakePointerChecker<NrGnbRrc>())
            .AddAttribute ("CellId",
                            "Cell Identifier",
                            UintegerValue (0),
                            MakeUintegerAccessor (&NrGnbNetDevice::m_cellId),
                            MakeUintegerChecker<uint16_t> ()) 
            .AddAttribute ("E2Termination",
                            "The E2 termination object associated to this node",
                            PointerValue (),
                            MakePointerAccessor (&NrGnbNetDevice::SetE2Termination,
                                                            &NrGnbNetDevice::GetE2Termination),
                            MakePointerChecker <E2Termination> ())  
            .AddAttribute ("EnableE2FileLogging",
                         "If true, force E2 indication generation and write E2 fields in csv file",
                         BooleanValue (false),
                         MakeBooleanAccessor (&NrGnbNetDevice::m_forceE2FileLogging),
                         MakeBooleanChecker ())
            .AddAttribute ("KPM_E2functionID", "Function ID to subscribe", DoubleValue (2),
                           MakeDoubleAccessor (&NrGnbNetDevice::e2_func_id),
                           MakeDoubleChecker<double> ())
            .AddAttribute("RC_E2functionID", "Function ID to subscribe", DoubleValue(3),
                           MakeDoubleAccessor(&NrGnbNetDevice::rc_e2_func_id),
                           MakeDoubleChecker<double>());               
    return tid;
}

NrGnbNetDevice::NrGnbNetDevice()
    : m_forceE2FileLogging (false),m_cellId(0),
     m_stopSendingMessages(false),
     m_isReportingEnabled (false)
{    
    NS_LOG_FUNCTION(this);
}

NrGnbNetDevice::~NrGnbNetDevice()
{
    NS_LOG_FUNCTION(this);
}

Ptr<NrMacScheduler>
NrGnbNetDevice::GetScheduler(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetScheduler();
}

void
NrGnbNetDevice::SetCcMap(const std::map<uint8_t, Ptr<BandwidthPartGnb>>& ccm)
{
    NS_ABORT_IF(!m_ccMap.empty());
    m_ccMap = ccm;
}

uint32_t
NrGnbNetDevice::GetCcMapSize() const
{
    return static_cast<uint32_t>(m_ccMap.size());
}


void NrGnbNetDevice::stopSendingAndCancelSchedule() {
    m_stopSendingMessages = true;
}


void
NrGnbNetDevice::KpmSubscriptionCallback (E2AP_PDU_t* sub_req_pdu)
{
  NS_LOG_DEBUG ("\nReceived RIC Subscription Request, cellId= " << m_cellId << "\n");

  E2Termination::RicSubscriptionRequest_rval_s params = m_e2term->ProcessRicSubscriptionRequest (sub_req_pdu);
  NS_LOG_DEBUG ("requestorId " << +params.requestorId <<
                 ", instanceId " << +params.instanceId <<
                 ", ranFuncionId " << +params.ranFuncionId <<
                 ", actionId " << +params.actionId);

  if (!m_stopSendingMessages && !m_isReportingEnabled && !m_forceE2FileLogging)
  {
    //BuildAndSendReportMessage (params);
    m_isReportingEnabled = true;
  }

}


void
    NrGnbNetDevice::ControlMessageReceivedCallback(E2AP_PDU_t *sub_req_pdu) {
        NS_LOG_DEBUG("\n\nLteEnbNetDevice::ControlMessageReceivedCallback: Received RIC Control Message");

        // Create RIC Control ACK
        Ptr <RicControlMessage> controlMessage = Create<RicControlMessage>(sub_req_pdu);
        NS_LOG_INFO("After RicControlMessage::RicControlMessage constructor");
        NS_LOG_INFO("Request type " << controlMessage->m_requestType);
        
    }
void
NrGnbNetDevice::SetE2Termination(Ptr<E2Termination> e2term)
{
  m_e2term = e2term;

  NS_LOG_DEBUG("Register E2SM NR");

  if (!m_forceE2FileLogging) {
       long m_e2_func_id = long (e2_func_id);
       long m_rc_e2_func_id = long(rc_e2_func_id);
      Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription> ();
      e2term->RegisterKpmCallbackToE2Sm (
              m_e2_func_id, kpmFd,std::bind (&NrGnbNetDevice::KpmSubscriptionCallback, this, std::placeholders::_1));

      Ptr <RicControlFunctionDescription> ricCtrlFd = Create<RicControlFunctionDescription>();
      e2term->RegisterSmCallbackToE2Sm(m_rc_e2_func_id, ricCtrlFd,
                                      std::bind(&NrGnbNetDevice::ControlMessageReceivedCallback,
                                                this, std::placeholders::_1));

      e2term->RegisterCallbackFunctionToE2Sm(1, std::bind(&NrGnbNetDevice::stopSendingAndCancelSchedule, this));
    }
}

Ptr<E2Termination>
NrGnbNetDevice::GetE2Termination() const
{
  return m_e2term;
}

void
NrGnbNetDevice::SetNrFhControl(Ptr<NrFhControl> nrFh)
{
    NS_LOG_FUNCTION(this);
    m_nrFhControl = nrFh;
}

Ptr<NrFhControl>
NrGnbNetDevice::GetNrFhControl()
{
    NS_LOG_FUNCTION(this);
    return m_nrFhControl;
}

void
NrGnbNetDevice::RouteIngoingCtrlMsgs(const std::list<Ptr<NrControlMessage>>& msgList,
                                     uint8_t sourceBwpId)
{
    NS_LOG_FUNCTION(this);

    for (const auto& msg : msgList)
    {
        uint8_t bwpId = DynamicCast<BwpManagerGnb>(m_componentCarrierManager)
                            ->RouteIngoingCtrlMsgs(msg, sourceBwpId);
        m_ccMap.at(bwpId)->GetPhy()->PhyCtrlMessagesReceived(msg);
    }
}

void
NrGnbNetDevice::RouteOutgoingCtrlMsgs(const std::list<Ptr<NrControlMessage>>& msgList,
                                      uint8_t sourceBwpId)
{
    NS_LOG_FUNCTION(this);

    for (const auto& msg : msgList)
    {
        uint8_t bwpId = DynamicCast<BwpManagerGnb>(m_componentCarrierManager)
                            ->RouteOutgoingCtrlMsg(msg, sourceBwpId);
        NS_ASSERT_MSG(m_ccMap.size() > bwpId,
                      "Returned bwp " << +bwpId << " is not present. Check your configuration");
        NS_ASSERT_MSG(
            m_ccMap.at(bwpId)->GetPhy()->HasDlSlot(),
            "Returned bwp "
                << +bwpId
                << " has no DL slot, so the message can't go out. Check your configuration");
        m_ccMap.at(bwpId)->GetPhy()->EncodeCtrlMsg(msg);
    }
}

void
NrGnbNetDevice::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    m_rrc->Initialize();

    NrNetDevice::DoInitialize();
}

void
NrGnbNetDevice::DoDispose()
{
    NS_LOG_FUNCTION(this);

    m_rrc->Dispose();
    m_rrc = nullptr;
    for (const auto& it : m_ccMap)
    {
        it.second->Dispose();
    }
    m_ccMap.clear();
    m_componentCarrierManager->Dispose();
    m_componentCarrierManager = nullptr;
    NrNetDevice::DoDispose();
}

Ptr<NrGnbMac>
NrGnbNetDevice::GetMac(uint8_t index) const
{
    return m_ccMap.at(index)->GetMac();
}

Ptr<NrGnbPhy>
NrGnbNetDevice::GetPhy(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetPhy();
}

Ptr<BwpManagerGnb>
NrGnbNetDevice::GetBwpManager() const
{
    return DynamicCast<BwpManagerGnb>(m_componentCarrierManager);
}

uint16_t
NrGnbNetDevice::GetCellId() const
{
    NS_LOG_FUNCTION(this);
    return m_cellId;
}

std::vector<uint16_t>
NrGnbNetDevice::GetCellIds() const
{
    std::vector<uint16_t> cellIds;

    cellIds.reserve(m_ccMap.size());
    for (auto& it : m_ccMap)
    {
        cellIds.push_back(it.second->GetCellId());
    }
    return cellIds;
}

void
NrGnbNetDevice::SetCellId(uint16_t cellId)
{
    NS_LOG_FUNCTION(this);
    m_cellId = cellId;
}

uint16_t
NrGnbNetDevice::GetBwpId(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetCellId();
}

uint16_t
NrGnbNetDevice::GetEarfcn(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetDlEarfcn(); // Ul or Dl doesn't matter, they are the same
}

void
NrGnbNetDevice::SetRrc(Ptr<NrGnbRrc> rrc)
{
    m_rrc = rrc;
}

Ptr<NrGnbRrc>
NrGnbNetDevice::GetRrc()
{
    return m_rrc;
}

bool
NrGnbNetDevice::DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);
    NS_ABORT_MSG_IF(protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                        protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                    "unsupported protocol " << protocolNumber
                                            << ", only IPv4 and IPv6 are supported");

    NS_LOG_INFO("Forward received packet to RRC Layer");
    m_txTrace(packet, dest);

    return m_rrc->SendData(packet);
}

void
NrGnbNetDevice::UpdateConfig()
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT(!m_ccMap.empty());

    std::map<uint8_t, Ptr<BandwidthPartGnb>> ccPhyConfMap;
    for (const auto& i : m_ccMap)
    {
        Ptr<BandwidthPartGnb> c = i.second;
        ccPhyConfMap.insert(std::pair<uint8_t, Ptr<BandwidthPartGnb>>(i.first, c));
    }
   
    m_rrc->ConfigureCell(ccPhyConfMap);
    if (m_e2term)
            {
              NS_LOG_DEBUG ("E2sim start in cell " << m_cellId << " force CSV logging "
                                                   << m_forceE2FileLogging);
              //
              if(!m_forceE2FileLogging) {
                  Simulator::Schedule (MicroSeconds (0), &E2Termination::Start, m_e2term);
                }
            }
}

uint16_t
NrGnbNetDevice::GetCellIdDlBandwidth(uint16_t cellId) const
{
    NS_ASSERT_MSG(m_rrc->HasCellId(cellId), "Unknown cellId");
    if (m_rrc->HasCellId(cellId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetCellId() == cellId)
            {
                return cc->GetDlBandwidth();
            }
        }
    }
    return 0;
}

uint16_t
NrGnbNetDevice::GetCellIdUlBandwidth(uint16_t cellId) const
{
    NS_ASSERT_MSG(m_rrc->HasCellId(cellId), "Unknown cellId");
    if (m_rrc->HasCellId(cellId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetCellId() == cellId)
            {
                return cc->GetUlBandwidth();
            }
        }
    }
    return 0;
}

uint32_t
NrGnbNetDevice::GetCellIdDlEarfcn(uint16_t cellId) const
{
    NS_ASSERT_MSG(m_rrc->HasCellId(cellId), "Unknown cellId");
    if (m_rrc->HasCellId(cellId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetCellId() == cellId)
            {
                return cc->GetDlEarfcn();
            }
        }
    }
    return 0;
}

uint32_t
NrGnbNetDevice::GetCellIdUlEarfcn(uint16_t cellId) const
{
    NS_ASSERT_MSG(m_rrc->HasCellId(cellId), "Unknown cellId");
    if (m_rrc->HasCellId(cellId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetCellId() == cellId)
            {
                return cc->GetUlEarfcn();
            }
        }
    }
    return 0;
}

} // namespace ns3
