// Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-link-filter.h"

#include "nr-spectrum-phy.h"
#include "nr-ue-net-device.h"

#include "ns3/boolean.h"
#include "ns3/spectrum-transmit-filter.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrLinkFilter");

NS_OBJECT_ENSURE_REGISTERED(NrLinkFilter);

NrLinkFilter::NrLinkFilter()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrLinkFilter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrLinkFilter")
                            .SetParent<SpectrumTransmitFilter>()
                            .AddConstructor<NrLinkFilter>();
    return tid;
}

bool
NrLinkFilter::DoFilter(Ptr<const SpectrumSignalParameters> params,
                       Ptr<const SpectrumPhy> receiverPhy)
{
    bool txIsGnb = DynamicCast<NrSpectrumPhy>(params->txPhy)->IsGnb();
    bool rxIsGnb = DynamicCast<const NrSpectrumPhy>(receiverPhy)->IsGnb();

    // Allow only UE-to-gNB or gNB-to-UE (different roles)
    return (txIsGnb == rxIsGnb);
}

int64_t
NrLinkFilter::DoAssignStreams(int64_t stream)
{
    return 0;
}

} // namespace ns3
