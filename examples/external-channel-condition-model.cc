// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#include "external-channel-condition-model.h"

#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ExternalChannelConditionModel");
NS_OBJECT_ENSURE_REGISTERED(ExternalChannelConditionModel);

TypeId
ExternalChannelConditionModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ExternalChannelConditionModel")
            .SetParent<ChannelConditionModel>()
            .SetGroupName("Nr")
            .AddConstructor<ExternalChannelConditionModel>()
            .AddAttribute("Default",
                          "Condition of a link that has none from outside yet",
                          EnumValue(ChannelCondition::LOS),
                          MakeEnumAccessor<ChannelCondition::LosConditionValue>(
                              &ExternalChannelConditionModel::m_default),
                          MakeEnumChecker(ChannelCondition::LOS, "LOS", ChannelCondition::NLOS, "NLOS"))
            .AddAttribute("VehicleBlockage",
                          "Condition reported for a link blocked by a vehicle (NLOSv)",
                          EnumValue(ChannelCondition::LOS),
                          MakeEnumAccessor<ChannelCondition::LosConditionValue>(
                              &ExternalChannelConditionModel::m_vehicle),
                          MakeEnumChecker(ChannelCondition::LOS, "LOS", ChannelCondition::NLOS, "NLOS"));
    return tid;
}

ExternalChannelConditionModel::ExternalChannelConditionModel()
    : m_default(ChannelCondition::LOS),
      m_vehicle(ChannelCondition::LOS)
{
}

ExternalChannelConditionModel::~ExternalChannelConditionModel() = default;

std::map<uint32_t, ChannelCondition::LosConditionValue>&
ExternalChannelConditionModel::Table()
{
    static std::map<uint32_t, ChannelCondition::LosConditionValue> table;
    return table;
}

void
ExternalChannelConditionModel::SetCondition(uint32_t ueNodeId, ChannelCondition::LosConditionValue c)
{
    Table()[ueNodeId] = c;
}

Ptr<ChannelCondition>
ExternalChannelConditionModel::GetChannelCondition(Ptr<const MobilityModel> a,
                                                   Ptr<const MobilityModel> b) const
{
    ChannelCondition::LosConditionValue c = m_default;
    for (auto m : {a, b})
    {
        Ptr<Node> n = m->GetObject<Node>();
        if (!n)
        {
            continue;
        }
        auto it = Table().find(n->GetId());
        if (it != Table().end())
        {
            c = it->second == ChannelCondition::NLOSv ? m_vehicle : it->second;
            break;
        }
    }
    return CreateObject<ChannelCondition>(c, ChannelCondition::O2O);
}

int64_t
ExternalChannelConditionModel::AssignStreams(int64_t /* stream */)
{
    return 0;
}

} // namespace ns3
