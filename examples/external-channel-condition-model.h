// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef EXTERNAL_CHANNEL_CONDITION_MODEL_H
#define EXTERNAL_CHANNEL_CONDITION_MODEL_H

#include "ns3/channel-condition-model.h"

#include <map>

namespace ns3
{

/**
 * @brief A channel condition set from outside, per UE.
 *
 * The radio bridge casts a ray from each UE to the gNB in the driving
 * simulator's world and sends the link's condition with each tick (phase 022
 * R4); the scenario passes it on with SetCondition. A link is identified by
 * the node of its UE, so this suits one gNB with many UEs. A link without a
 * condition yet gets the Default attribute. NLOSv (a vehicle in the way) is
 * reported as VehicleBlockage, since the 3GPP UMa/UMi/RMa path loss models
 * do not take NLOSv.
 */
class ExternalChannelConditionModel : public ChannelConditionModel
{
  public:
    static TypeId GetTypeId();

    ExternalChannelConditionModel();
    ~ExternalChannelConditionModel() override;

    /// Set the condition of the link of the UE on node ueNodeId.
    static void SetCondition(uint32_t ueNodeId, ChannelCondition::LosConditionValue c);

    /// The condition last set for the UE on node ueNodeId, as reported (0 LOS,
    /// 1 NLOS, 2 NLOSv), or -1 if none was set yet.
    static int GetCondition(uint32_t ueNodeId);

    Ptr<ChannelCondition> GetChannelCondition(Ptr<const MobilityModel> a,
                                              Ptr<const MobilityModel> b) const override;
    int64_t AssignStreams(int64_t stream) override;

  private:
    static std::map<uint32_t, ChannelCondition::LosConditionValue>& Table();

    ChannelCondition::LosConditionValue m_default;
    ChannelCondition::LosConditionValue m_vehicle;
};

} // namespace ns3

#endif // EXTERNAL_CHANNEL_CONDITION_MODEL_H
