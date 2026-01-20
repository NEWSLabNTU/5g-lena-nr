// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/spectrum-transmit-filter.h"
#pragma once

namespace ns3
{

class NrLinkFilter : public SpectrumTransmitFilter
{
  public:
    NrLinkFilter();

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief Ignore the received signal if both the transmitter and receiver
     * belong to a GNB-GNB or UE-UE link.
     *
     * @param params The parameters of the received signal.
     * @param receiverPhy The SpectrumPhy of the receiver.
     * @return Whether the signal should be ignored (filtered out).
     *
     * @note This filter should ONLY be used with a TDD pattern composed of pure DL
     * and UL slots, where the ignored signal does not represent interference.
     */
    bool DoFilter(Ptr<const SpectrumSignalParameters> params,
                  Ptr<const SpectrumPhy> receiverPhy) override;

  protected:
    int64_t DoAssignStreams(int64_t stream) override;
};

} // namespace ns3
