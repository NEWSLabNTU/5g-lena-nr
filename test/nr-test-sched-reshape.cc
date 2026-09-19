// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-mac-scheduler-ue-info.h"
#include "ns3/test.h"

#include <algorithm>

/**
 * @file nr-test-sched-reshape.cc
 * @ingroup test
 *
 * @brief Reshaping a HARQ retransmission must always terminate.
 */
namespace ns3
{

/// Minimal SAP user: the reshape path needs the slot geometry and nothing else.
class ReshapeSchedSapUser : public NrMacSchedSapUser
{
  public:
    void SchedConfigInd(const struct SchedConfigIndParameters&) override
    {
    }

    Ptr<const SpectrumModel> GetSpectrumModel() const override
    {
        return nullptr;
    }

    uint32_t GetNumRbPerRbg() const override
    {
        return 1;
    }

    uint8_t GetNumHarqProcess() const override
    {
        return 20;
    }

    uint16_t GetBwpId() const override
    {
        return 0;
    }

    uint16_t GetCellId() const override
    {
        return 0;
    }

    uint32_t GetSymbolsPerSlot() const override
    {
        return 14;
    }

    Time GetSlotPeriod() const override
    {
        return MilliSeconds(1);
    }

    void BuildRarList(SlotAllocInfo&) override
    {
    }
};

/// Minimal control SAP user: every confirmation is ignored.
class ReshapeCschedSapUser : public NrMacCschedSapUser
{
  public:
    void CschedCellConfigCnf(const struct CschedCellConfigCnfParameters&) override
    {
    }

    void CschedUeConfigCnf(const struct CschedUeConfigCnfParameters&) override
    {
    }

    void CschedLcConfigCnf(const struct CschedLcConfigCnfParameters&) override
    {
    }

    void CschedLcReleaseCnf(const struct CschedLcReleaseCnfParameters&) override
    {
    }

    void CschedUeReleaseCnf(const struct CschedUeReleaseCnfParameters&) override
    {
    }

    void CschedUeConfigUpdateInd(const struct CschedUeConfigUpdateIndParameters&) override
    {
    }

    void CschedCellConfigUpdateInd(const struct CschedCellConfigUpdateIndParameters&) override
    {
    }
};

/// Exposes the protected reshape trampoline so a test can drive it directly.
class ReshapeAccessor : public NrMacSchedulerTdmaRR
{
  public:
    std::vector<DciInfoElementTdma> Reshape(const std::vector<DciInfoElementTdma>& dcis,
                                            uint8_t& startingSymbol,
                                            uint8_t& numSymbols,
                                            std::vector<bool>& bitmask)
    {
        return ReshapeAllocation(dcis, startingSymbol, numSymbols, bitmask, true);
    }
};

/**
 * @brief Unusable retransmission allocations must be declined without changing slot resources.
 *
 * The grant being retransmitted spans 137 RBGs over 14 symbols, so it holds 1918 resources.
 * Only 13 RBGs are still free, which puts the smallest usable symbol count at
 * ceil(1918/13) = 148 -- far beyond the 14 symbols a slot has. No rectangle preserves the
 * resource count, so the process has to be left for a later slot.
 *
 * Reaching that conclusion by raising a symbol count until the two resource counts agree does
 * not work here: with seven resource block groups left, the uint8_t counter wraps at 256
 * before the resource count reaches 1918, so the search never ends. Additional cases cover
 * an empty resource mask and grants with zero symbols or zero resource block groups.
 */
class ReshapeTerminatesTestCase : public TestCase
{
  public:
    /**
     * @param name Test case description.
     * @param dciRbgs Resource block groups in the original grant.
     * @param freeRbgs Available resource block groups.
     * @param dciSymbols Symbols in the original grant.
     */
    ReshapeTerminatesTestCase(const std::string& name,
                              std::size_t dciRbgs,
                              std::size_t freeRbgs,
                              uint8_t dciSymbols)
        : TestCase(name),
          m_dciRbgs(dciRbgs),
          m_freeRbgs(freeRbgs),
          m_dciSymbols(dciSymbols)
    {
    }

  private:
    void DoRun() override
    {
        constexpr uint16_t rnti = 1;
        constexpr std::size_t bwpRbgs = 138;

        auto sched = CreateObject<ReshapeAccessor>();
        sched->SetMacSchedSapUser(&m_schedSapUser);
        sched->SetMacCschedSapUser(&m_cschedSapUser);

        NrMacCschedSapProvider::CschedUeConfigReqParameters ueParams{};
        ueParams.m_rnti = rnti;
        ueParams.m_beamId = BeamId(0, 0.0);
        sched->DoCschedUeConfigReq(ueParams);

        // Build the grant being retransmitted.
        std::vector<bool> dciBitmask(bwpRbgs, false);
        std::fill_n(dciBitmask.begin(), m_dciRbgs, true);
        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::DL,
                               0,            // symStart
                               m_dciSymbols, // numSym
                               0,            // mcs
                               1,            // rank
                               nullptr,
                               100, // tbs
                               0,   // ndi: retransmission
                               1,   // rv
                               DciInfoElementTdma::DATA,
                               0,  // bwpIndex
                               0); // tpc
        dci.m_rbgBitmask = dciBitmask;

        // What is still free when the retransmission is scheduled.
        std::vector<bool> bitmask(bwpRbgs, false);
        std::fill_n(bitmask.begin(), m_freeRbgs, true);

        const auto originalBitmask = bitmask;
        uint8_t startingSymbol = 0;
        uint8_t numSymbols = 14;
        const auto reshaped = sched->Reshape({dci}, startingSymbol, numSymbols, bitmask);

        NS_TEST_EXPECT_MSG_EQ(reshaped.empty(),
                              true,
                              "An unusable retransmission allocation must be declined");
        NS_TEST_EXPECT_MSG_EQ(+startingSymbol, 0, "A declined reshape must not advance the start");
        NS_TEST_EXPECT_MSG_EQ((bitmask == originalBitmask),
                              true,
                              "A declined reshape must not change available resource block groups");
        NS_TEST_EXPECT_MSG_EQ(+numSymbols, 14, "A declined reshape must not consume symbols");
    }

    const std::size_t m_dciRbgs;  ///< Resource block groups in the original grant.
    const std::size_t m_freeRbgs; ///< Available resource block groups.
    const uint8_t m_dciSymbols;   ///< Symbols in the original grant.
    ReshapeSchedSapUser m_schedSapUser;
    ReshapeCschedSapUser m_cschedSapUser;
};

class NrSchedReshapeTestSuite : public TestSuite
{
  public:
    NrSchedReshapeTestSuite()
        : TestSuite("nr-test-sched-reshape", Type::UNIT)
    {
        AddTestCase(new ReshapeTerminatesTestCase(
                        "Reshaping declines a retransmission that cannot be refitted",
                        137,
                        13,
                        14),
                    Duration::QUICK);
        AddTestCase(
            new ReshapeTerminatesTestCase(
                "Reshaping declines a retransmission with no available resource block groups",
                137,
                0,
                14),
            Duration::QUICK);
        AddTestCase(
            new ReshapeTerminatesTestCase("Reshaping declines a retransmission with zero symbols",
                                          137,
                                          13,
                                          0),
            Duration::QUICK);
        AddTestCase(new ReshapeTerminatesTestCase(
                        "Reshaping declines a retransmission with zero resource block groups",
                        0,
                        13,
                        14),
                    Duration::QUICK);
    }
};

static NrSchedReshapeTestSuite g_nrSchedReshapeTestSuite;

} // namespace ns3
