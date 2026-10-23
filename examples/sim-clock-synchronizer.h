// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef SIM_CLOCK_SYNCHRONIZER_H
#define SIM_CLOCK_SYNCHRONIZER_H

#include "ns3/synchronizer.h"
#include "ns3/vector.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace ns3
{

/**
 * @brief A Synchronizer that follows an external simulator's clock.
 *
 * With RealtimeSimulatorImpl (SynchronizerType set to this class), ns-3 then
 * runs on the clock of the world it emulates a network for, not on the wall
 * clock: a packet arriving from a TAP is stamped with the current external
 * simulation time, and a delay of d holds it until that time has advanced by
 * d. A world that runs slower than real time therefore no longer sees every
 * network delay stretched by its slowdown.
 *
 * The clock is fed over UDP (Port): each datagram carries one tick's
 * simulation time in nanoseconds, a little-endian uint64, optionally followed
 * by the radio bridge's positions: a little-endian uint32 count n and n
 * float32 triples x, y, z in metres (ns-3's frame; NaN for a UE whose vehicle
 * is absent), then n condition bytes (0 LOS, 1 NLOS, 2 NLOSv, 255 unknown),
 * kept for the scenario to apply (GetPositions). Between ticks the clock is interpolated at
 * the rate measured over the last ticks and never runs more than one step
 * past the last tick, so it stops when the world stops. It never runs
 * backwards, and a jump of more than ten steps (at least 1 s) is taken as a
 * restart of the external clock and absorbed. Before the first tick ns-3
 * waits.
 */
class SimClockSynchronizer : public Synchronizer
{
  public:
    static TypeId GetTypeId();

    SimClockSynchronizer();
    ~SimClockSynchronizer() override;

    /// Simulation time (ns) of the last tick received, relative to the first.
    uint64_t GetLastTick() const;

    /// Ticks received so far.
    uint64_t GetTickCount() const;

    /**
     * The positions of the last tick that carried them.
     *
     * @param [out] out One entry per UE; NaN where the vehicle was absent.
     * @return The number of ticks received when they arrived (0 for none),
     *         so that a caller can tell whether they changed.
     */
    uint64_t GetPositions(std::vector<Vector>& out) const;

    /**
     * The link conditions of the last tick that carried them.
     *
     * @param [out] out One byte per UE: 0 LOS, 1 NLOS, 2 NLOSv, 255 unknown.
     * @return As for GetPositions.
     */
    uint64_t GetConditions(std::vector<uint8_t>& out) const;

  protected:
    bool DoRealtime() override;
    uint64_t DoGetCurrentRealtime() override;
    void DoSetOrigin(uint64_t ns) override;
    int64_t DoGetDrift(uint64_t ns) override;
    bool DoSynchronize(uint64_t nsCurrent, uint64_t nsDelay) override;
    void DoSignal() override;
    void DoSetCondition(bool cond) override;
    void DoEventStart() override;
    uint64_t DoEventEnd() override;
    void DoDispose() override;

  private:
    using Clock = std::chrono::steady_clock;

    void Listen();
    /// The external clock relative to the origin, in ns; caller holds m_mutex.
    uint64_t NowLocked();

    uint16_t m_port;
    int m_socket;
    std::thread m_thread;
    bool m_stop;

    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_condition;

    bool m_haveTick;
    uint64_t m_firstTick;     ///< external time of the first tick (ns)
    uint64_t m_tick;          ///< last tick, relative to the first (ns)
    Clock::time_point m_tickWall;
    double m_rate;            ///< external ns per wall ns, measured
    uint64_t m_step;          ///< last tick interval in external ns
    uint64_t m_ticks;
    uint64_t m_jumps;         ///< discontinuities absorbed
    std::vector<Vector> m_positions;
    uint64_t m_positionsTick; ///< m_ticks when m_positions arrived
    std::vector<uint8_t> m_conditions;
    uint64_t m_conditionsTick;

    bool m_originPending;     ///< origin set before the first tick
    uint64_t m_origin;        ///< clock value at the origin (ns)
    uint64_t m_last;          ///< last value returned, to stay monotonic
    uint64_t m_eventStart;
};

} // namespace ns3

#endif // SIM_CLOCK_SYNCHRONIZER_H
