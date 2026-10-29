// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef SIM_CLOCK_SYNCHRONIZER_H
#define SIM_CLOCK_SYNCHRONIZER_H

#include "ns3/nstime.h"
#include "ns3/synchronizer.h"
#include "ns3/vector.h"

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <string>
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
 * backwards: a jump of more than ten steps (at least 1 s) either way is
 * taken as a restart of the external clock (a world that ran unwatched, or a
 * new episode whose clock starts again from zero) and absorbed as one step.
 * Before the first tick ns-3 waits.
 *
 * Lockstep (Gate set to the external simulator's tick gate, host:port): the
 * world then waits for ns-3 as well. A tick may end with the external frame
 * number (a little-endian uint64 after the condition bytes). ns-3 owns the
 * interval from that tick's time to one step later; once every event before
 * the interval's end has run and the clock has reached it, it sends
 * `DONE <frame>` to the gate, which holds the next tick until then. The
 * clock never leads the event being executed by more than Slack: an event
 * that takes long, or a backlog of events, pauses the clock instead of
 * making later events late. Rate fixes the pace within the interval
 * (external ns per wall ns); 0 keeps the measured rate.
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
    /// The pace within an interval: Rate, or the measured rate.
    double PaceLocked() const;
    /// Send one datagram of the tick-gate protocol, if a gate is set.
    void SendGate(const std::string& msg);
    /// The clock at the current interval's end; caller holds m_mutex.
    uint64_t IntervalEndLocked() const;
    /// Statistics line, every so many acknowledged frames; caller holds m_mutex.
    void ReportLocked();

    uint16_t m_port;
    std::string m_gate;       ///< tick gate host:port, empty for none
    double m_fixedRate;       ///< external ns per wall ns, 0 for measured
    Time m_slackTime;         ///< how far the clock may lead the executing event
    Time m_stepTime;          ///< the step assumed before one is measured
    std::string m_traceFile;  ///< per-frame CSV, empty for none
    int32_t m_channelLag;     ///< hold each frame until this many frames back are traced (-1 off)
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

    // Lockstep.
    bool m_gateOn;
    sockaddr_in m_gateAddr;
    Clock::time_point m_tickWallRaw; ///< arrival of the last tick, for the rate
    bool m_haveFrame;
    uint64_t m_frame;           ///< external frame of the last tick
    bool m_acked;               ///< the current interval was acknowledged
    Clock::time_point m_ackWall;
    bool m_idle;                ///< caught up and waiting for the next event
    uint64_t m_simPos;          ///< simulation time (ns) of the executing event
    // Statistics, reset at each report.
    uint64_t m_acks;
    double m_holdMs;            ///< wall time held at an interval's end
    uint64_t m_holdSignals;     ///< external events scheduled while held
    double m_pausedMs;          ///< wall time the clock was paused for ns-3
    uint64_t m_maxLateNs;       ///< largest lag of an event behind the clock
    uint64_t m_lateEvents;      ///< events more than 1 ms behind the clock
    uint64_t m_events;
    // Per-frame trace.
    std::ofstream m_trace;
    Clock::time_point m_wall0;
    double m_framePausedMs;     ///< clock paused during the current frame
    uint64_t m_frameMaxLateNs;  ///< largest lateness during the current frame
    double m_tickArrivalMs;     ///< wall time of the current frame's tick
};

} // namespace ns3

#endif // SIM_CLOCK_SYNCHRONIZER_H
