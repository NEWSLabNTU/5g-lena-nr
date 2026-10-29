// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#include "sim-clock-synchronizer.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SimClockSynchronizer");
NS_OBJECT_ENSURE_REGISTERED(SimClockSynchronizer);

TypeId
SimClockSynchronizer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SimClockSynchronizer")
            .SetParent<Synchronizer>()
            .SetGroupName("Nr")
            .AddConstructor<SimClockSynchronizer>()
            .AddAttribute("Port",
                          "UDP port on which the external simulator's ticks arrive",
                          UintegerValue(5600),
                          MakeUintegerAccessor(&SimClockSynchronizer::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Gate",
                          "The external simulator's tick gate (host:port), to run in "
                          "lockstep with it; empty for none",
                          StringValue(""),
                          MakeStringAccessor(&SimClockSynchronizer::m_gate),
                          MakeStringChecker())
            .AddAttribute("Rate",
                          "Pace within an interval, external ns per wall ns; 0 for the "
                          "rate measured over the last ticks",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&SimClockSynchronizer::m_fixedRate),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("Slack",
                          "In lockstep, how far the clock may lead the event ns-3 executes",
                          TimeValue(MilliSeconds(5)),
                          MakeTimeAccessor(&SimClockSynchronizer::m_slackTime),
                          MakeTimeChecker())
            .AddAttribute("TraceFile",
                          "CSV with one line per external tick: frame, simulation time, wall "
                          "time of the tick, and of the previous frame's acknowledgement, the "
                          "clock's pause for ns-3 and the largest event lateness in that frame",
                          StringValue(""),
                          MakeStringAccessor(&SimClockSynchronizer::m_traceFile),
                          MakeStringChecker())
            .AddAttribute("Step",
                          "In lockstep, the step assumed until two ticks have measured one",
                          TimeValue(MilliSeconds(50)),
                          MakeTimeAccessor(&SimClockSynchronizer::m_stepTime),
                          MakeTimeChecker());
    return tid;
}

SimClockSynchronizer::SimClockSynchronizer()
    : m_port(5600),
      m_fixedRate(0.0),
      m_slackTime(MilliSeconds(5)),
      m_stepTime(MilliSeconds(50)),
      m_socket(-1),
      m_stop(false),
      m_condition(false),
      m_haveTick(false),
      m_firstTick(0),
      m_tick(0),
      m_rate(1.0),
      m_step(0),
      m_ticks(0),
      m_jumps(0),
      m_positionsTick(0),
      m_conditionsTick(0),
      m_originPending(true),
      m_origin(0),
      m_last(0),
      m_eventStart(0),
      m_gateOn(false),
      m_gateAddr{},
      m_haveFrame(false),
      m_frame(0),
      m_acked(false),
      m_idle(true),
      m_simPos(0),
      m_acks(0),
      m_holdMs(0),
      m_holdSignals(0),
      m_pausedMs(0),
      m_maxLateNs(0),
      m_lateEvents(0),
      m_events(0),
      m_wall0(Clock::now()),
      m_framePausedMs(0),
      m_frameMaxLateNs(0),
      m_tickArrivalMs(0)
{
}

SimClockSynchronizer::~SimClockSynchronizer()
{
    DoDispose();
}

void
SimClockSynchronizer::DoDispose()
{
    if (m_gateOn)
    {
        SendGate("BYE");
        m_gateOn = false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
    }
    if (m_socket >= 0)
    {
        shutdown(m_socket, SHUT_RDWR);
    }
    if (m_thread.joinable())
    {
        m_thread.join();
    }
    if (m_socket >= 0)
    {
        close(m_socket);
        m_socket = -1;
    }
    if (m_trace.is_open())
    {
        m_trace.close();
    }
    Synchronizer::DoDispose();
}

uint64_t
SimClockSynchronizer::GetLastTick() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tick;
}

uint64_t
SimClockSynchronizer::GetPositions(std::vector<Vector>& out) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    out = m_positions;
    return m_positionsTick;
}

uint64_t
SimClockSynchronizer::GetConditions(std::vector<uint8_t>& out) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    out = m_conditions;
    return m_conditionsTick;
}

uint64_t
SimClockSynchronizer::GetTickCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_ticks;
}

void
SimClockSynchronizer::SendGate(const std::string& msg)
{
    if (!m_gateOn || m_socket < 0)
    {
        return;
    }
    // From the tick port, so that the gate knows ns-3 by one address.
    sendto(m_socket,
           msg.data(),
           msg.size(),
           MSG_DONTWAIT,
           reinterpret_cast<const sockaddr*>(&m_gateAddr),
           sizeof(m_gateAddr));
}

double
SimClockSynchronizer::PaceLocked() const
{
    return m_fixedRate > 0 ? m_fixedRate : m_rate;
}

uint64_t
SimClockSynchronizer::IntervalEndLocked() const
{
    uint64_t end = m_tick + m_step;
    return end > m_origin ? end - m_origin : 0;
}

void
SimClockSynchronizer::ReportLocked()
{
    std::clog << "sim clock: " << m_ticks << " ticks, frame " << m_frame << "; last "
              << m_acks << " frames acknowledged, held " << m_holdMs / std::max<uint64_t>(1, m_acks)
              << " ms per frame on average, " << m_holdSignals
              << " external events while held, clock paused " << m_pausedMs
              << " ms for ns-3, " << m_lateEvents << " of " << m_events
              << " events over 1 ms behind the clock (largest " << m_maxLateNs / 1e6 << " ms)"
              << std::endl;
    m_acks = 0;
    m_holdMs = 0;
    m_holdSignals = 0;
    m_pausedMs = 0;
    m_lateEvents = 0;
    m_events = 0;
    m_maxLateNs = 0;
}

void
SimClockSynchronizer::Listen()
{
    timeval tv{0, 200000}; // wake up to notice m_stop
    setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint8_t buf[2048];
    while (true)
    {
        ssize_t n = recv(m_socket, buf, sizeof(buf), 0);
        auto wall = Clock::now();
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_stop)
        {
            return;
        }
        if (n < 8)
        {
            continue;
        }
        uint64_t t = 0;
        for (int i = 7; i >= 0; --i)
        {
            t = (t << 8) | buf[i];
        }
        bool hello = false;
        if (!m_haveTick)
        {
            m_haveTick = true;
            m_firstTick = t;
            m_tick = 0;
            if (m_gateOn)
            {
                m_step = m_stepTime.GetNanoSeconds(); // until two ticks measure it
                hello = true;
            }
        }
        else
        {
            // The external time of the last tick; m_firstTick is an offset
            // that may wrap (a backward jump makes it "negative"), so all
            // arithmetic on it is modulo 2^64 and only differences are used.
            uint64_t last = m_firstTick + m_tick;
            // A jump far beyond one step is a discontinuity of the external
            // clock, not time that passed: forward a seek or a world that ran
            // unwatched, backward a new episode (CARLA's clock restarts when
            // a map is loaded). Continue from the last tick as if one step had
            // elapsed, instead of leaving ns-3 that far behind or frozen.
            uint64_t jump = std::max<uint64_t>(10 * m_step, 1000000000ULL);
            // One tick has not measured a step yet: assume the configured one.
            uint64_t step = m_step > 0 ? m_step : m_stepTime.GetNanoSeconds();
            uint64_t rel;
            uint64_t dSim;
            // Forward, a jump needs two ticks to tell it from a slow start;
            // backward, any jump beyond the bound is a restart, even after a
            // single tick (a stray frame of the previous episode, then the
            // scenario's world: its ticks must not be ignored until its time
            // passes the old one's).
            bool fwdJump = m_ticks >= 2 && t > last && t - last > jump;
            bool backJump = t < last && last - t > jump;
            if (t > last && !fwdJump)
            {
                rel = t - m_firstTick;
                dSim = rel - m_tick;
            }
            else if (fwdJump || backJump)
            {
                m_firstTick = t - m_tick - step;
                rel = m_tick + step;
                dSim = step;
                ++m_jumps;
                hello = m_gateOn; // a new episode: the frame numbers may start again
                NS_LOG_WARN("external clock jumped; continuing from the last tick");
            }
            else
            {
                continue; // repeated or old tick
            }
            double dWall =
                std::chrono::duration<double, std::nano>(wall - m_tickWallRaw).count();
            if (dWall > 0)
            {
                // Smoothed over a few ticks; a tick's own jitter should not
                // make the interpolation jump.
                double r = std::clamp(dSim / dWall, 0.01, 10.0);
                m_rate = m_ticks < 2 ? r : 0.7 * m_rate + 0.3 * r;
            }
            m_step = dSim;
            m_tick = rel;
        }
        if (m_trace.is_open())
        {
            // The frame that ends with this tick: its tick, its acknowledgement
            // (-1 for none), the clock's pause and the largest lateness.
            auto ms = [this](Clock::time_point w) {
                return std::chrono::duration<double, std::milli>(w - m_wall0).count();
            };
            m_trace << m_frame << "," << m_tick << "," << m_tickArrivalMs << ","
                    << (m_acked ? ms(m_ackWall) : -1.0) << "," << ms(wall) << ","
                    << m_framePausedMs << "," << m_frameMaxLateNs / 1e6 << "\n";
        }
        m_tickArrivalMs = std::chrono::duration<double, std::milli>(wall - m_wall0).count();
        m_framePausedMs = 0;
        m_frameMaxLateNs = 0;
        if (m_acked)
        {
            m_holdMs += std::chrono::duration<double, std::milli>(wall - m_ackWall).count();
            m_acked = false;
        }
        m_tickWall = wall;
        m_tickWallRaw = wall;
        ++m_ticks;
        if (!m_gateOn && m_ticks % 400 == 0)
        {
            ReportLocked();
        }
        if (n >= 12)
        {
            uint32_t count = 0;
            std::memcpy(&count, buf + 8, 4); // little-endian hosts only
            if (n >= static_cast<ssize_t>(12 + 12 * count))
            {
                m_positions.resize(count);
                for (uint32_t i = 0; i < count; ++i)
                {
                    float c[3];
                    std::memcpy(c, buf + 12 + 12 * i, 12);
                    m_positions[i] = Vector(c[0], c[1], c[2]);
                }
                m_positionsTick = m_ticks;
                if (n >= static_cast<ssize_t>(12 + 13 * count))
                {
                    m_conditions.assign(buf + 12 + 12 * count, buf + 12 + 13 * count);
                    m_conditionsTick = m_ticks;
                }
                if (n >= static_cast<ssize_t>(12 + 13 * count + 8))
                {
                    std::memcpy(&m_frame, buf + 12 + 13 * count, 8);
                    m_haveFrame = true;
                }
            }
        }
        if (m_originPending)
        {
            m_originPending = false;
            m_origin = 0;
        }
        lock.unlock();
        if (hello)
        {
            SendGate("HELLO");
        }
        m_cv.notify_all();
    }
}

uint64_t
SimClockSynchronizer::NowLocked()
{
    if (!m_haveTick || m_originPending)
    {
        return m_last;
    }
    double pace = PaceLocked();
    auto wallNow = Clock::now();
    double since = std::chrono::duration<double, std::nano>(wallNow - m_tickWall).count();
    uint64_t lead = static_cast<uint64_t>(std::max(0.0, since * pace));
    if (m_step > 0)
    {
        lead = std::min(lead, m_step);
    }
    else
    {
        lead = 0; // one tick only: no rate yet
    }
    uint64_t abs = m_tick + lead;
    uint64_t now = abs > m_origin ? abs - m_origin : 0;
    // Lockstep: while ns-3 executes an event or works off a backlog, the clock
    // leads that event by at most the slack; the wall time beyond it is not
    // simulation time (the tick's wall time moves on with it).
    uint64_t cap = m_simPos + m_slackTime.GetNanoSeconds();
    if (m_gateOn && !m_idle && now > cap && cap >= m_last && pace > 0)
    {
        uint64_t over = now - cap;
        auto shift = std::chrono::nanoseconds(static_cast<int64_t>(over / pace));
        m_tickWall += std::chrono::duration_cast<Clock::duration>(shift);
        m_pausedMs += over / pace / 1e6;
        m_framePausedMs += over / pace / 1e6;
        now = cap;
    }
    m_last = std::max(m_last, now);
    return m_last;
}

bool
SimClockSynchronizer::DoRealtime()
{
    return true;
}

uint64_t
SimClockSynchronizer::DoGetCurrentRealtime()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return NowLocked();
}

void
SimClockSynchronizer::DoSetOrigin(uint64_t ns)
{
    NS_LOG_FUNCTION(this << ns);
    if (m_socket < 0)
    {
        m_socket = socket(AF_INET, SOCK_DGRAM, 0);
        NS_ABORT_MSG_IF(m_socket < 0, "socket: " << std::strerror(errno));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_port);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        NS_ABORT_MSG_IF(bind(m_socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0,
                        "bind to UDP port " << m_port << ": " << std::strerror(errno));
        if (!m_gate.empty())
        {
            auto colon = m_gate.rfind(':');
            NS_ABORT_MSG_IF(colon == std::string::npos, "Gate is host:port, not " << m_gate);
            addrinfo hints{};
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_DGRAM;
            addrinfo* res = nullptr;
            int rc = getaddrinfo(m_gate.substr(0, colon).c_str(),
                                 m_gate.substr(colon + 1).c_str(),
                                 &hints,
                                 &res);
            NS_ABORT_MSG_IF(rc != 0 || !res, "Gate " << m_gate << ": " << gai_strerror(rc));
            std::memcpy(&m_gateAddr, res->ai_addr, sizeof(m_gateAddr));
            freeaddrinfo(res);
            m_gateOn = true;
            std::clog << "sim clock: lockstep with the tick gate at " << m_gate << ", pace "
                      << (m_fixedRate > 0 ? std::to_string(m_fixedRate) : std::string("measured"))
                      << ", slack " << m_slackTime.GetMilliSeconds() << " ms" << std::endl;
        }
        if (!m_traceFile.empty())
        {
            m_trace.open(m_traceFile);
            m_trace << "frame,t_ns,tick_wall_ms,ack_wall_ms,next_tick_wall_ms,paused_ms,max_late_ms\n";
        }
        m_thread = std::thread(&SimClockSynchronizer::Listen, this);
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_haveTick)
    {
        m_originPending = false;
        m_origin = 0;
        m_last = 0;
        m_origin = NowLocked(); // the clock now, so that it reads 0 here
        m_last = 0;
    }
    else
    {
        m_originPending = true;
    }
}

int64_t
SimClockSynchronizer::DoGetDrift(uint64_t ns)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    uint64_t now = NowLocked();
    return now > ns ? static_cast<int64_t>(now - ns) : -static_cast<int64_t>(ns - now);
}

bool
SimClockSynchronizer::DoSynchronize(uint64_t nsCurrent, uint64_t nsDelay)
{
    uint64_t target = nsCurrent + nsDelay;
    std::unique_lock<std::mutex> lock(m_mutex);
    while (true)
    {
        uint64_t now = NowLocked();
        // Lockstep: the next event lies at or past the interval's end and the
        // clock has reached it, so every event of the interval has run.
        if (m_gateOn && m_haveFrame && !m_acked && m_step > 0 && !m_originPending)
        {
            uint64_t end = IntervalEndLocked();
            if (now >= end && target >= end)
            {
                m_acked = true;
                m_ackWall = Clock::now();
                ++m_acks;
                std::string done = "DONE " + std::to_string(m_frame);
                if (m_acks % 400 == 0)
                {
                    ReportLocked();
                }
                lock.unlock();
                SendGate(done);
                lock.lock();
                continue;
            }
        }
        if (now >= target)
        {
            return true;
        }
        m_idle = true; // caught up: the clock may run until the next event
        if (m_condition)
        {
            return false; // an event arrived from another thread
        }
        // Sleep until the interpolated clock should reach the target, or until a
        // tick or a signal arrives, whichever is first; re-check either way.
        auto wait = std::chrono::milliseconds(50);
        if (m_haveTick && !m_originPending && m_step > 0 && PaceLocked() > 0)
        {
            double ns = (target - now) / PaceLocked();
            wait = std::min(wait, std::chrono::milliseconds(std::max<int64_t>(1, ns / 1e6)));
        }
        m_cv.wait_for(lock, wait);
    }
}

void
SimClockSynchronizer::DoSignal()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_condition = true;
        if (m_acked)
        {
            ++m_holdSignals;
        }
    }
    m_cv.notify_all();
}

void
SimClockSynchronizer::DoSetCondition(bool cond)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_condition = cond;
}

void
SimClockSynchronizer::DoEventStart()
{
    uint64_t ts = Simulator::Now().GetNanoSeconds();
    std::lock_guard<std::mutex> lock(m_mutex);
    m_idle = false;
    m_simPos = ts;
    m_eventStart = NowLocked();
    ++m_events;
    if (m_eventStart > ts + 1000000)
    {
        ++m_lateEvents;
    }
    if (m_eventStart > ts)
    {
        m_maxLateNs = std::max(m_maxLateNs, m_eventStart - ts);
        m_frameMaxLateNs = std::max(m_frameMaxLateNs, m_eventStart - ts);
    }
}

uint64_t
SimClockSynchronizer::DoEventEnd()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return NowLocked() - m_eventStart;
}

} // namespace ns3
