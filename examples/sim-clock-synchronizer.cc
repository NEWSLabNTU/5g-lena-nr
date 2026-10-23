// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#include "sim-clock-synchronizer.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
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
                          MakeUintegerChecker<uint16_t>());
    return tid;
}

SimClockSynchronizer::SimClockSynchronizer()
    : m_port(5600),
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
      m_eventStart(0)
{
}

SimClockSynchronizer::~SimClockSynchronizer()
{
    DoDispose();
}

void
SimClockSynchronizer::DoDispose()
{
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
        if (!m_haveTick)
        {
            m_haveTick = true;
            m_firstTick = t;
            m_tick = 0;
        }
        else if (t > m_firstTick + m_tick)
        {
            uint64_t rel = t - m_firstTick;
            uint64_t dSim = rel - m_tick;
            // A jump far beyond one step is a discontinuity of the external
            // clock (a restart or a seek), not time that passed: continue
            // from the last tick as if one step had elapsed, instead of
            // leaving ns-3 that far behind.
            uint64_t jump = std::max<uint64_t>(10 * m_step, 1000000000ULL);
            if (m_ticks >= 2 && dSim > jump)
            {
                uint64_t step = m_step > 0 ? m_step : 0;
                m_firstTick += dSim - step;
                rel = m_tick + step;
                dSim = step;
                ++m_jumps;
                NS_LOG_WARN("external clock jumped; continuing from the last tick");
            }
            double dWall =
                std::chrono::duration<double, std::nano>(wall - m_tickWall).count();
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
        else
        {
            continue; // repeated or old tick
        }
        m_tickWall = wall;
        ++m_ticks;
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
            }
        }
        if (m_originPending)
        {
            m_originPending = false;
            m_origin = 0;
        }
        lock.unlock();
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
    double since = std::chrono::duration<double, std::nano>(Clock::now() - m_tickWall).count();
    uint64_t lead = static_cast<uint64_t>(std::max(0.0, since * m_rate));
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
        if (now >= target)
        {
            return true;
        }
        if (m_condition)
        {
            return false; // an event arrived from another thread
        }
        // Sleep until the interpolated clock should reach the target, or until a
        // tick or a signal arrives, whichever is first; re-check either way.
        auto wait = std::chrono::milliseconds(50);
        if (m_haveTick && !m_originPending && m_step > 0 && m_rate > 0)
        {
            double ns = (target - now) / m_rate;
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
    std::lock_guard<std::mutex> lock(m_mutex);
    m_eventStart = NowLocked();
}

uint64_t
SimClockSynchronizer::DoEventEnd()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return NowLocked() - m_eventStart;
}

} // namespace ns3
