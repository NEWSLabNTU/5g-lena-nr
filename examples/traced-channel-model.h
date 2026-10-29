// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef TRACED_CHANNEL_MODEL_H
#define TRACED_CHANNEL_MODEL_H

#include "ns3/matrix-based-channel-model.h"

#include <atomic>
#include <complex>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

namespace ns3
{

/// The newest CARLA frame whose traced paths are complete (-1 for none), for
/// SimClockSynchronizer's channel lag.
extern std::atomic<int64_t> g_tracedChannelFrame;

/**
 * @brief The channel between the gNB and each UE from paths traced through the
 * scene (2026-coop-perception phase 026).
 *
 * The RT server (sim/rt/rt_server.py) traces every CARLA frame with Sionna RT
 * and sends each UE's paths over UDP (Port): per path the delay, the complex
 * gain of single isotropic antennas (path loss and interactions included, no
 * propagation phase), the directions of departure at the gNB and of arrival at
 * the UE (global frame) and the Doppler shift (both ends' motion and moving
 * scatterers). Each path becomes one cluster of the channel matrix: its gain
 * times the arrays' responses in its directions, as ThreeGppChannelModel forms
 * a ray, so 5G-LENA's beamforming and spectrum model apply unchanged. The gain
 * is absolute: the channel carries no propagation loss model. The Doppler goes
 * through the spectrum model's scatterer term (2 alpha D), the nodes having no
 * velocity of their own, and the gains are rotated back by the Doppler phase at
 * the frame's time, so that the phase is the traced one there.
 *
 * Datagrams, little-endian: "RT5P", frame u64, t_ns u64 (ns-3's simulation
 * time of the frame), ue u16, n u16, then n times f32 delay ns, re, im, aod,
 * zod, aoa, zoa (rad), Doppler Hz; "RT5E", frame u64 ends a frame. A UE with no
 * path in the newest frame gets one 300 dB down.
 */
class TracedChannelModel : public MatrixBasedChannelModel
{
  public:
    static TypeId GetTypeId();
    TracedChannelModel();
    ~TracedChannelModel() override;

    /// The gNB's node and each UE's node, by the RT server's UE index; starts
    /// listening for the RT server unless `listen` is false (tests).
    void SetNodes(uint32_t gnbNode, const std::vector<uint32_t>& ueNodes, bool listen = true);

    struct Path;
    /// A complete frame of paths by UE index, as if it had arrived (tests).
    void SetPaths(int64_t frame, const std::map<int, std::vector<Path>>& paths);

    Ptr<const ChannelMatrix> GetChannel(Ptr<const MobilityModel> aMob,
                                        Ptr<const MobilityModel> bMob,
                                        Ptr<const PhasedArrayModel> aAntenna,
                                        Ptr<const PhasedArrayModel> bAntenna) override;
    Ptr<const ChannelParams> GetParams(Ptr<const MobilityModel> aMob,
                                       Ptr<const MobilityModel> bMob) const override;

    /// One traced path, gNB to UE.
    struct Path
    {
        double delayNs;
        std::complex<double> gain;
        double aod, zod, aoa, zoa; ///< rad, global frame
        double dopplerHz;
    };

  protected:
    void DoDispose() override;

  private:
    void Listen();
    /// The UE index of a node, or -1 for the gNB or an unknown node.
    int UeOf(uint32_t node) const;

    uint16_t m_port;
    double m_frequency;
    int m_socket;
    std::thread m_thread;
    std::atomic<bool> m_stop;
    uint32_t m_gnbNode;
    std::map<uint32_t, int> m_ueByNode;

    mutable std::mutex m_mutex;
    int64_t m_frame;                                ///< frame of m_paths
    Time m_frameTime;                               ///< when ns-3 first used it
    std::map<int, std::vector<Path>> m_paths;       ///< newest complete frame
    std::map<int, std::vector<Path>> m_pending;     ///< frame being received
    int64_t m_pendingFrame;
    /// Channel matrices of the current frame, by node-pair key.
    std::map<uint64_t, Ptr<ChannelMatrix>> m_matrices;
    mutable std::map<uint64_t, Ptr<ChannelParams>> m_params;
    int64_t m_cachedFrame;
};

} // namespace ns3

#endif // TRACED_CHANNEL_MODEL_H
