/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef ON_OFF_KEYING_CHANNEL_H
#define ON_OFF_KEYING_CHANNEL_H

#include <list>
#include "ns3/channel.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/traced-callback.h"
#include "on-off-keying-net-device.h"

namespace ns3 {

class PointToPointNetDevice;
class Packet;


class OOKChannel : public Channel 
{
public:
  /**
   * \brief Get the TypeId
   *
   * \return The TypeId for this class
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Create a PointToPointChannel
   *
   * By default, you get a channel that has an "infinitely" fast 
   * transmission speed and zero delay.
   */
  OOKChannel ();

  /**
   * \brief Attach a given netdevice to this channel
   * \param device pointer to the netdevice to attach to the channel
   */
  void Attach (Ptr<OnOffKeyingNetDevice> device);

  /**
   * \brief Transmit a packet over this channel
   * \param p Packet to transmit
   * \param src Source PointToPointNetDevice
   * \param txTime Transmit time to apply
   * \returns true if successful (currently always true)
   */
  virtual bool TransmitStart (Ptr<Packet> p, Ptr<OnOffKeyingNetDevice> src, Time txTime);

  /**
   * \brief Get number of devices on this channel
   * \returns number of devices on this channel
   */
  virtual uint32_t GetNDevices (void) const;

  /**
   * \brief Get PointToPointNetDevice corresponding to index i on this channel
   * \param i Index number of the device requested
   * \returns Ptr to PointToPointNetDevice requested
   */
  Ptr<OnOffKeyingNetDevice> GetOOKDevice (uint32_t i) const;

  /**
   * \brief Get NetDevice corresponding to index i on this channel
   * \param i Index number of the device requested
   * \returns Ptr to NetDevice requested
   */
  virtual Ptr<NetDevice> GetDevice (uint32_t i) const;

protected:
  /**
   * \brief Get the delay associated with this channel
   * \returns Time delay
   */
  Time GetDelay (void) const;

  /**
   * \brief Check to make sure the link is initialized
   * \returns true if initialized, asserts otherwise
   */
  bool IsInitialized (void) const;

  /**
   * \brief Get the net-device source 
   * \param i the link requested
   * \returns Ptr to PointToPointNetDevice source for the 
   * specified link
   */
  Ptr<OnOffKeyingNetDevice> GetSource (uint32_t i) const;

  /**
   * \brief Get the net-device destination
   * \param i the link requested
   * \returns Ptr to PointToPointNetDevice destination for 
   * the specified link
   */
  Ptr<OnOffKeyingNetDevice> GetDestination (uint32_t i) const;

  /**
   * TracedCallback signature for packet transmission animation events.
   *
   * \param [in] packet The packet being transmitted.
   * \param [in] txDevice the TransmitTing NetDevice.
   * \param [in] rxDevice the Receiving NetDevice.
   * \param [in] duration The amount of time to transmit the packet.
   * \param [in] lastBitTime Last bit receive time (relative to now)
   */
  typedef void (* TxRxAnimationCallback)
    (const Ptr<const Packet> packet,
     const Ptr<const NetDevice> txDevice, const Ptr<const NetDevice> rxDevice,
     const Time duration, const Time lastBitTime);
                    
private:
  /** Each point to point link has exactly two net devices. */
  static const int N_DEVICES = 2;

  Time          m_delay;    //!< Propagation delay
  int32_t       m_nDevices; //!< Devices of this channel

  /**
   * The trace source for the packet transmission animation events that the 
   * device can fire.
   * Arguments to the callback are the packet, transmitting
   * net device, receiving net device, transmission time and 
   * packet receipt time.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet>, // Packet being transmitted
                 Ptr<NetDevice>,    // Transmitting NetDevice
                 Ptr<NetDevice>,    // Receiving NetDevice
                 Time,              // Amount of time to transmit the pkt
                 Time               // Last bit receive time (relative to now)
                 > m_txrxPointToPoint;

  /** \brief Wire states
   *
   */
  enum WireState
  {
    /** Initializing state */
    INITIALIZING,
    /** Idle state (no transmission from NetDevice) */
    IDLE,
    /** Transmitting state (data being transmitted from NetDevice. */
    TRANSMITTING,
    /** Propagating state (data is being propagated in the channel. */
    PROPAGATING
  };

  /**
   * \brief Wire model for the PointToPointChannel
   */
  class Link
  {
public:
    /** \brief Create the link, it will be in INITIALIZING state
     *
     */
    Link() : m_state (INITIALIZING), m_src (0), m_dst (0) {}

    WireState                  m_state; //!< State of the link
    Ptr<OnOffKeyingNetDevice> m_src;   //!< First NetDevice
    Ptr<OnOffKeyingNetDevice> m_dst;   //!< Second NetDevice
  };

  Link    m_link[N_DEVICES]; //!< Link model
};

} // namespace ns3

#endif /* ON_OFF_KEYING_CHANNEL_H */
