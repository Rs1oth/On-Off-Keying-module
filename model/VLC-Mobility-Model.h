
/* 
 *
 * Author: Ryan Ackerman   <rea9@njit.edu>
 *         
 *         
*/
#ifndef VLC_MOBILITY_MODEL_H
#define VLC_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"

namespace ns3 {

/**
 * \ingroup mobility
 *
 * \brief Mobility model for which the current acceleration does not change once it has been set and until it is set again explicitely to a new value.
 */
class VlcMobilityModel : public MobilityModel 
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  /**
   * Create position located at coordinates (0,0,0) with
   * speed (0,0,0).
   */
  VlcMobilityModel ();
  virtual ~VlcMobilityModel ();
  /**
   * Set the model's velocity and acceleration
   * \param velocity the velocity (m/s)
   * \param acceleration the acceleration (m/s^2)
   */
  void SetVelocityAndAcceleration (const Vector &velocity, const Vector &acceleration);

  void SetAzimuth(double angle);
  double GetAzimuth(void);

  void SetElevation (double angle);
  double GetElevation (void);  

private:
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;

  Time m_baseTime;  //!< the base time
  Vector m_basePosition; //!< the base position
  Vector m_baseVelocity; //!< the base velocity
  Vector m_acceleration;  //!< the acceleration
  double m_azimuth;    //Rotation (left/;; and Right)
  double m_elevation; //Rotation Angle
};

} // namespace ns3

#endif /* VLC_MOBILITY_MODEL_H */
