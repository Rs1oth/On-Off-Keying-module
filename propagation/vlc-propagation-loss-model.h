
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007,2008, 2009 INRIA, UDcast
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
 *
 * Author: James Basuino       <jb392@njit.edu>
 *                              
 */
 
 /*Normal Header structure:
   All parts of the equations that are constant have set and get functions
   while the variables have only get functions as they are calculated and rely on the position of the nodes
   */

#ifndef VLC_PROPAGATION_LOSS_MODEL_H
#define VLC_PROPAGATION_LOSS_MODEL_H
#include <vector>
#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/mobility-module.h"
#include "ns3/VLC-Mobility-Model.h"

namespace ns3 {

class VLCPropagationLossModel : public PropagationLossModel
{

public:
  /**
   * \breif Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  //Constructor
  VLCPropagationLossModel ();
  void SetTxPower(double dBm);
  void SetTxPower(double deviceMax, double peak, double desired);
  void setEfficacy (int low, int up, double t);
  double getEfficacy();
  double GetTxPower();
  double GetOpticalPower();
  void SetLambertianOrder(double semiangle); //Must be in degrees
  double GetLambertianOrder();
  void SetFilterGain(double gain);
  double GetFilterGain();
  void SetConcentratorGain(double fov, double refracIndex); //fov is an angle and must be in degrees
//, Ptr<MobilityModel> a, Ptr<MobilityModel> b);
  double GetConcentratorGain();
  void SetPhotoDetectorArea(double area); // meters squared
  static double Fov;
  double GetPhotoDetectorArea();
  double GetDistance(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
  double GetRadianceAngle(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
  double GetIncidenceAngle(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
  double GetRxPower(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
  double dotProduct(std::vector<double> v1, std::vector<double> v2) const; 
  double magnitude(std::vector<double> v) const; 

  int wavelength_lower; //Lower bound WaveLength
  int wavelength_upper; //Upper bound Wavelength
  double temp;  // Blackbody temp of LED

  double SpectralRadiance(int wavelength, double temperature);

  double integralLum();

  double integralPlanck();

private:
/**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  VLCPropagationLossModel (const VLCPropagationLossModel &);
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns
   */
  VLCPropagationLossModel & operator = (const VLCPropagationLossModel &);
  double calculateIlluminance(Ptr<MobilityModel> a, Ptr<MobilityModel> b);
  virtual double DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
  virtual int64_t DoAssignStreams (int64_t stream);
  double m_TxPower;
  double m_LambertianOrder;
  double m_FilterGain;
  double m_ConcentratorGain;
  double m_PhotoDetectorArea;
  double m_RxPower;
  double m_efficacy;
  double illuminance;
  double m_Optical;

};

}

#endif /* VLCPROPAGATIONMODEL*/
