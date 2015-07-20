
/* 
 *
 * Author: Ryan Ackerman   <rea9@njit.edu>
 *         
 *         
*/
#ifndef OOK_ERROR_MODEL_H
#define OOK_ERROR_MODEL_H

#include <list>
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/error-model.h"
#include "ns3/traced-value.h"


namespace ns3 {

class Packet;



//
//AErrorModel
//
class OOKErrorModel : public ErrorModel
{
public:

  OOKErrorModel ();
   ~OOKErrorModel ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */

  //static TypeId GetTypeId (void);

  double No;   // Noise power in A^2
  double Rx;   // Received Power in dbm
  double BER;  // Bit Error Rate
  double res;  //Responsitivity of Receiver
  double SNR;  //Signal to noise ratio
  int wavelength_lower; //Lower bound WaveLength
  int wavelength_upper; //Upper bound Wavelength
  double temp;  // Blackbody temp of LED
  static double V_lambda[];
  static double Response[];

  double calculateBER ();

  void calculateEb();

  double SpectralRadiance(int wavelength, double temperature);

  double integralLum();

  double integralPlanck();

  double integralRes();

  double getWavelengthUpper();

  double getWavelengthLower();

  double getTemperature();

  void setNo (int lower, int upper, int T ,double n, double a, double SetRx);

  //void setRx (double x);

  double getEb(void);

  double getBER(void);
  
  double getNo(void);

  double getSNR(void);
   
private:

virtual bool DoCorrupt(Ptr<Packet> p);

virtual void DoReset(void);


double m_rate;



Ptr<RandomVariableStream> m_ranvar;
  
};


} // namespace ns3
#endif
