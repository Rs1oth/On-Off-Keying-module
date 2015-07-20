
/* 
 *
 * Author: Ryan Ackerman   <rea9@njit.edu>
 *         
 *         
*/
#ifndef PAM_ERROR_MODEL_H
#define PAM_ERROR_MODEL_H

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
class PAMErrorModel : public ErrorModel
{
public:
//Constructor and Deconstructor
  PAMErrorModel ();
   ~PAMErrorModel ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */

  //static TypeId GetTypeId (void);

  double No;   // Noise power in A^2
  double Rx;   // Received Power in dbm
  double SER;  // Bit Error Rate
  double res;  //Responsitivity of Receiver
  double SNR;  //Signal to noise ratio
  int wavelength_lower; //Lower bound WaveLength
  int wavelength_upper; //Upper bound Wavelength
  double temp;  // Blackbody temp of LED
  double M; //Size of Symbol
  static double V_lambda[];
  static double Response[];

  double calculateSER (); //Calculates SER

//Used to calculate Responsitivty and Lumanince. 
  double SpectralRadiance(int wavelength, double temperature);
  double integralLum();
  double integralPlanck();
  double integralRes();


  double getWavelengthUpper();

  double getWavelengthLower();

  double getTemperature();

  void setNo (int lower, int upper, int T ,double n, double a , double rx);// Sets Noise and Received Power
  
  void setM (double m); //Sets the size of Symbol
  
  int getM(void);

  double getSER(void); //Returns Symbol Error Rate 
  
  double getNo(void); //Returns Noise power

  double getSNR(void); // Return Signal to Noise Ratio
   
private:

virtual bool DoCorrupt(Ptr<Packet> p); //Virtual method called by simulator to determind when a packets drops/ is corrupted

virtual void DoReset(void); //Virtual method does nothing


double m_rate;//Not used



Ptr<RandomVariableStream> m_ranvar; //Not used
  
};


} // namespace ns3
#endif
