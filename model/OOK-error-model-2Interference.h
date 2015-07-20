
/* 
 *
 * Author: Ryan Ackerman   <rea9@njit.edu>
 *         
 *         
*/
#ifndef OOK_2Int_ERROR_MODEL_H
#define OOK_2Int_ERROR_MODEL_H

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
class OOK2IntErrorModel : public ErrorModel
{
public:
//Constructor and Deconstructor
  OOK2IntErrorModel ();
   ~OOK2IntErrorModel ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
//Vector for general values of standard luminocity function
  static double V_lambda[] ;
//Vector for general values of Respositivity
static double Response[] ;


  static TypeId GetTypeId (void);

  double No;   // Noise power in A^2
  double Rx;   // Received Power in dbm
  double BER;  // Bit Error Rate
  double res;  //Responsitivity of Receiver
  double SNR;  //Signal to noise ratio
  double temp;  // Blackbody temp of LED
  double Inttemp; //Interferer's Temp
  double IntRx;  //Interferer's Received power
  double Intres; //Interferer's Respositivity
  double IntNo;    //Interferer's Noise power
  double INR;  //Interference to Noise ratio

  double calculateBER (); //Calculates BER

//Used to calculate Responsitivty and Lumanince. 
  double SpectralRadiance(int wavelength, double temperature);
  double integralLum(int lower, int upper);
  double integralPlanck(int lower, int upper);
  double integralRes(int lower, int upper);


  double getTemperature();

  void setNo (int lower, int upper, int T ,double n, double a, double rx, double intrx); //Sets Noise Power and Received Power

   void setIntNo (int lower, int upper, int T ,double n, double a);//Sets the Interferer's Noise Power and Received Power

  double getBER(void);
  
  double getNo(void);

  double getSNR(void);
  double getINR(void);
   
private:

virtual bool DoCorrupt(Ptr<Packet> p); //Virtual Method called by the Simulator to determin if a packet is corrupted or not

virtual void DoReset(void); //Virtual Method does nothing


double m_rate; // not used



Ptr<RandomVariableStream> m_ranvar;// not used
  
};


} // namespace ns3
#endif
