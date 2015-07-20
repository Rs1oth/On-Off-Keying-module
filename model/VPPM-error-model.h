
/* 
 *
 * Author: Ryan Ackerman   <rea9@njit.edu>
 *         
 *         
*/
#ifndef VPPM_ERROR_MODEL_H
#define VPPM_ERROR_MODEL_H
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
class VPPMErrorModel : public ErrorModel
{
public:
//Constructor and Deconstructor
VPPMErrorModel ();
~VPPMErrorModel ();
/**
* \brief Get the type ID.
* \return the object TypeId
*/
//static TypeId GetTypeId (void);

double b;
double alpha; //Duty-cycle
double No; // Noise power in A^2
double Rx; // Received Power in dbm
double BER; // Bit Error Rate
double res; //Responsitivity of Receiver
double SNR; //Signal to noise ratio
int wavelength_lower; //Lower bound WaveLength
int wavelength_upper; //Upper bound Wavelength
double temp; // Blackbody temp of LED
// double M; //Size of Symbol
static double V_lambda[];
static double Response[];
double calculateBER (); //Calculates SER
//Used to calculate Responsitivty and Lumanince.
double SpectralRadiance(int wavelength, double temperature);
double integralLum();
double integralPlanck();
double integralRes();
double getWavelengthUpper();
double getWavelengthLower();
double getTemperature();
void setNo (int lower, int upper, int T ,double n, double a , double rx);// Sets Noise and Received Power
//void setM (double m); //Sets the size of Symbol
//int getM(void);
double getBER(void); //Returns Symbol Error Rate
double getNo(void); //Returns Noise power
double getSNR(void); // Return Signal to Noise Ratio


void setDutyCycle(double a); // Sets the Duty Cycle
double getDutyCycle(void); // Returns the Duty Cycle
void setb(double B);
double getb(void);


private:
virtual bool DoCorrupt(Ptr<Packet> p); //Virtual method called by simulator to determind when a packets drops/ is corrupted
virtual void DoReset(void); //Virtual method does nothing
double m_rate;//Not used
Ptr<RandomVariableStream> m_ranvar; //Not used
};
} // namespace ns3
#endif
