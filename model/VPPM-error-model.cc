/*
*
* Author: Ryan Ackerman <rea9@njit.edu>
* 
*
*/
#include <cmath>
#include "ns3/VPPM-error-model.h"
#include "ns3/packet.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/traced-value.h"
#include "ns3/trace-source-accessor.h"
#include <time.h>
#include <stdlib.h>
namespace ns3 {
//
// OOKErrorModel
//
NS_LOG_COMPONENT_DEFINE("VPPMErrorModel");
NS_OBJECT_ENSURE_REGISTERED (VPPMErrorModel);
//Values of the Standard Luminocity Functions
double VPPMErrorModel::V_lambda[] = {
0.000039, 0.000120, 0.000396, 0.001210, 0.004000, 0.011600, 0.023000,
0.038000, 0.060000, 0.090980, 0.139020, 0.208020, 0.323000, 0.503000,
0.710000, 0.862000, 0.954000, 0.994950, 0.995000, 0.952000, 0.870000,
0.757000, 0.631000, 0.503000, 0.381000, 0.265000, 0.175000, 0.107000,
0.061000, 0.032000, 0.017000, 0.008210, 0.004102, 0.002091, 0.001047,
0.000520, 0.000249, 0.000120, 0.000060, 0.000030 };
//General values for Respositvity
double VPPMErrorModel::Response[] = {
0.150, 0.160, 0.170, 0.190, 0.200, 0.220, 0.230, 0.240, 0.250, 0.260,
0.270, 0.280, 0.300, 0.320, 0.330, 0.350, 0.360, 0.370, 0.375, 0.380,
0.390, 0.400, 0.415, 0.420, 0.430, 0.440, 0.450, 0.460, 0.470, 0.475,
0.480, 0.485, 0.490, 0.495, 0.500, 0.505, 0.510, 0.520, 0.526, 0.532 };
//Constructor
VPPMErrorModel::VPPMErrorModel ()
{
NS_LOG_FUNCTION (this);
srand(time(NULL)); //Seeds Random Number Generator
}

VPPMErrorModel::~VPPMErrorModel ()
{
NS_LOG_FUNCTION (this);
}
/*
TypeId OOKErrorModel::GetTypeId (void)
{
static TypeId tid = TypeId ("ns3::OOKErrorModel")
.SetGroupName("Network")
.AddConstructor<OOKErrorModel> ()
;
return tid;
}
*/
//Calculates Spectral Radiance base on wave length and Blackbody temp of LED
double VPPMErrorModel::SpectralRadiance( int wavelength, double temperature){
double spectral_rad;
double h = 6.62606957e-34; //Planck's constant
double c = 299792458; //speed of light
double k = 1.3806488e-23; //Boltzmann constant
double waveLength = wavelength * 1e-9; //nm
return spectral_rad = 15*((std::pow((h*c)/(M_PI*k*temperature), 4)))/((std::pow(waveLength, 5)) * ((std::exp((h*c)/(waveLength*k*temperature)))-1));
}
//Definite integral of the Luminosity Function(wavelength)*Spectral Radiance(wavelength, temperature) d(wavelength)
double VPPMErrorModel::integralLum(){
double integral = 0;
int waveLower = wavelength_lower;
int waveUpper = wavelength_upper;
while(waveLower <= waveUpper)
{
integral += V_lambda[(waveLower-380)/10] * SpectralRadiance(waveLower, temp) * 10e-9;
waveLower += 10;
}
return integral;
}
//Definite integral of the Spectral Radiance(wavelength, temperature) d(wavelength)
double VPPMErrorModel::integralPlanck(){
double integral = 0;
int waveLower = wavelength_lower;
int waveUpper = wavelength_upper;
while(waveLower <= waveUpper)
{
integral += SpectralRadiance(waveLower, temp) * 10e-9;
waveLower += 10;
}
return integral;
}
//Definite integral of the Response(wavelength)*Spectral Radiance(wavelength, temperature) d(wavelength)

double VPPMErrorModel::integralRes(){
double integral = 0;
int waveLower = wavelength_lower;
int waveUpper = wavelength_upper;
while(waveLower <= waveUpper)
{
integral += Response[(waveLower-380)/10] * SpectralRadiance(waveLower, temp) * 10e-9;
waveLower += 10;
}
return integral;
}
double VPPMErrorModel::getWavelengthUpper()
{
return wavelength_lower;
}
double VPPMErrorModel::getWavelengthLower()
{
return wavelength_upper;
}

void VPPMErrorModel::setDutyCycle(double a){
alpha = a;
} 

void VPPMErrorModel::setb(double B){
b = B;
}


double VPPMErrorModel::getTemperature()
{
return temp;
}
// Virtual method from Error Model. Determinds what packets to be dropped.
bool VPPMErrorModel::DoCorrupt(Ptr<Packet> p){
NS_LOG_FUNCTION(this << p);
BER = calculateBER();
//Caculated the Packet Error Rate by finding the complement of the probablility
//that a packets is not corrupted
double num = 1;
double per = 1.0 - (double)std::pow((double)(1.0 - BER), static_cast<double>((8*p->GetSize())/num));
//Randomizies a number and if its less than the PER the packet is rejected
double rnd = (double) rand()/(double)(RAND_MAX);
return (rnd < per);
}
void VPPMErrorModel::DoReset(void){
}
//Calculates BER from SNR
double VPPMErrorModel::calculateBER (){
//SNR calculation
SNR = (std::pow((Rx*res),2)/No);
double ber;
if(SNR > 0){
//BER calculation
//ser = (2*(M-1)/M)* 0.5 *erfc((std::sqrt(SNR)/std::sqrt(2))/(M-1));
//std::cout<<"SER : " << ser << std::endl;
//std::cout << "alpha = "<< alpha << std::endl;

b = 10;
if(alpha<0.5)
{


	ber = 0.5*erfc((std::sqrt((b*SNR)/(2*alpha)))/std::sqrt(2));
}
else
{
	ber = 0.5*erfc(std::sqrt(((1-alpha)*b*SNR)/(2*std::pow(alpha,2)))/std::sqrt(2));
}

//std::cout << "BER = " << ser << std::endl;
}else{
ber = 1;
}
return ber;
}
//Set Noise power and Received Power
void VPPMErrorModel::setNo (int lower, int upper, int T ,double B, double A, double rx){ //B is the Bandwidth of the electrical filter [b/s] and photodetector Area [cm^2
wavelength_lower = lower;
wavelength_upper = upper;

//alpha = duty_cycle;
temp = T;
Rx = rx;
res = integralRes()/integralPlanck();
double q = 1.60217e-19; //electronic charge [Coulombs]
double k = 1.38064e-23; //Boltzmann constant [m^2 kg s^-2 K^-1]
double I2 = 0.5620; //noise bandwidth factor
double I3 = 0.0868; //noise bandwidth factor
double Ib = 5100e-6; //photocurrent due to background radiation [microA]
double Gol = 10; //open-loop voltage gain
double Cpd = 112e-12; //fixed capacitance of photodetector per unit area [pF/cm^2]
double gm = 30e-3; //FET transconductance [mS]
double	gamma = 1.5; //FET channel noise factor
double abs_temp = 295; //Absolute temperature [K]
double shot_var, thermal_var;
//shot variance
shot_var = 2*q*res*Rx*B + 2*q*Ib*I2*B;
//thermal variance
thermal_var = ((8*M_PI*k*abs_temp)/Gol)*Cpd*A*I2*(std::pow(B, 2)) + ((16*(std::pow(M_PI, 2))*k*abs_temp*gamma)/gm)*(std::pow(Cpd, 2))*(std::pow(A, 2))*I3*(std::pow(B, 3));
No = shot_var + thermal_var;
}
//Gets Noise power
double VPPMErrorModel::getNo(void){
return No;
}
//Gets BER
double VPPMErrorModel::getBER(void){
return BER;
}
//Gets SNR
double VPPMErrorModel::getSNR(void){
return SNR;
}
//Gets DutyCycle
double VPPMErrorModel::getDutyCycle(void){
return alpha;
}
//Gets b
double VPPMErrorModel::getb(void){
return b;
}




//Sets the Symbol size of the M-Pam
//void VPPMErrorModel::setM(double m){
//M = m;
//}
//int VPPMErrorModel::getM(void){
//return M;
//}
} // namespace ns3
