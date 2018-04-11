// TI File $Revision: /main/8 $
// Checkin $Date: September 13, 2011   15:17 $
//###########################################################################
//
// FILE:    CSR.c
//
// TITLE:   Action Qualifier Module - Using up/down count
//
// ASSUMPTIONS:
//
//    This program requires the DSP2833x header files.
//
//    Monitor ePWM1-ePWM3 pins on an oscilloscope as described
//    below.
//
//       EPWM1A is on GPIO0
//       EPWM1B is on GPIO1
//
//       EPWM2A is on GPIO2
//       EPWM2B is on GPIO3
//
//       EPWM3A is on GPIO4
//       EPWM3B is on GPIO5
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2833x Boot Mode table is shown below.
//    For information on configuring the boot mode of an eZdsp,
//    please refer to the documentation included with the eZdsp,
//
//       $Boot_Table:
//
//         GPIO87   GPIO86     GPIO85   GPIO84
//          XA15     XA14       XA13     XA12
//           PU       PU         PU       PU
//        ==========================================
//            1        1          1        1    Jump to Flash
//            1        1          1        0    SCI-A boot
//            1        1          0        1    SPI-A boot
//            1        1          0        0    I2C-A boot
//            1        0          1        1    eCAN-A boot
//            1        0          1        0    McBSP-A boot
//            1        0          0        1    Jump to XINTF x16
//            1        0          0        0    Jump to XINTF x32
//            0        1          1        1    Jump to OTP
//            0        1          1        0    Parallel GPIO I/O boot
//            0        1          0        1    Parallel XINTF boot
//            0        1          0        0    Jump to SARAM	    <- "boot to SARAM"
//            0        0          1        1    Branch to check boot mode
//            0        0          1        0    Boot to flash, bypass ADC cal
//            0        0          0        1    Boot to SARAM, bypass ADC cal
//            0        0          0        0    Boot to SCI-A, bypass ADC cal
//                                              Boot_Table_End$
//
// DESCRIPTION:
//
//    This example configures ePWM1, ePWM2, ePWM3 to produce an
//    waveform with independant modulation on EPWMxA and
//    EPWMxB.
//
//    The compare values CMPA and CMPB are modified within the ePWM's ISR
//
//    The TB counter is in up/down count mode for this example.
//
//    View the EPWM1A/B, EPWM2A/B and EPWM3A/B waveforms
//    via an oscilloscope
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################


#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"
#include "Vacon_var_20130627.h"

void INITpara();
void PLL();
void Control();

#pragma CODE_SECTION(PLL, "ramfuncs");
#pragma CODE_SECTION(Control, "ramfuncs");

//#define Ki_i         35//25//28//25//26.6//26.6//28.3//28.3//35
//#define Kp_i         0.105//0.03//0.08//0.04//0.05//0.075//0.08//0.085//0.085 //0.105
#define kr_pr		  5

#define wb           377

#define  DUTYCYCLE_LIMIT   1.2245


double theta = 0;           /* rotor position              (rad)  */
double Da_Out = 0, Db_Out = 0, Dc_Out = 0;
double Vas = 0, Vbs = 0, Vcs = 0;
double w = 0;
double t = 0;
double Modulation_Index = 0;
double CMP_a, CMP_b, CMP_c;


double IMbase = 11.8; //just be positive, the current calculated from motor model is negative
double Iq = 0;
double Id = 0;
double Id_pu = 0;
double Iq_pu = 0;
double Id_ref = 0;
double Iq_ref = 0;
double Iref = 0;

double Id1=0;
double Iq1=0;
double Id1_pre = 0;
double Iq1_pre = 0;
double Id1_LPF1_pre = 0;
double Iq1_LPF1_pre = 0;
double Id1_LPF1 = 0;
double Iq1_LPF1 = 0;

double Error_Id = 0;
double Error_Iq = 0;
double Integra_Id = 0;// 0.943; //2/Vdc*Vd=2/130*50*1.226
double Integra_Iq = 0;//0.1; //2/Vdc*Vq=2/130*0*1.226

double Vq = 0; 
double Vd = 0;

double Vq_pre = 0;
double Vd_LPF = 0;
double Vq_LPF = 0;
double Vq_LPF_pre = 0;
double Vd_pre = 0;
double Vd_LPF_pre = 0;

double Vq_prec = 0;
double Vd_LPFc = 0;
double Vq_LPFc = 0;
double Vq_LPF_prec = 0;
double Vd_prec = 0;
double Vd_LPF_prec = 0;

double Vdc_LPF = 0;
double Vdc_pre = 0;
double Vdc_LPF_pre = 0;

double Vt = 0;

double Iq_LPF = 0;
double Id_LPF = 0;
double Id_pre = 0;
double Iq_pre = 0;
double Id_LPF_pre = 0;
double Iq_LPF_pre = 0;

double Id_ref_ZIP = 0;
double Iq_ref_ZIP = 0;

double Pbasef = 0;
double Qbasef = 0;

double Iq_ZIP_ref = 0;
double Id_ZIP_ref = 0;

double Error_I0 = 0;
double Integra_I0 = 0;
double I0_ref = 0;
double I0 = 0;
double D0_ref = 0;
double Dd_temp = 0; 
double Dd_ref = 0; 
double Dq_temp = 0; 
double Dq_ref = 0; 

double theta_pll = 0;
double w_pll = 0;
double Error_pll = 0;
double Integra_pll = 0;
double w_LPF = 0;
double w_LPF_pre = 0;
double w_pre = 0;

double LPFv1 = 0;
double LPFv2 = 0;
double LPFi1 = 0;
double LPFi2 = 0;
double LPFw1 = 0;
double LPFw2 = 0;
double LPFc1 = 0;
double LPFc2 = 0;

double Prop = 0;

double ia_ref_dc = 0;
double ib_ref_dc = 0;
double ic_ref_dc = 0;
double a1r3,a2r3,b0r3,b2r3;
double a1r5,a2r5,b0r5,b2r5;
double Error_I0_1=0, Error_I0_2=0;
double irout3=0,irout31=0,irout32=0;
double irout5=0,irout51=0,irout52=0;
double costheta=0,sintheta=0;

void INITpara()
{
	  LPFv1=(2*Tauv - PWM_PERIOD)/(2*Tauv + PWM_PERIOD);
	  LPFv2=PWM_PERIOD/(2*Tauv + PWM_PERIOD);

	  LPFi1=(2*Taui - PWM_PERIOD)/(2*Taui + PWM_PERIOD);
	  LPFi2=PWM_PERIOD/(2*Taui + PWM_PERIOD);

	  LPFw1=(2*Tauw - PWM_PERIOD)/(2*Tauw + PWM_PERIOD);
	  LPFw2=PWM_PERIOD/(2*Tauw + PWM_PERIOD);

	  LPFc1=(2*Tauc - PWM_PERIOD)/(2*Tauc + PWM_PERIOD);
	  LPFc2=PWM_PERIOD/(2*Tauc + PWM_PERIOD);

}

void PLL()
{

    Vas = (Vab-Vca)*0.33333333;
    Vbs = (Vbc-Vab)*0.33333333;
    Vcs = (Vca-Vbc)*0.33333333;

    Error_pll = Vq/Vt - 0;
    Integra_pll += Error_pll*PWM_PERIOD * KI_PLL;
    w_pll = Integra_pll + Error_pll*KP_PLL;

    if (Integra_pll > 420) Integra_pll=420;
    if (Integra_pll < 350) Integra_pll=350;

    w_LPF = LPFw1*w_LPF_pre + LPFw2*(w_pll + w_pre);
    w_LPF_pre = w_LPF;
    w_pre = w_pll;
	
   //Test, constant frequency. Wenchao, 20130507
//    w_LPF = 120 * PI;

	w = w_LPF;
    theta_pll += w_LPF*PWM_PERIOD;

    if (theta_pll >= 2*PI)
    {
       theta_pll = theta_pll - 2*PI;
    }

    theta = theta_pll;
    // End of PLL //

    costheta = cos(theta);
    sintheta = sin(theta);

    Vd = 0.8165*(costheta*Vas + ((-0.5)*costheta + 0.866*sintheta)*Vbs + ((-0.5)*costheta - 0.866*sintheta)*Vcs);
    Vq = -0.8165*(sintheta*Vas + (sintheta*(-0.5)-costheta*0.866)*Vbs + (sintheta*(-0.5)+costheta*0.866)*Vcs);
    Vt = sqrt(Vd*Vd+Vq*Vq);

    Id = 0.8165*(costheta*Ia + ((-0.5)*costheta + 0.866*sintheta)*Ib + ((-0.5)*costheta - 0.866*sintheta)*Ic);
    Iq = -0.8165*(sintheta*Ia + (sintheta*(-0.5)-costheta*0.866)*Ib + (sintheta*(-0.5)+costheta*0.866)*Ic);
    I0 = (Ia+Ib+Ic)*0.333;
	
    Vdc_LPF = LPFc1*Vdc_LPF_pre + LPFc2*(Vdc + Vdc_pre);              
    Vdc_LPF_pre = Vdc_LPF;
    Vdc_pre = Vdc;

     Vd_LPFc = LPFc1*Vd_LPF_prec + LPFc2*(Vd + Vd_prec);
     Vd_LPF_prec = Vd_LPFc;
     Vd_prec = Vd;
     Vq_LPFc = LPFc1*Vq_LPF_prec + LPFc2*(Vq + Vq_prec);
     Vq_LPF_prec = Vq_LPFc;
     Vq_prec = Vq;

/*    Id_LPF = LPFi1*Id_LPF_pre + LPFi2*(Id + Id_pre);
    Id_LPF_pre = Id_LPF;
    Id_pre = Id;

    Iq_LPF = LPFi1*Iq_LPF_pre + LPFi2*(Iq + Iq_pre);
    Iq_LPF_pre = Iq_LPF;
    Iq_pre = Iq;
*/
}

void Control()
{

    Iref = sqrt((Id_ref)*(Id_ref) + (Iq_ref)*(Iq_ref));

    if ( Iref >= 70 )
    {
      Id_ref = Id_ref/Iref * 70;
	  Iq_ref = Iq_ref/Iref * 70;
	}
	
	Error_Id = Id_ref - Id;
	Integra_Id += Error_Id*PWM_PERIOD*Ki_i*130/Vdc;
	if (Integra_Id>2) Integra_Id=2;
	if (Integra_Id<-2) Integra_Id=-2;
	Dd_temp = Integra_Id + Error_Id*Kp_i*130/Vdc - Iq*Lr*w*0.0077;     //adding *130/Vdc
	Dd_ref = Dd_temp + 2/Vdc*Vd_LPFc;

	Error_Iq = Iq_ref - Iq;
	Integra_Iq += Error_Iq*PWM_PERIOD*Ki_i*130/Vdc;
	if (Integra_Iq<-2) {Integra_Iq=-2;} //integration limitation
	if (Integra_Iq>2) {Integra_Iq=2;}
	Dq_temp = Integra_Iq + Error_Iq*Kp_i*130/Vdc + Id*Lr*w*0.0077;//
	Dq_ref = Dq_temp + 2/Vdc*Vq_LPFc;

/*	Error_I0=I0_ref-I0;
	Integra_I0 += Error_I0*PWM_PERIOD*Ki_i*130/Vdc;
	if (Integra_I0<-0.1) {Integra_I0=-0.1;} //integration limitation
	if (Integra_I0>0.1) {Integra_I0=0.1;}

	irout3=b0r3*Error_I0+b2r3*Error_I0_2;
	irout3=irout3-a1r3*irout31-a2r3*irout32;

	if (irout3<-0.1) {irout3=-0.1;} // limitation
	if (irout3>0.1) {irout3=0.1;}

	irout32=irout31;
	irout31=irout3;

	Error_I0_2=Error_I0_1;
	Error_I0_1=Error_I0;

	D0_ref = Integra_I0 + Error_I0*Kp_i*130/Vdc +irout3 ;
*/
	D0_ref = 0;

    Modulation_Index = sqrt((Dd_ref)*(Dd_ref) + (Dq_ref)*(Dq_ref) + (D0_ref)*(D0_ref));

    if ( Modulation_Index >= DUTYCYCLE_LIMIT )
    {
      Dd_ref = Dd_ref/Modulation_Index * DUTYCYCLE_LIMIT;
	  Dq_ref = Dq_ref/Modulation_Index * DUTYCYCLE_LIMIT;
	  D0_ref = D0_ref/Modulation_Index * DUTYCYCLE_LIMIT;
	}
	
	Da_Out = 0.8165*(costheta*Dd_ref - sintheta*Dq_ref) + D0_ref;
	Db_Out = 0.8165*(((-0.5)*costheta + 0.866*sintheta)*Dd_ref - (sintheta*(-0.5)-costheta*0.866)*Dq_ref) +D0_ref;
	Dc_Out = 0.8165*(((-0.5)*costheta - 0.866*sintheta)*Dd_ref - (sintheta*(-0.5)+costheta*0.866)*Dq_ref) +D0_ref;

    CMP_a = (1 + Da_Out)*0.5 * EPWM_TIMER_TBPRD;
	CMP_b = (1 + Db_Out)*0.5 * EPWM_TIMER_TBPRD;
    CMP_c = (1 + Dc_Out)*0.5 * EPWM_TIMER_TBPRD;

}



void Deadtime_comp()
{
/*
	    ia_ref_dc = 0.8165*(cos(theta)*Id_ref - sin(theta)*Iq_ref);
	    ib_ref_dc = 0.8165*((-0.5)*cos(theta) + 0.866*sin(theta))*Id_ref - (sin(theta)*(-0.5)-cos(theta)*0.866)*Iq_ref;
	    ic_ref_dc = 0.8165*((-0.5)*cos(theta) - 0.866*sin(theta))*Id_ref - (sin(theta)*(-0.5)+cos(theta)*0.866)*Iq_ref;

	    if (ia_ref_dc>1)
	    	CMP_a += DEAD_TIME;
	    else if (ia_ref_dc<-1)
	        CMP_a -= DEAD_TIME;
	    else
	    	CMP_a += ia_ref_dc*DEAD_TIME;
	    if (ib_ref_dc>1)
	    	CMP_b += DEAD_TIME;
	    else if (ib_ref_dc<-1)
	    	CMP_b -= DEAD_TIME;
	    else
	    	CMP_b += ib_ref_dc*DEAD_TIME;
	    if (ic_ref_dc>1)
	    	CMP_c += DEAD_TIME;
	    else if (ic_ref_dc<-1)
	    	CMP_c -= DEAD_TIME;
	    else
	    	CMP_c += ic_ref_dc*DEAD_TIME;
*/
}
