/*###########################################################################
File: Battery Energy Storage System Emulator
Author: Jessica Boles - jboles@vols.utk.edu
Date: 06/16/2017


INSTRUCTIONS: PREPARE EMULATOR FOR USE:

1. Update the following parameters about the HTB:
	Ts, Fs
	Vbase, Vbaseinv
	Ibase, Ibaseinv
	Pbase
	battery_Inverter_Filter_L

2. Update the following parameters about the system the HTB is emulating:
	Vscale, VscalekV
	Iscale, Iscaleinv
	Pscale

3. Update the following parameters for the desired battery system:
	blocks, blocksinv, blockssquared (Each block corresponds to 1 MW of power capacity. The number of blocks should be the number of MW desired for the battery system)
	battery_type
	battery_BatterySOC (initial condition only)

4. Update the following parameters based on the test mode and the modes of operation desired for the battery system:
	test_mode
	freqreg_mode
	voltagesupport_mode


REFERENCES FOR INTERNAL BATTERY MODELS:

	Lithium Ion and Lead Acid models:
	O. Tremblay and L.-A. Dessaint, “Experimental validation of a battery dynamic model for EV
	applications,” World Electric Vehicle Journal, vol. 3, no. 1, pp. 1–10, May 2009.

	Vanadium Redox Flow model:
	Y. Zhang, J. Zhao, P. Wang, M. Skyllas-Kazacos, B. Xiong, and R. Badrinarayanan, “A
	comprehensive equivalent circuit model of all-vanadium redox flow battery for power system
	analysis,” Journal of Power Sources, vol. 290, pp. 14–24, 2015.


THIS EMULATOR HAS BEEN INCLUDED IN THE FOLLOWING PUBLICATIONS:

	J. D. Boles, Y. Ma, W. Cao, L. M. Tolbert, and F. Wang, “Battery energy storage emulation in
	a converter-based power system emulator,” in Proc. IEEE Applied Power Electronics Conference
	and Exposition, 2017, pp. 1–8.

	J. D. Boles, “Battery energy storage emulation for power systems applications,” Master’s
	Thesis, University of Tennessee, 2017.


//###########################################################################*/


#include "DSP2833x_Device.h"     				// DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   				// DSP2833x Examples Include File
#include "math.h"
#include "Vacon_var_20130627.h"


void Data_store_mon();
void sync_mot();
void sync_mot_ctl();
void turbine();
void PSS();
void PLL();
void Init_VControl_para();
void Init_Gen_para();
void Init_sys();
void Deadtime_comp();
void Trans();
void Ramp();



/********************************** Functions for Battery Emulation ************************/
void Battery_Emulation();

void Battery_Internal();						// Internal battery model
void Battery_BoostConv();						// Boost converter model
void Battery_BoostConvCtrl();					// Boost converter control
void Battery_BoostConvCtrl_Discharge();			// Boost converter control (discharging sub-function)
void Battery_BoostConvCtrl_Charge();			// Boost converter control (charging sub-function)
void Battery_DCLink();							// DC link model
void Battery_HTBInterface_HtoB();				// HTB measurement scaling
void Battery_HTBInterface_BtoH();				// HTB reference scaling
void Battery_InverterCtrl_QCtrl();				// Reactive power control
void Battery_InverterCtrl_VdcCtrl();			// DC link voltage control
void Battery_FreqReg();							// Frequency control
void Battery_VoltageSupport();					// Voltage support control

/*******************************************************************************************/

interrupt void AD_int(void);

#pragma CODE_SECTION(Data_store_mon, "ramfuncs");
#pragma CODE_SECTION(Send_CAN, "ramfuncs");
#pragma CODE_SECTION(sync_mot, "ramfuncs");
//#pragma CODE_SECTION(GEN_Model, "ramfuncs");
#pragma CODE_SECTION(sync_mot_ctl, "ramfuncs");
#pragma CODE_SECTION(turbine, "ramfuncs");
#pragma CODE_SECTION(PSS, "ramfuncs");

#pragma CODE_SECTION(Battery_Emulation, "ramfuncs");
#pragma CODE_SECTION(Battery_Internal, "ramfuncs");
#pragma CODE_SECTION(Battery_BoostConvCtrl_Discharge, "ramfuncs");
#pragma CODE_SECTION(Battery_BoostConvCtrl_Charge, "ramfuncs");
#pragma CODE_SECTION(Battery_BoostConv, "ramfuncs");
#pragma CODE_SECTION(Battery_BoostConvCtrl, "ramfuncs");
#pragma CODE_SECTION(Battery_DCLink, "ramfuncs");
#pragma CODE_SECTION(Battery_HTBInterface_HtoB, "ramfuncs");
#pragma CODE_SECTION(Battery_HTBInterface_BtoH, "ramfuncs");
#pragma CODE_SECTION(Battery_InverterCtrl_QCtrl, "ramfuncs");
#pragma CODE_SECTION(Battery_InverterCtrl_VdcCtrl, "ramfuncs");

double SampleTable1[BUF_SIZE];
double SampleTable2[BUF_SIZE];
double SampleTable3[BUF_SIZE];
double SampleTable4[BUF_SIZE];
double SampleTable5[BUF_SIZE];
double SampleTable6[BUF_SIZE];
double SampleTable7[BUF_SIZE];
double SampleTable8[BUF_SIZE];

Uint32  TestMbox1;
Uint32  TestMbox2;
Uint32  TestMbox3;
Uint32  TestMbox4;
Uint32  TestMbox5;								// On/off button from visualization room
Uint32  TestMbox6;								// Active power command or frequency control command operator input
Uint32  TestMbox7;								// Reactive power command or voltage support command operator input
short income_mailbox;
volatile struct MBOX *Mailbox;
int MIV;
double Sample_count1 =0;

/********************************** Definitions for Battery Emulation ************************/

// HTB System Parameters - Update before use
#define Ts      					0.0001		// DSP sampling time (seconds)
#define Fs							10000.0		// DSP sampling frequency (Hz)
#define Vbase 						43.833		// Voltage base of HTB (V)
#define Vbaseinv 					.0228139	// Reciprocal of HTB voltage base (1/V)
#define Ibase 						45.6277		// Current base of HTB (A)
#define Ibaseinv 					0.0219165	// Reciprocal of HTB current base (1/A)
#define Pbase 						2000		// Power base of HTB (W) (Pbase = Vbase*Ibase)
#define battery_Inverter_Filter_L	0.0005		// Inductor filter on HTB (H)

#define Vscale 						500000		// Voltage base of system being emulated by HTB (V)
#define VscalekV 					500			// Voltage base of system being emulated by HTB (kV)
#define Iscale 						20000		// Current base of system being emulated by HTB (A) (Iscale = Pscale/Vscale)
#define Iscaleinv 					0.00005		// Reciprocal of system current base (1/A)
#define Pscale 						10000000000	// Power base of system being emulated by HTB (W)


// BESS Settings - Update before use
#define blocks 						700			// Number of 1 MW battery blocks in whole battery storage system
#define blocksinv 					0.00142857 	// Reciprocal of number of blocks in whole battery storage system; 00142857 = 1/700 for 700 blocks
#define blockssquared				490000		// Square of number of blocks in whole battery storage system; 490000=700*700 for 700 blocks
#define battery_type 				2			// Battery type: 0 for Lead Acid, 1 for Lithium Ion, 2 for Vanadium Redox flow
long double battery_BatterySOC = .60000000000;  // Battery SOC (proportion between 0 and 1) - NEEDS INITIALIZATION
#define freqreg_mode 				0 			// Frequency control mode: 0 for off (active power reference based on operator command), 1 for on (active power reference automatically generated with frequency control loop)
#define voltagesupport_mode 		0 			// Voltage support mode: 0 for off (reactive power reference based on operator command), 1 for on (reactive power reference automatically generated with voltage support loop)
#define test_mode 					3 			// Test mode: 0 for no voltage, no current, no communications (DSP alone). 1 for communication only (HTB with no voltage). 2 for communication and voltage (HTB with E-stop on). 3 for full emulation on HTB.

// DC-DC Converter - DO NOT CHANGE
#define  battery_BoostConv_Cb 		.01			// Capacitance of the battery-side capacitor in the boost converter (F)
#define  battery_BoostConv_Cb_Inv 	100			// Reciprocal of Cb (1/F)
#define  battery_BoostConv_Lb 		.0001		// Inductance of inductor in the boost converter (H)
#define  battery_BoostConv_Lb_Inv 	10000		// Reciprocal of Lb (1/H)

// DC Link - DO NOT CHANGE
#define	 battery_DCLink_Vdc_Rated 	900			// Rated voltage value of DC link (V)
#define	 battery_DCLink_Cdc			.05			// DC-link capacitance (F)
#define	 battery_DCLink_Cdc_Inv		20			// Reciprocal of the DC-link capacitance (1/F)



/*************************** Parameters and Variables for Battery Emulation ******************/


// Command Variables - DO NOT CHANGE
double battery_Pcommand = 0;					// Active power command (W/block)
double battery_Qcommand = 0;					// Reactive power command (VAR/block)
double freqreg_command = 0; 					// Frequency control command. 2 = IE + PFR, 1 = PFR only, 0 = off. Only applies while freqreg_mode = 1
double voltagesupport_command = 0; 				// Voltage support command. 1 = on, 0 = off. Only applies while voltagesupport_mode = 1

// Internal Battery Model - DO NOT CHANGE
double battery_Ts = Ts/3600;					// DSP sampling time for internal Lithium Ion model (hours)
double battery_Fs = 3600/Ts;					// DSP sampling frequency for internal Lithium Ion model (1/hours)
double battery_K; 								// Polarization constant (V/A or V/Ah)
double battery_B;								// Exponential zone decay factor ((Ah)^-1)
double battery_A;								// Exponential zone amplitude (V)
double battery_R;								// Series resistance (ohm)
double battery_SOCfloor;						// SOC lower limit
double battery_SOCsat;							// SOC saturation point: transition between CC and CV when charging Lithium Ion
double battery_nomvoltage;						// Nominal cell voltage (For Lead Acid and Lithium Ion only) (V)
long double battery_q;							// Total charge in battery (Ah)
double battery_qshort;							// Short double-type version of battery_q
double battery_dis;								// Total-charged-discharged from battery (1-q) (Ah)
double battery_expzone;  						// Voltage drop across exponential zone of battery
double battery_polvoltage;						// Voltage drop due to total-charge-discharged-dependent polarization of battery
double battery_polres;							// Voltage drop due to current-dependent polarization (aka polarization resistance) of battery
double battery_BatteryV;						// Battery string voltage (cells added in series) (V) - NEEDS INITIALIZATION (initialization occurs at beginning of loop)
double battery_BatteryVOLD;						// Battery string voltage from previous DSP control cycle (V) - Initialized with the value of battery_BatteryV
double battery_BatteryI = 0;					// Battery current (discharging = positive, charging = negative)- NEEDS INITIALIZATION
long double battery_SOCtemp;					// Temporary variable to help battery_BatterySOC calculation
double battery_SOCshort;						// Short double-type version of battery_BatterySOC
double battery_cap = 0;							// Battery cell capacity (Ah)
double battery_capinv = 0;						// Reciprocal of battery cell capacity (1/Ah)
double battery_series;							// Number of battery cells in series to make a battery string
double battery_seriesinv;						// Reciprocal of battery_series
double battery_parallel;						// Number of battery strings in parallel to make full battery block
double Vcell;									// Voltage of individual battery cell (V)
double Icell;									// Current of individual battery cell (A)
double VcellOLDOLD;								// Voltage of individual battery cell from two DSP control cycles ago (V)
double VcellOLD;								// Voltage of individual battery cell from previous DSP control cycle (V)
double IcellOLDOLD;								// Current of individual battery cell from two DSP control cycles ago (A)
double IcellOLD;								// Current of individual battery cell from previous DSP control cycle (A)
double battery_C;								// R-C pair capacitance (F)
double battery_E1;								// Voltage across R-C pair (V)
double battery_E1OLD;							// Voltage across R-C pair from previous DSP control cycle (V)
double battery_Eocv;							// Open-circuit cell voltage (for Vanadium redox flow battery only) (V)
double battery_Eo;								// Standard electrode potential of cell, multiplied by number of cells in stack (V)
double battery_F;								// Faraday constant (96485 C/mol)
double battery_Finv;							// Reciprocal of Faraday constant (mol/C)
double battery_flowrate;						// Flow rate of electrolyte (L/min)
double battery_Iint;							// Internal cell current before diffusion occurs (A)
double battery_R1;								// R-C pair resistance (ohm)
double battery_Rint;							// Total internal resistance (ohm)
double battery_R2;								// Total internal resistance minus R-C pair resistance (ohm)
double battery_Rconstant;						// Gas constant (8.314 J/(mol*K))
double battery_T;								// Temperature of battery cell (K)
double battery_Vtank;							// Volume of electrolyte tank (L)
double battery_conc;							// Concentration of Vanadium ions in the electrolyte (M)
double battery_SOCcell;							// Apparent SOC to battery cell
long double battery_SOCtank;					// True SOC, based on electrolyte concentration in tank
double battery_Den1;							// Constant for flow battery model calculations
double battery_Den2;							// Constant for flow battery model calculations
double battery_Den3;							// Constant for flow battery model calculations
double battery_Den;								// Constant for flow battery model calculations
double battery_Deninv;							// Inverse of a constant for flow battery model calculations
double battery_Num1;							// Constant for flow battery model calculations

// DC-DC Converter - DO NOT CHANGE
double battery_BoostConv_Iout = 0; 				// Output current of the boost converter (same as battery_DCLink_Idc1) (A)
double battery_BoostConv_Db = 0;				// Duty-cycle of boost converter (between 0 and 1)
double battery_BoostConv_Il = 0; 				// Inductor current in boost converter (A)
double battery_BoostConv_IlOLD = 0;				// Inductor current from previous DSP control cycle (A)

// DC Link - DO NOT CHANGE
double battery_DCLink_Vdc = battery_DCLink_Vdc_Rated; // DC-link voltage (V)
double battery_DCLink_Idc1 = 0; 				// DC-link current before the DC-link capacitor (same as battery_BoostConv_Iout) (A)
double battery_DCLink_Idc2 = 0; 				// DC-link current after the DC-link capacitor (A)

// DC-DC Converter: Active Power Control - DO NOT CHANGE
double battery_BoostConvCtrl_IlCtrl_Ref = 0; 	// Inductor current reference for boost converter controller (A)
double battery_BoostConvCtrl_IlCtrl_Err = 0; 	// Inductor current error for boost converter controller (A)
double battery_BoostConvCtrl_IlCtrl_Ki = 0; 	// Integral gain for boost converter controller - set inside Battery_BoostConvCtrl() function
double battery_BoostConvCtrl_IlCtrl_Kp = 0;		// Proportional gain for boost converter controller - set inside Battery_BoostConvCtrl() function
double battery_BoostConvCtrl_IlCtrl_Mb = 0;		// Boost converter modulation index (between -1 and 1)
double battery_BoostConvCtrl_IlCtrl_Out = 0;	// Output of boost converter PI controller
double battery_BoostConvCtrl_IlCtrl_Ui = 0;		// Integral contribution to boost converter controller output
double battery_BoostConvCtrl_IlCtrl_Up = 0;		// Proportional contribution to boost converter controller output
double battery_BoostConvCtrl_IlCtrl_VoltageRef = 0; // Battery voltage reference for boost converter controller (V) (only used during CV charging)
double battery_BoostConvCtrl_chargecount = 0;	// Counter for setting initial conditions when charging/discharging state changes

// Inverter: DC Link Voltage Control - DO NOT CHANGE
double battery_InverterCtrl_VdcCtrl_Err = 0; 	// DC link voltage error for DC link voltage controller (V)
double battery_InverterCtrl_VdcCtrl_Kp = 0; 	// Proportional gain for DC link voltage controller - set inside Battery_DCLink() function
double battery_InverterCtrl_VdcCtrl_Ki = 0; 	// Integral gain for DC link voltage controller - set inside Battery_DCLink() function
double battery_InverterCtrl_VdcCtrl_Up = 0; 	// Proportional contribution to DC link voltage controller output
double battery_InverterCtrl_VdcCtrl_Ui = 0;		// Integral contribution to DC link voltage controller output
double battery_InverterCtrl_VdcCtrl_Out = 0; 	// Output of DC link voltage PI controller
double battery_InverterCtrl_VdcCtrl_scaleparams = 12.5/VscalekV; // PI gain parameter scale factor depending on scale of system being emulated

// Inverter: Q control - DO NOT CHANGE
double battery_InverterCtrl_QCtrl_QRef = 0;     // Reactive power reference (VAR/block)
double battery_InverterCtrl_QCtrl_Qmax = 0;		// Maximum allowable reactive power output due to capacity constraints (MVAR)
double battery_InverterCtrl_QCtrl_Qmaxblock = 0; // Maximum allowable reactive power output due to capacity constraints (VAR/block)
double battery_InverterCtrl_QCtrl_scale = 1000000/blocks; // Conversion between MVAR and VAR/block used in QCtrl

// Frequency Control - DO NOT CHANGE
double battery_FreqReg_Ref = 377;				// Frequency reference for frequency controller (radians)
double battery_FreqReg_Err = 0;					// Frequency error for frequency controller (radians)
double battery_FreqReg_ErrOld = 0;				// Frequency error for frequency controller from previous DSP cycle (radians)
double battery_FreqReg_ErrDiff = 0;				// Frequency change between current DSP cycle and previous cycle (radians)
double battery_FreqReg_Kd = 0;					// Derivative gain for frequency controller (used for inertia emulation) - set inside Battery_FreqReg() function
double battery_FreqReg_Kp = 0;					// Proportional gain for frequency controller (used for primary frequency control) - set inside Battery_FreqReg() function
double battery_FreqReg_Out = 0;					// Output of frequency PD controller
double battery_FreqReg_Ud = 0;					// Derivative contribution to frequency controller output
double battery_FreqReg_Up = 0;					// Proportional contribution to frequency controller output
double battery_FreqReg_LPFd1 = 0;				// Constant #1 for frequency LPFs
double battery_FreqReg_LPFd2 = 0;				// Constant #2 for frequency LPFs
double battery_FreqReg_Diff_Old = 0;			// Frequency change between previous DSP cycle and 2 cycles ago (radians)
double battery_FreqReg_Diff_LPF = 0;			// Output of LPF for derivative of frequency (radians)
double battery_FreqReg_Diff_LPF_Old = 0;		// Output of LPF for derivative of frequency from previous DSP control cycle (radians)
double battery_FreqReg_wfreqreg_Old = 377;		// Frequency from previous DSP cycle (radians)
double battery_FreqReg_wfreqreg_LPF = 377;		// Output of LPF for frequency (radians)
double battery_FreqReg_wfreqreg_LPF_Old = 377;	// Output of LPF for frequency from previous DSP control cycle (radians)
double battery_FreqReg_Taud = 0.6;				// Time constant for LPFs
double battery_FreqReg_db = 0;					// Counter for inertia emulation deadband
double battery_FreqReg_Hysteresis = 0;			// Hysteresis trigger for primary frequency control
double battery_FreqReg_startup1 = 1;			// Initial condition trigger for LPF
double battery_FreqReg_startup2 = 1;			// Initial condition trigger for LPF
double battery_FreqReg_startup3 = 1;			// Initial condition trigger for LPF

// Voltage Control - DO NOT CHANGE
double battery_VoltageSupport_Err = 0;			// Voltage error for voltage support controller (per unit)
double battery_VoltageSupport_Kp = 0;			// Proportional gain for voltage support controller - set inside Battery_VoltageSupport() function
double battery_VoltageSupport_Out = 0;			// Output of voltage support P controller
double battery_VoltageSupport_Ref = 0;			// Voltage reference for voltage support controller (per unit)
double battery_VoltageSupport_Up = 0;			// Proportional contribution to voltage support controller output
double battery_VoltageSupport_LPF = 1;			// Output of LPF for voltage (per unit)
double battery_VoltageSupport_LPF_Old = 1;		// Output of LPF for voltage from previous DSP cycle (per unit)
double battery_VoltageSupport_Old = 1;			// Voltage from previous DSP cycle (per unit)
double battery_VoltageSupport_Tauvs = 0.1;		// Time constant for voltage support LPF
double battery_VoltageSupport_LPFvs1 = 0;		// Constant for voltage support LPF
double battery_VoltageSupport_LPFvs2 = 0;		// Constant for voltage support LPF
double battery_VoltageSupport_startup1 = 1;		// Initial condition trigger for voltage support LPF
double battery_VoltageSupport_VHysteresis = 0;	// Hysteresis trigger for voltage support

// HTB Interface - DO NOT CHANGE
double battery_HTBInterface_IdRef = 0; 			// Id reference output of DC link voltage control (A/block)
double battery_HTBInterface_IqRef = 0; 			// Iq reference output of reactive power control (A/block)
double battery_HTBInterface_battery_Vacllrms = 0; // Emulated system AC voltage (line-to-line rms value), (per unit)
double battery_HTBInterface_battery_Vgd = 0;  	// HTB d-axis voltage after inductor filter (V)
double battery_HTBInterface_battery_Vgq = 0;  	// HTB q-axis voltage after inductor filter (V)
double battery_HTBInterface_battery_Vgd_fullscale = 0; // Emulated d-axis system voltage after inductor filter (V)
double battery_HTBInterface_battery_Vgq_fullscale = 0; // Emulated q-axis system voltage after inductor filter (V)
double battery_HTBInterface_battery_Igd = 0;  	// HTB d-axis current (A), then converted to emulated system current per block (A/block)
double battery_HTBInterface_battery_Igq = 0;  	// HTB q-axis current (A), then converted to emulated system current per block (A/block)
double battery_HTBInterface_battery_IgdOld = 0;	// HTB d-axis current from previous DSP cycle (A)
double battery_HTBInterface_battery_IgqOld = 0; // HTB q-axis current from previous DSP cycle (A)
double battery_HTBInterface_battery_Vld = 0;  	// Emulated d-axis system voltage between inverter and inductor filter (V)
double battery_HTBInterface_battery_Vlq = 0;  	// Emulated q-axis system voltage between inverter and inductor filter (V)
double batteryP = 0;							// Active power leaving the emulated battery system per block (W/block)
double batteryPMW = 0;							// Total active power leaving the emulated battery system (MW)
double batteryPkW = 0;							// Total active power leaving the emulated battery system (kW)
double batteryQ = 0;							// Reactive power leaving the emulated battery system per block (VAR/block)

short cal_flag=0, cal_flag1 = 0, cal_flag2 = 0, dt_com_flag = 0;




 /********************************** Main Function **************************************************/


void main(void) {

    Init_sys();
    INITpara();

    PieCtrlRegs.PIEIER9.bit.INTx6 = 1; //ECAN1INTA
    IER |= M_INT9;

    scale_ch0= -0.1063333;
    scale_ch1= -0.1067891;
    scale_ch2= -0.1067891;
    scale_vdc= 0.1583996;
    scale_vab= -0.3030478;
    scale_vbc= -0.3017603;
    scale_vca= -0.3042849;

    offset_ch0= 1787.283;
    offset_ch1= 1758.022;
    offset_ch2= 1772.403;
    offset_vdc= 3.961562;
    offset_vab= 2071.619;
    offset_vbc= 2063.322;
    offset_vca= 2063.322;

    // Set battery parameters and initial conditions for Lead Acid battery - DO NOT CHANGE
    if (battery_type == 0) {

        battery_K = .047;
        battery_B = 125;
        battery_A = .83;
        battery_R = .04;
        battery_SOCfloor = .2;
        battery_SOCsat = .75;
        battery_nomvoltage = 12.4659;
        battery_cap = 7.2;
        battery_capinv = 1/battery_cap;
        battery_series = 50;
        battery_seriesinv = 1/battery_series;
        battery_parallel = 223;

        battery_SOCshort = battery_BatterySOC;
        battery_q = battery_BatterySOC*battery_cap;
        battery_qshort = battery_q;

        battery_dis = battery_cap-(battery_SOCshort*battery_cap); // Set total-charge-discharged initial condition based on SOC initial condition
        battery_BatteryV =  (battery_series)*(battery_nomvoltage + (battery_A * exp(-1*battery_B*(battery_dis))) - (battery_K*battery_cap*(battery_dis)/(battery_SOCshort*battery_cap))); // Set battery string voltage initial condition using internal battery model assuming current is zero
    	battery_BatteryVOLD = battery_BatteryV;

    }

    // Set battery parameters and initial conditions for Lithium Ion battery - DO NOT CHANGE
    else if (battery_type == 1) {

    	battery_K = .0076;		//Lithium Ion battery parameters
    	battery_B = 26.5487;
    	battery_A = .26422;
    	battery_R = .01;
    	battery_SOCfloor = .2;
    	battery_SOCsat = .75;
    	battery_nomvoltage = 3.366;
    	battery_cap = 2.3;
    	battery_capinv = 1/battery_cap;
    	battery_series = 200;
        battery_seriesinv = 1/battery_series;
    	battery_parallel = 646;

    	battery_SOCshort = battery_BatterySOC;
    	battery_q = battery_BatterySOC*battery_cap;
    	battery_qshort = battery_q;

    	battery_dis = battery_cap-(battery_SOCshort*battery_cap); // Set total-charge-discharged initial condition based on SOC initial condition
    	battery_BatteryV =  (battery_series)*(battery_nomvoltage + (battery_A * exp(-1*battery_B*(battery_dis))) - (battery_K*battery_cap*(battery_dis)/(battery_SOCshort*battery_cap))); // Set battery string voltage initial condition using internal battery model assuming current is zero
    	battery_BatteryVOLD = battery_BatteryV;

    }

    // Set battery parameters and initial conditions for Vanadium redox flow battery - DO NOT CHANGE
    else if (battery_type == 2) {

        battery_Eo = 1.39*15;
        battery_Rint = .0248;
        battery_R1 = .0031;
        battery_C = 1900;
        battery_R2 = battery_Rint - battery_R1; // Subtract the R-C pair resistance from the total internal resistance to calculate R2
        battery_conc = 2;
        battery_Vtank = 30;
        battery_F = 96487;
        battery_Finv = 1/battery_F;
        battery_Rconstant = 8.314;
        battery_T = 300;
        battery_flowrate = 4;
    	battery_nomvoltage = battery_Eo;
    	battery_series = 30;
    	battery_seriesinv = 1/battery_series;
    	battery_parallel = 34;
    	battery_BatteryV = battery_Eo*battery_series; // Set battery string voltage initial condition
    	battery_BatteryVOLD = battery_BatteryV;
    	battery_E1OLD = 0;
    	battery_SOCfloor = .2;
    	battery_SOCsat = 1;
    	battery_cap = 47;
    	battery_capinv = 1/battery_cap;

    	battery_SOCtank = battery_BatterySOC;
    	battery_SOCcell = battery_BatterySOC;
    	battery_SOCshort = battery_BatterySOC;
    	battery_q = battery_BatterySOC*battery_cap;
    	battery_qshort = battery_q;
    	battery_dis = battery_cap-(battery_SOCshort*battery_cap); //Set total-charge-discharged initial condition based on SOC initial condition (although not needed for VRB model)

    	battery_Den = battery_R2 + battery_R1/(1 + battery_R1*battery_C*Fs);
    	battery_Deninv = 1/battery_Den;
    	battery_Den1 = 1/(1 + battery_R1*battery_C*Fs);
    	battery_Den2 = 15/(battery_F*battery_Vtank*battery_conc);
    	battery_Den3 = 15/(2*battery_F*battery_flowrate*battery_conc);
    	battery_Num1 = 1.4537*2*15*battery_Rconstant*battery_T*battery_Finv;

    }

    battery_BoostConvCtrl_IlCtrl_Mb = 2*(battery_DCLink_Vdc-battery_BatteryV)/battery_DCLink_Vdc - 1;  // Set DC-DC converter modulation index initial condition based on battery string voltage initial condition
    battery_BoostConvCtrl_IlCtrl_Ui = battery_BoostConvCtrl_IlCtrl_Mb;	// Set DC-DC converter control integrator initial condition equal to modulation index initial condition

    //battery_Fs = 1/battery_Ts;
    //battery_InverterCtrl_QCtrl_MVAsquared = blocks*blocks;

    if ((test_mode == 0)) TestMbox5 = 1;													// Set communication boxes for testing without communication
    if ((test_mode == 0)&&(freqreg_mode == 0)) battery_Pcommand = 0.05*Pscale*blocksinv;	// Set active power command for testing without communication
	if ((test_mode == 0)&&(voltagesupport_mode == 0)) battery_Qcommand = 0.05*Pscale*blocksinv; // Set active power command for testing without communication
    if ((test_mode == 0)&&(freqreg_mode == 1)) freqreg_command = 2;							// Set frequency control command for testing without communication
	if ((test_mode == 0)&&(voltagesupport_mode == 1)) voltagesupport_command = 1;			// Set voltage support command for testing without communication

	// Clear INT flag for this timer
	// EPwm1Regs.ETCLR.bit.INT = 1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    EPwm4Regs.ETCLR.bit.SOCA = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = 0xFFFF;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
   

    // IDLE loop. Loop forever and wait for interrupt
    for(;;) {
    	if ((AD_flag!=0)&&(AD_flag % 4 == 0)) {

    		swcount = (int)AD_flag*0.25;
    		AD_flag=Ia=Ib=Ic=Vdc=Vab=Vbc=Vca=0;

    		switch(swcount) {
   				case 1:
   					swdiv = 1; break;
   				case 2:
   					swdiv = 0.5; break;
   				case 3:
   					swdiv = 0.333333; break;
   				default:
   					swdiv = 0; break;
    		}

    		for (AD_i=0;AD_i<swcount*4;AD_i++) {
    			Ia+=(AD_buffer[0][AD_i]);
    			Ib+=(AD_buffer[1][AD_i]);
    			Ic+=(AD_buffer[2][AD_i]);
    			Vdc+=(AD_buffer[3][AD_i]);
    			Vab+=(AD_buffer[4][AD_i]);
    			Vbc+=(AD_buffer[5][AD_i]);
    			Vca+=(AD_buffer[6][AD_i]);
    		}

    		// Offsets for balanced phase readings
    		Ia=Ia*0.25*swdiv;
    		Ib=Ib*0.25*swdiv+0.9;
    		Ic=Ic*0.25*swdiv+1.2;
    		Vdc=Vdc*0.25*swdiv;
    		Vab=Vab*0.25*swdiv+0.3;
    		Vbc=Vbc*0.25*swdiv;
    		Vca=Vca*0.25*swdiv-2;


    		if (cal_flag==1) {

    			if (TestMbox5==0) cal_flag=0;

    			PLL(); //get w and theta

    			if((test_mode > 0) && (freqreg_mode == 0)) battery_Pcommand = Pscale*(TestMbox6*0.0001-2)*blocksinv;		// Read active power command operator input when frequency control is not enabled
    			if((test_mode > 0) && (voltagesupport_mode == 0)) battery_Qcommand = Pscale*(TestMbox7*0.0001-2)*blocksinv;	// Read reactive power command operator input when voltage support is not enabled
    			if((test_mode > 0) && (freqreg_mode == 1)) freqreg_command = TestMbox6*0.0001-2; 							// Read frequency control operator command input when frequency control is enabled. 2 = IE + primary frequency control, 1 = primary frequency control only, 0 = off
    			if((test_mode > 0) && (voltagesupport_mode == 1)) voltagesupport_command = TestMbox7*0.0001-2; 				// Read voltage support operator command input when voltage support is enabled. 1 = on, 0 = off

    			Battery_Emulation(); // Completes full battery storage system emulation

    			Data_store_mon();

    			Control();

    			CMP_a = (1 + Da_Out)*0.5 * EPWM_TIMER_TBPRD;
    			CMP_b = (1 + Db_Out)*0.5 * EPWM_TIMER_TBPRD;
    			CMP_c = (1 + Dc_Out)*0.5 * EPWM_TIMER_TBPRD;

    			EPwm1Regs.CMPA.half.CMPA = CMP_a;   // Set up switch duty cycle
    			EPwm1Regs.CMPB = CMP_a;  			// Set low switch duty cycle
    			EPwm2Regs.CMPA.half.CMPA = CMP_b;   // Set up switch duty cycle
    			EPwm2Regs.CMPB = CMP_b;  			// Set low switch duty cycle
    			EPwm3Regs.CMPA.half.CMPA = CMP_c;   // Set up switch duty cycle
    			EPwm3Regs.CMPB = CMP_c;  			// Set low switch duty cycle

    		}

    		else {

    			EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
    			EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
    			EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

    			EPwm1Regs.CMPA.half.CMPA = 0;    // Set up switch duty cycle
    			EPwm1Regs.CMPB = 10000;  // Set low switch duty cycle

    			EPwm2Regs.CMPA.half.CMPA = 0;    // Set up switch duty cycle
    			EPwm2Regs.CMPB = 10000;  // Set low switch duty cycle

    			EPwm3Regs.CMPA.half.CMPA = 0;    // Set up switch duty cycle
    			EPwm3Regs.CMPB = 10000;  // Set low switch duty cycle

    			Id_ref = 0;
    			Iq_ref = 0;

	      		if (TestMbox5==1) {
	      			cal_flag=1;
	      			EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	      			EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	      			EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	      		}

	     		PLL(); //get w and theta

	     		Prop = 0;

    		}
    	}
    }
}

/*******************************************************************************************/



void Data_store_mon() {

	if (Sample_count1<600) Sample_count1++;

	else {

		Sample_count1=0;

		if (Sample_count<AVG) {

			// Store data for viewing in SampleTables - Change as desired
			SampleTable1[Sample_count] = (Id_ref * Vd_LPFc + Iq_ref * Vq_LPFc)*0.0005;
			SampleTable2[Sample_count] = (-Vd_LPFc * Iq_ref + Vq_LPFc * Id_ref)*0.0005;
			SampleTable3[Sample_count] = battery_BatteryV;
			SampleTable4[Sample_count] = battery_BatteryI;
			SampleTable5[Sample_count] = battery_BatterySOC*100;
			SampleTable6[Sample_count] = battery_DCLink_Vdc;
			SampleTable7[Sample_count] = w/(2*3.14159);
			SampleTable8[Sample_count] = battery_HTBInterface_battery_Vacllrms;

			Sample_count++;

		}

		else Sample_count=0;

	}
}


interrupt void ECAN_RX_T1() {

	ECanaShadow.CANGIF1.all = ECanaRegs.CANGIF1.all;
    MIV=ECanaShadow.CANGIF1.bit.MIV1;               //get the interrupt mailbox
	income_mailbox = ECanaShadow.CANGIF1.bit.MIV1;   //miv1.0:miv1.4;

	Mailbox = &ECanaMboxes.MBOX16;
	TestMbox1 = Mailbox->MDL.all; // = 0x9555AAAn (n is the MBX number)
	TestMbox2 = Mailbox->MDH.all; // = 0x89ABCDEF (a constant)
	TestMbox3 = Mailbox->MSGID.all;// = 0x9555AAAn (n is the MBX number)

	Mailbox = &ECanaMboxes.MBOX17;
	TestMbox4 = Mailbox->MDL.all; // = 0x9555AAAn (n is the MBX number)
	TestMbox5 = Mailbox->MDH.all; // = 0x89ABCDEF (a constant)
//	TestMbox6 = Mailbox->MSGID.all;// = 0x9555AAAn (n is the MBX number)

	Mailbox = &ECanaMboxes.MBOX18;
	TestMbox6 = Mailbox->MDH.all; // = 0x89ABCDEF (a constant)

	Mailbox = &ECanaMboxes.MBOX19;
	TestMbox7 = Mailbox->MDH.all; // = 0x89ABCDEF (a constant)

	Mailbox = &ECanaMboxes.MBOX20;
	TestMbox1 = Mailbox->MDH.all;

	if ((TestMbox1 == 1)&&(income_mailbox == 20)) {

		ECanaMboxes.MBOX24.MDL.all = (Id_ref * Vd_LPFc + Iq_ref * Vq_LPFc)*0.0005* 100000 +10000000;
	    ECanaMboxes.MBOX24.MDH.all = -(Vd_LPFc * Iq_ref - Vq_LPFc * Id_ref)*0.0005* 100000 +10000000;

		ECanaShadow.CANTRS.all = 0x01000000;
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;  // Set TRS for  transmit mailbox 16

	}

	if ((TestMbox1 == 2)&&(income_mailbox == 20)) {

		ECanaMboxes.MBOX25.MDL.all = battery_BatterySOC * 100000+10000000;//*1592;//1/2/pi
	    ECanaMboxes.MBOX25.MDH.all = battery_HTBInterface_battery_Vacllrms * 100000+10000000;//*1592;//1/2/pi

		ECanaShadow.CANTRS.all = 0x02000000;
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;  // Set TRS for  transmit mailbox 16

	}

	if ((TestMbox1 == 3)&&(income_mailbox == 20)) {

	    ECanaMboxes.MBOX26.MDL.all = battery_BatterySOC * 100000+10000000;//*1592;//1/2/pi
	    ECanaMboxes.MBOX26.MDH.all = battery_DCLink_Vdc * 100000+10000000;//*1592;//1/2/pi

		ECanaShadow.CANTRS.all = 0x04000000;
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;  // Set TRS for  transmit mailbox 16

	}

	sync_flag = 0;
	ECanaRegs.CANRMP.all=0x001f0000;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}

void Send_CAN() {}

void Init_CANMboxes() {

	ECanaMboxes.MBOX16.MSGID.all = 0x3555AAA0;
    ECanaMboxes.MBOX17.MSGID.all = 0x2501AAA0; //140 start
    ECanaMboxes.MBOX18.MSGID.all = 0x2505AAA0; //141 P
    ECanaMboxes.MBOX19.MSGID.all = 0x2509AAA0; //142 Q
    ECanaMboxes.MBOX20.MSGID.all = 0x253DAAA0; //14f request
    ECanaMboxes.MBOX24.MSGID.all = 0x2539AAA0; //14e P,Q
	ECanaMboxes.MBOX25.MSGID.all = 0x2535AAA0; //14d f,v
	ECanaMboxes.MBOX26.MSGID.all = 0x2531AAA0; //14c protectioncode
    ECanaRegs.CANMD.all = 0x001f0000;
    ECanaRegs.CANME.all = 0x071f0000;

    // Specify that 8 bits will be sent/received

    ECanaMboxes.MBOX16.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX17.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX20.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX24.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX26.MSGCTRL.bit.DLC = 8;
}










/********************************** Functions for Battery Emulation ************************/

void Battery_Emulation() {

	Battery_FreqReg();					// Conduct primary frequency regulation and inertia emulation if enabled. Output: battery_Pcommand
	Battery_BoostConvCtrl();			// Conduct active power control using DC-DC converter. Output: battery_BoostConvCtrl_IlCtrl_Mb
	Battery_BoostConv();				// Calculate DC-DC converter states. Outputs: battery_BatteryV, battery_BoostConv_Il, battery_BoostConv_Iout
	Battery_Internal();  				// Calculate internal battery states. Outputs: battery_BatteryI, battery_BatterySOC
	Battery_HTBInterface_HtoB();		// Scale HTB states to emulated BESS units. Outputs: BatteryP, BatteryQ, battery_DCLink_Idc2, battery_HTBInterface_battery_Vacllrms
	Battery_DCLink();					// Calculate DC link states. Output: battery_DCLink_Vdc
	Battery_InverterCtrl_VdcCtrl();		// Conduct DC link voltage control. Output: battery_HTBInterface_IdRef
	Battery_VoltageSupport();			// Conduct voltage support if enabled. Output: battery_Qcommand
	Battery_InverterCtrl_QCtrl();		// Conduct reactive power control. Output: battery_HTBInterface_IqRef
	Battery_HTBInterface_BtoH();		// Scale BESS output to HTB units. Outputs: Id_ref, Iq_ref

}


void Battery_FreqReg() { // Conduct primary frequency regulation and inertia emulation if enabled. Output: battery_Pcommand

	if (freqreg_mode == 1) {

		// Only execute function if frequency control is enabled
		if (freqreg_command > 0) {

			// Calculate constants for frequency LPF
			battery_FreqReg_LPFd1=(2*battery_FreqReg_Taud - Ts)/(2*battery_FreqReg_Taud + Ts); //0.99983;
			battery_FreqReg_LPFd2=Ts/(2*battery_FreqReg_Taud + Ts); //0.00008;

			// Set initial conditions for frequency LPF at startup
			if (battery_FreqReg_startup1 == 1) {
				battery_FreqReg_startup1 = 0;
				battery_FreqReg_wfreqreg_LPF = w;
			}
			// Frequency LPF
			else battery_FreqReg_wfreqreg_LPF = battery_FreqReg_LPFd1*battery_FreqReg_wfreqreg_LPF_Old + battery_FreqReg_LPFd2*(w + battery_FreqReg_wfreqreg_Old);
			// Update "old" values for frequency LPF
			battery_FreqReg_wfreqreg_LPF_Old = battery_FreqReg_wfreqreg_LPF;
			battery_FreqReg_wfreqreg_Old = w;

			// Set the frequency control parameters
			battery_FreqReg_Kp = .01;
			battery_FreqReg_Kd = .1;
			battery_FreqReg_Ref = 2*3.141592653589*60;

			// Calculate frequency error
			battery_FreqReg_Err = battery_FreqReg_Ref - battery_FreqReg_wfreqreg_LPF;

			// Set initial condition for old frequency error at startup
			if (battery_FreqReg_startup2 == 1) {
				battery_FreqReg_startup2 = 0;
				battery_FreqReg_ErrOld = battery_FreqReg_Err;
			}

			// Calculate difference between new and old frequency error (same as difference between new and old frequency)
			battery_FreqReg_ErrDiff = battery_FreqReg_Err - battery_FreqReg_ErrOld;
			battery_FreqReg_ErrOld = battery_FreqReg_Err;//battery_FreqReg_ErrAv;

			// Calculate constants for frequency difference LPF
			battery_FreqReg_LPFd1=.999667; // (.5*2*Taud - Ts)/(.5*2*Taud + Ts);
			battery_FreqReg_LPFd2=.000167; // Ts/(.5*2*Taud + Ts);

			// Set initial conditions for frequency difference LPF at startup
			if (battery_FreqReg_startup3 == 1) {
				battery_FreqReg_startup3 = 0;
				battery_FreqReg_Diff_LPF = battery_FreqReg_ErrDiff;
			}
			// Frequency difference LPF
			else battery_FreqReg_Diff_LPF = battery_FreqReg_LPFd1*battery_FreqReg_Diff_LPF_Old + battery_FreqReg_LPFd2*(battery_FreqReg_ErrDiff + battery_FreqReg_Diff_Old);
			// Update "old" values
			battery_FreqReg_Diff_LPF_Old = battery_FreqReg_Diff_LPF;
			battery_FreqReg_Diff_Old = battery_FreqReg_ErrDiff;

			//Diff_LPF = battery_FreqReg_ErrDiff; // TEST without second LPF

			// Disable inertia emulation if not command-enabled
			if (freqreg_command != 2)  battery_FreqReg_Kd = 0;

			// Enable inertia emulation if frequency difference escapes deadband, then count down for 10 seconds and disable again
			if ((battery_FreqReg_Diff_LPF > 0.00006)||(battery_FreqReg_Diff_LPF < -0.00006))  battery_FreqReg_db = 100000;
			if (battery_FreqReg_db > 0) {
				battery_FreqReg_Kd = battery_FreqReg_Kd;
				battery_FreqReg_db = battery_FreqReg_db - 1;
			}
			else  battery_FreqReg_Kd = 0;

			// Enable primary frequency regulation only if frequency error escapes deadband, then use hysteretic boundary for turn-off
			if ((battery_FreqReg_Err < 0.008)&&(battery_FreqReg_Err > -0.008)) battery_FreqReg_Hysteresis = 0;
			if ((battery_FreqReg_Err > 0.01)||(battery_FreqReg_Err < -0.01)||(battery_FreqReg_Hysteresis==1)) {
				battery_FreqReg_Kp = battery_FreqReg_Kp;
				battery_FreqReg_Hysteresis = 1;
			}
			else  battery_FreqReg_Kp = 0;

			// PD regulator for frequency control
			battery_FreqReg_Up = battery_FreqReg_Kp * battery_FreqReg_Err;
			battery_FreqReg_Ud = battery_FreqReg_Kd * (battery_FreqReg_Diff_LPF) * Fs;
			battery_FreqReg_Out = battery_FreqReg_Up + battery_FreqReg_Ud;

			// Set active power command equal to regulator output
			battery_Pcommand = battery_FreqReg_Out*Pscale*blocksinv;
		}

	// Set active power command equal to zero if frequency control not enabled
	else battery_Pcommand = 0;

	}

}



void Battery_BoostConvCtrl() { // Conduct active power control using DC-DC converter. Output: battery_BoostConvCtrl_IlCtrl_Mb

	// Set the active power control parameters
	battery_BoostConvCtrl_IlCtrl_Kp = .0015;
	battery_BoostConvCtrl_IlCtrl_Ki = .0025;

	// Select the charge/discharge mode based on active power command and calculate inductor current reference
	if (battery_Pcommand > 0) Battery_BoostConvCtrl_Discharge();
	else if (battery_Pcommand < 0) Battery_BoostConvCtrl_Charge();
	else 	battery_BoostConvCtrl_IlCtrl_Ref = 0;

	// Calculate inductor current error
	battery_BoostConvCtrl_IlCtrl_Err = battery_BoostConvCtrl_IlCtrl_Ref - battery_BoostConv_Il;

	// PI regulator for inductor current with saturation limits for the integrator
	battery_BoostConvCtrl_IlCtrl_Up = battery_BoostConvCtrl_IlCtrl_Kp * battery_BoostConvCtrl_IlCtrl_Err;
	battery_BoostConvCtrl_IlCtrl_Ui = battery_BoostConvCtrl_IlCtrl_Ui + (battery_BoostConvCtrl_IlCtrl_Ki * battery_BoostConvCtrl_IlCtrl_Err) * Ts;
		if (battery_BoostConvCtrl_IlCtrl_Ui >= .9) battery_BoostConvCtrl_IlCtrl_Ui = .9;
		if (battery_BoostConvCtrl_IlCtrl_Ui <= -.9) battery_BoostConvCtrl_IlCtrl_Ui = -.9;
	battery_BoostConvCtrl_IlCtrl_Out = battery_BoostConvCtrl_IlCtrl_Up + battery_BoostConvCtrl_IlCtrl_Ui;

	// Set the DC-DC converter modulation index equal to the regulator output as long as limits are not exceeded
	battery_BoostConvCtrl_IlCtrl_Mb = battery_BoostConvCtrl_IlCtrl_Out;
	if (battery_BoostConvCtrl_IlCtrl_Mb >= .99) battery_BoostConvCtrl_IlCtrl_Mb = .99;
	if (battery_BoostConvCtrl_IlCtrl_Mb <= -.99) battery_BoostConvCtrl_IlCtrl_Mb = -.99;

}



void Battery_BoostConvCtrl_Discharge() { // Calculate inductor current reference for discharging

	// Prevent/stop discharge if SOC is below floor constraint
	if (battery_SOCshort<battery_SOCfloor) battery_BoostConvCtrl_IlCtrl_Ref = 0;

	else {
		// Calculate inductor current reference
		battery_BoostConvCtrl_IlCtrl_Ref = battery_Pcommand/battery_BatteryV;

		// Saturate inductor current reference at battery current limits if necessary, assuming 1C charging/discharging limit
		if (battery_BoostConvCtrl_IlCtrl_Ref > (battery_cap*battery_parallel)) battery_BoostConvCtrl_IlCtrl_Ref = battery_cap*battery_parallel;
	}

}



void Battery_BoostConvCtrl_Charge() { // Calculate inductor current reference for charging

	// Select constant current (CC) charging if SOC is below saturation point
	if (battery_SOCshort < battery_SOCsat) {

		// Reset chargecount variable if this is the first cycle of CC charging
        if (battery_BoostConvCtrl_chargecount >= 0) battery_BoostConvCtrl_chargecount = 0;

        // Calculate Il reference assuming CC charging
		battery_BoostConvCtrl_IlCtrl_Ref = battery_Pcommand/(battery_nomvoltage*battery_series);

		// Decrement chargecount variable
        battery_BoostConvCtrl_chargecount = battery_BoostConvCtrl_chargecount-1;
	}

	// Select constant voltage (CV) charging if SOC is above saturation point (Lithium Ion and Lead Acid only)
	else {

		// Set references and reset chargecount variable if this the first cycle of CV charging
        if (battery_BoostConvCtrl_chargecount <= 0) {
        	// Calculate voltage reference if this is the first cycle of charging (no prior CC charging)
            if (battery_BoostConvCtrl_chargecount == 0) battery_BoostConvCtrl_IlCtrl_VoltageRef = 1.15*battery_BatteryV;
            // Keep current voltage as voltage reference if transitioning from CC charging to CV charging
            else battery_BoostConvCtrl_IlCtrl_VoltageRef = battery_BatteryV;
            // Reset chargecount variable
            battery_BoostConvCtrl_chargecount = 0;
        }

        // Calculate Il reference based on voltage reference and internal battery model
        battery_BoostConvCtrl_IlCtrl_Ref = battery_parallel*(-battery_BoostConvCtrl_IlCtrl_VoltageRef*battery_seriesinv - battery_polvoltage + battery_nomvoltage + battery_expzone) / (battery_R + battery_polres);

        // Increment chargecount variable
        battery_BoostConvCtrl_chargecount = battery_BoostConvCtrl_chargecount + 1;
	}

	// Saturate inductor current reference at battery current limits, assuming 1C charging/discharging limit
	if (battery_BoostConvCtrl_IlCtrl_Ref < (-battery_cap*battery_parallel)) battery_BoostConvCtrl_IlCtrl_Ref = -battery_cap*battery_parallel;

}



void Battery_BoostConv() { // Calculate DC-DC converter states. Outputs: battery_BatteryV, battery_BoostConv_Il, battery_BoostConv_Iout

	// Calculate duty cycle based on modulation index
	battery_BoostConv_Db = (1.0000000000000 + battery_BoostConvCtrl_IlCtrl_Mb) * 0.50000000000000000000;

	// Calculate inductor current
	battery_BoostConv_Il = battery_BoostConv_IlOLD + (battery_BatteryVOLD - (1.0000000000 - battery_BoostConv_Db) * battery_DCLink_Vdc) * battery_BoostConv_Lb_Inv * Ts;
	//battery_BoostConv_Il = battery_BatteryI; // TEST with simple transformer model for converter

	// Calculate capacitor voltage (in parallel with the battery string voltage)
	battery_BatteryV = battery_BatteryVOLD + (battery_BatteryI - battery_BoostConv_IlOLD) * battery_BoostConv_Cb_Inv * Ts;
	//battery_BatteryV = (1 - battery_BoostConv_Db) * battery_DCLink_Vdc;    // TEST with simple transformer model for converter

	battery_BoostConv_IlOLD = battery_BoostConv_Il;
	battery_BatteryVOLD = battery_BatteryV;

	// Calculate DC-DC converter current output
	battery_BoostConv_Iout = (1 - battery_BoostConv_Db) * battery_BoostConv_Il; // battery_BoostConv_Iout is equal to current entering DC link (i_dc1)
	//battery_BoostConv_Iout = battery_Pcommand/battery_DCLink_Vdc;    // TEST without active power control

}



void Battery_Internal() { // Calculate internal battery states. Outputs: battery_BatteryI, battery_BatterySOC

	// Internal battery model for Lithium Ion and Lead Acid batteries
	if (battery_type <= 1){


		// Calculate average cell voltage
		Vcell = battery_BatteryV*battery_seriesinv;

		// Calculate voltage drop due to energy-discharged-dependent polarization
		battery_polvoltage = battery_K*battery_cap*battery_dis/battery_qshort;

		// Calculate exponential zone voltage and voltage drop due to current-dependent polarization for charging
		if (battery_Pcommand < 0) {
			if (battery_type == 0) battery_expzone = battery_A - battery_A*exp(-1*battery_B*battery_dis);
			else battery_expzone = battery_A*exp(-1*battery_B*battery_dis);
			battery_polres = battery_K*battery_cap/(battery_dis-0.1*battery_cap);
		}
		// Calculate exponential zone voltage and voltage drop due to current-dependent polarization for discharging
		else {
			battery_expzone = battery_A*exp(-1*battery_B*battery_dis);
			battery_polres = battery_K*battery_cap/(battery_cap-battery_dis);
		}

		// Calculate average cell current using internal battery model; then multiply by number of strings in parallel
		Icell = (-Vcell - battery_polvoltage + battery_nomvoltage + battery_expzone) / (battery_R + battery_polres);
		battery_BatteryI = battery_parallel*Icell;

		// Calculate total battery charge, total charge discharged, and SOC
		battery_q = battery_q - Icell*battery_Ts;
		battery_SOCtemp = battery_q*battery_capinv;
		battery_BatterySOC = battery_SOCtemp;
		battery_SOCshort = battery_BatterySOC;
		battery_qshort = battery_q;
		battery_dis = battery_cap - battery_qshort;


	}

	// Internal battery model for Vanadium redox flow batteries
	else if (battery_type == 2) {

		// Calculate average cell voltage
		Vcell = battery_BatteryV*battery_seriesinv;

		// Calculate open circuit voltage based on battery model
		battery_Eocv = battery_Eo + battery_Num1*log(battery_SOCcell/(1-battery_SOCcell));

		// Calculate cell current based on battery model
		Icell = (battery_Eocv - battery_R1*battery_C*battery_E1OLD*Fs*battery_Den1 - Vcell)*battery_Deninv;
		battery_BatteryI = battery_parallel*Icell;

		// Calculate updated voltage across R-C pair
		battery_E1 = (battery_R1*Icell + battery_R1*battery_C*battery_E1OLD*Fs)*battery_Den1;
		battery_E1OLD = battery_E1;

		// Calculate the internal stack current, the tank SOC, and the apparent cell SOC
		battery_Iint = 1.026*Icell;
		battery_SOCtank = battery_SOCtank - battery_Den2*Icell*Ts;
		battery_SOCcell = battery_SOCtank - battery_Iint*battery_Den3;
		battery_BatterySOC = battery_SOCtank;

	}
}



void Battery_HTBInterface_HtoB() { // Scale HTB states to emulated BESS units. Outputs: BatteryP, BatteryQ, battery_DCLink_Idc2, battery_HTBInterface_battery_Vacllrms

	// Create example voltage values for testing without communication
	if (test_mode <= 1) {
	Vd_LPFc = 79;
	Vq_LPFc = 40;
	}

	// Convert HTB voltage and current readings from power-equivalent to magnitude-equivalent values
	battery_HTBInterface_battery_Vgd = Vd_LPFc * .8165; // D-axis grid voltage in HTB (true HTB voltage). *sqrt(2/3)=.8165: from power-equivalent to magnitude-equivalent
	battery_HTBInterface_battery_Vgq = Vq_LPFc * .8165; // Q-axis grid voltage in HTB (true HTB voltage).
	battery_HTBInterface_battery_Igd = Id_ref * .8165; // D-axis grid current in HTB (true HTB current).
	battery_HTBInterface_battery_Igq = Iq_ref * .8165; // Q-axis grid current in HTB (true HTB current).

	// Calculate the voltage values between inverter output and inductor filter, assuming dq average model
	battery_HTBInterface_battery_Vld = battery_HTBInterface_battery_Vgd + (battery_HTBInterface_battery_Igd-battery_HTBInterface_battery_IgdOld)*battery_Inverter_Filter_L*Fs - w*battery_Inverter_Filter_L*battery_HTBInterface_battery_Igq; // (true D-axis HTB voltage between inverter and inductor filter)
	battery_HTBInterface_battery_Vlq = battery_HTBInterface_battery_Vgq + (battery_HTBInterface_battery_Igq-battery_HTBInterface_battery_IgqOld)*battery_Inverter_Filter_L*Fs + w*battery_Inverter_Filter_L*battery_HTBInterface_battery_Igd; // (true Q-axis HTB voltage between inverter and inductor filter)
	battery_HTBInterface_battery_IgdOld = battery_HTBInterface_battery_Igd;
	battery_HTBInterface_battery_IgqOld = battery_HTBInterface_battery_Igq;

	// Scale HTB voltage and current values to full emulated system units, per BESS power block
	battery_HTBInterface_battery_Vld = battery_HTBInterface_battery_Vld * Vscale * Vbaseinv;
	battery_HTBInterface_battery_Vlq = battery_HTBInterface_battery_Vlq * Vscale * Vbaseinv;
	battery_HTBInterface_battery_Igd = battery_HTBInterface_battery_Igd * Iscale * Ibaseinv * blocksinv;
	battery_HTBInterface_battery_Igq = battery_HTBInterface_battery_Igq * Iscale * Ibaseinv * blocksinv;

	// Calculate active and reactive power output per BESS power block (before inductor filter) in full emulated system units
	batteryP = 1.5*(battery_HTBInterface_battery_Vld * battery_HTBInterface_battery_Igd + battery_HTBInterface_battery_Vlq * battery_HTBInterface_battery_Igq);
	batteryQ = 1.5*(battery_HTBInterface_battery_Vld * battery_HTBInterface_battery_Igq - battery_HTBInterface_battery_Vlq * battery_HTBInterface_battery_Igd);


	// Calculate the DC-link current after the DC-link capacitor in battery model, based on power output before inductor filter
	battery_DCLink_Idc2 = (batteryP) / battery_DCLink_Vdc;

	// Calculate the ac grid voltage (line-to-line rms value)
	battery_HTBInterface_battery_Vacllrms = 1.2247*sqrt(battery_HTBInterface_battery_Vgd * battery_HTBInterface_battery_Vgd + battery_HTBInterface_battery_Vgq * battery_HTBInterface_battery_Vgq)/Vbase;  // sqrt(3/2) = 1.2247449.

}



void Battery_DCLink() { // Calculate DC link states. Output: battery_DCLink_Vdc

	// Set DC link input current equal to DC-DC converter output current
	battery_DCLink_Idc1 = battery_BoostConv_Iout;

	// Calculate change in DC link voltage assuming capacitor-like behavior
	battery_DCLink_Vdc = battery_DCLink_Vdc + (battery_DCLink_Idc1 - battery_DCLink_Idc2) * battery_DCLink_Cdc_Inv * Ts;

}



void Battery_InverterCtrl_VdcCtrl() { // Conduct DC link voltage control. Output: battery_HTBInterface_IdRef

	// Set the DC link voltage control parameters based on system scale
	battery_InverterCtrl_VdcCtrl_Kp = 3.1*battery_InverterCtrl_VdcCtrl_scaleparams;
	battery_InverterCtrl_VdcCtrl_Ki = 304*battery_InverterCtrl_VdcCtrl_scaleparams;

	// Calculate the DC link voltage error
	battery_InverterCtrl_VdcCtrl_Err = battery_DCLink_Vdc_Rated - battery_DCLink_Vdc;

	// PI regulator for DC link voltage control
	battery_InverterCtrl_VdcCtrl_Up = battery_InverterCtrl_VdcCtrl_Kp * battery_InverterCtrl_VdcCtrl_Err;
	battery_InverterCtrl_VdcCtrl_Ui = battery_InverterCtrl_VdcCtrl_Ui + (battery_InverterCtrl_VdcCtrl_Ki * battery_InverterCtrl_VdcCtrl_Err)* Ts;
	battery_InverterCtrl_VdcCtrl_Out = battery_InverterCtrl_VdcCtrl_Up + battery_InverterCtrl_VdcCtrl_Ui;

	// Set the inverter's id reference equal to the negative of the regulator output
	battery_HTBInterface_IdRef = -battery_InverterCtrl_VdcCtrl_Out; //NEEDS TO BE POSITIVE (so negative feedback loop is inverted with minus sign)

}


void Battery_VoltageSupport() { // Conduct voltage support if enabled. Output: battery_Qcommand

	if (voltagesupport_mode == 1) {

		// Only execute function if voltage support is enabled
		if (voltagesupport_command == 1)  {

			// Calculate constants for voltage LPF
			battery_VoltageSupport_LPFvs1=.999; // (2*.01*Tauvs - Ts)/(2*.01*Tauvs + Ts);//
			battery_VoltageSupport_LPFvs2=.0005; // Ts/(2*.01*Tauvs + Ts);//

			// Set initial conditions for voltage LPF at startup
			if (battery_VoltageSupport_startup1 == 1){
				battery_VoltageSupport_startup1 = 0;
				battery_VoltageSupport_LPF = battery_HTBInterface_battery_Vacllrms;
			}
			// Voltage LPF
			else battery_VoltageSupport_LPF = battery_VoltageSupport_LPFvs1*battery_VoltageSupport_LPF_Old + battery_VoltageSupport_LPFvs2*(battery_HTBInterface_battery_Vacllrms + battery_VoltageSupport_Old);
			// Update "old" values for voltage LPF
			battery_VoltageSupport_LPF_Old = battery_VoltageSupport_LPF;
			battery_VoltageSupport_Old = battery_HTBInterface_battery_Vacllrms;

			// Calculate voltage error
			battery_VoltageSupport_Ref = 1;
			battery_VoltageSupport_Err = battery_VoltageSupport_Ref - battery_VoltageSupport_LPF;//battery_HTBInterface_battery_Vacllrms; //TEST without voltage LPF

			// Set the voltage control parameter (or slope)
			battery_VoltageSupport_Kp = .5;

			// Enable voltage support only if frequency error escapes deadband, then use hysteretic boundary for turn-off
			if ((battery_VoltageSupport_Err < .03)&&(battery_VoltageSupport_Err > -0.03)) battery_VoltageSupport_VHysteresis = 0;
			if ((battery_VoltageSupport_Err > .05)||(battery_VoltageSupport_Err < -0.05)||(battery_VoltageSupport_VHysteresis==1)) {
				battery_VoltageSupport_Kp = battery_VoltageSupport_Kp;
				battery_VoltageSupport_VHysteresis = 1;
			}
			else{battery_VoltageSupport_Kp = 0;}

			// P regulator for voltage support
			battery_VoltageSupport_Up = battery_VoltageSupport_Kp * (battery_VoltageSupport_Err);
			battery_VoltageSupport_Out = battery_VoltageSupport_Up;

			// Set reactive power command equal to regulator output
			battery_Qcommand = battery_VoltageSupport_Out*Pscale*blocksinv;
		}

	// Set active power command equal to zero if voltage support is not enabled
    else battery_Qcommand = 0;

	}
}



void Battery_InverterCtrl_QCtrl() { // Conduct reactive power control. Output: battery_HTBInterface_IqRef

	// Calculate the total active power leaving the emulated BESS in kW and MW
	batteryPkW = batteryP*blocks*.001;
	batteryPMW = batteryPkW*.001;

	// Calculate the maximum reactive power output for the emulated BESS based on its capacity and its active power output
	battery_InverterCtrl_QCtrl_Qmax = sqrt((blockssquared) - batteryPMW*batteryPMW); // blocks is equal to power rating in MW
	battery_InverterCtrl_QCtrl_Qmaxblock = battery_InverterCtrl_QCtrl_Qmax*battery_InverterCtrl_QCtrl_scale;

	// Set the reactive power reference equal to the reactive power command as long as the maximum reactive power output is not exceeded
	if (battery_Qcommand > battery_InverterCtrl_QCtrl_Qmaxblock) battery_InverterCtrl_QCtrl_QRef = battery_InverterCtrl_QCtrl_Qmaxblock;
	else if (battery_Qcommand < -battery_InverterCtrl_QCtrl_Qmaxblock) battery_InverterCtrl_QCtrl_QRef = -battery_InverterCtrl_QCtrl_Qmaxblock;
	else battery_InverterCtrl_QCtrl_QRef = battery_Qcommand;

	// Scale HTB voltage readings to full emulated system units
	battery_HTBInterface_battery_Vgd_fullscale = battery_HTBInterface_battery_Vgd * Vscale * Vbaseinv; //already multiplied by .8165 in HtoB
	battery_HTBInterface_battery_Vgq_fullscale = battery_HTBInterface_battery_Vgq * Vscale * Vbaseinv;

	// Calculate the inverter's iq reference based on the reactive power reference, the inverter's id reference, and the emulated system's voltage
	battery_HTBInterface_IqRef = - (battery_InverterCtrl_QCtrl_QRef*.666667 - (battery_HTBInterface_battery_Vgq_fullscale * battery_HTBInterface_IdRef))/battery_HTBInterface_battery_Vgd_fullscale;

}



void Battery_HTBInterface_BtoH() { // Scale BESS output to HTB units. Outputs: Id_ref, Iq_ref

	// Scale the emulated system's output references to the HTB's current base
	Id_ref = battery_HTBInterface_IdRef * 1.2247 * blocks*Ibase*Iscaleinv;  //*sqrt(3/2): from magnitude-equivalent to power-equivalent
	Iq_ref = battery_HTBInterface_IqRef * 1.2247 * blocks*Ibase*Iscaleinv;  //*sqrt(3/2): from magnitude-equivalent to power-equivalent

}






