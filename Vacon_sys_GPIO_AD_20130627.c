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

void Init_sys();
void InitPWM(void);
void Init_ADC(void);
void Init_GPIO(void);
void AD_read();
interrupt void ECAN_TX_TO(void);
interrupt void ECAN_RX_T1(void);


#pragma CODE_SECTION(AD_read, "ramfuncs");
#pragma CODE_SECTION(AD_int, "ramfuncs");

double AD_buffer[11][12];
double Average_AD1,Average_AD2,Average_AD3;


double scale_ch0;          // CW: Only Iu, Iv, Iw are used in the program.
double scale_ch1;
double scale_ch2;
double scale_vdc;
double scale_vab;
double scale_vbc;
double scale_vca;
double scale_t1;
double scale_t2;
double scale_t3;

double offset_ch0;
double offset_ch1;
double offset_ch2;
double offset_vdc;
double offset_vab;
double offset_vbc;
double offset_vca;
double offset_t1;
double offset_t2;
double offset_t3;

int interrupt_count=0;

int AD_i,AD_flag=0;
int swcount=0;
double swdiv=0;

int temp_tsc;
int temp_ctr;
int prd_offset=0;
int prd=7500;

unsigned long testshadow;

short Send_flag=0;
short sync_flag=0;

double Ia,Ib,Ic;
double Vab,Vbc,Vca;
double Vdc;

int j, Sample_count=0, Save_flag=0;

Uint16 ii=0, cycle_count=0;

interrupt void AD_int(void)
{
    EPwm4Regs.ETCLR.bit.SOCA = 1;
    AD_read();


	interrupt_count++;
	if (interrupt_count>=32)
	{
		interrupt_count=0;

		Send_CAN();
	}

	if 	(interrupt_count==1)
	{

	}

	if 	(interrupt_count==2)
	{

	}


	if ((ECanaRegs.CANTSC>300)&&(sync_flag==0))
	{
		sync_flag = 1;
		temp_ctr = EPwm1Regs.TBCTR;
		if (temp_ctr>EPwm1Regs.TBCTR) temp_ctr = 15000-temp_ctr;
		temp_tsc = ECanaRegs.CANTSC-300;
		temp_tsc = temp_tsc*300;

		temp_tsc -= temp_ctr;
		if (temp_tsc>7500) {prd_offset -= (temp_tsc-15000)*0.001-1;}
		else if (temp_tsc<-7500) {prd_offset -= (temp_tsc+15000)*0.001+1;}
		else if (temp_tsc>0) {prd_offset -= (temp_tsc)*0.001+1;}
		else if (temp_tsc<0) {prd_offset -= (temp_tsc)*0.001-1;}

		if (prd_offset>40) prd_offset=40;
		if (prd_offset<-40) prd_offset=-40;

		prd = EPWM_TIMER_TBPRD;//prd_offset+
		EPwm1Regs.TBPRD = prd;           // Set timer period
		EPwm2Regs.TBPRD = prd;           // Set timer period
		EPwm3Regs.TBPRD = prd;           // Set timer period
//		EPwm4Regs.TBPRD = prd/2-1;           // Set timer period

/*		if   (Sample_count<AVG)
			{
			   SampleTable1[Sample_count] = ECanaRegs.CANTSC;
			   SampleTable2[Sample_count] = EPwm1Regs.TBCTR;
			   SampleTable3[Sample_count] = temp_ctr;
			   SampleTable4[Sample_count] = temp_tsc;
			   SampleTable5[Sample_count] = prd;
			   Sample_count++;
			}
			else { Sample_count=0;}
*/
	}


// Clear INT flag for this timer
//    EPwm1Regs.ETCLR.bit.INT = 1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;


// Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



void AD_read()
{
//   AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;               // CW: Clear interrupt bit.

if ((AD_flag<=12)&&(AD_flag>=0))
{
   AD_buffer[0][AD_flag]=((((double)(AdcRegs.ADCRESULT0>>4)
   	  +(double)(AdcRegs.ADCRESULT8>>4)) * 0.5)-offset_ch0)*scale_ch0; 	//ia

   AD_buffer[1][AD_flag]=((((double)(AdcRegs.ADCRESULT1>>4)
   	  +(double)(AdcRegs.ADCRESULT9>>4)) * 0.5)-offset_ch1)*scale_ch1;  //ib

   AD_buffer[2][AD_flag]=((((double)(AdcRegs.ADCRESULT2>>4)
   	  +(double)(AdcRegs.ADCRESULT10>>4)) * 0.5)-offset_ch2)*scale_ch2; //ic

   AD_buffer[4][AD_flag]=((((double)(AdcRegs.ADCRESULT4>>4)
   	  +(double)(AdcRegs.ADCRESULT12>>4)) * 0.5)-offset_vab)*scale_vab; 	//vab

   AD_buffer[5][AD_flag]=((((double)(AdcRegs.ADCRESULT5>>4)
   	  +(double)(AdcRegs.ADCRESULT13>>4)) * 0.5)-offset_vbc)*scale_vbc; 	//vbc

   AD_buffer[6][AD_flag]=((((double)(AdcRegs.ADCRESULT6>>4)
   	  +(double)(AdcRegs.ADCRESULT14>>4)) * 0.5)-offset_vca)*scale_vca; 	//vca

   AD_buffer[3][AD_flag]=((((double)(AdcRegs.ADCRESULT3>>4)
	  +(double)(AdcRegs.ADCRESULT11>>4)) * 0.5)-offset_vdc)*scale_vdc; 	    //vdc

   AD_buffer[8][AD_flag]=((((double)(AdcRegs.ADCRESULT7>>4)
	  +(double)(AdcRegs.ADCRESULT15>>4)) * 0.5)-offset_t1)*scale_t1; 	    //t1

	  
   AD_flag++;
}
else {AD_flag =0;}
}

void Init_sys()
{
	// System Initialization
	// Initialize ADC and PWM which are used later //

	// Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the DSP2833x_SysCtrl.c file.
	   InitSysCtrl();

	// Specific clock setting for this example:
	   EALLOW;
	   SysCtrlRegs.HISPCP.all = ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
	   EDIS;

   	   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

   	// Call Flash Initialization to setup flash waitstates
   	// This function must reside in RAM
   	   InitFlash();

	// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
	// These functions are in the DSP2833x_EPwm.c file
	   InitEPwm1Gpio();
	   InitEPwm2Gpio();
	   InitEPwm3Gpio();
	   InitEPwm4Gpio();
	   InitEPwm5Gpio();
	   InitEPwm6Gpio();
	   InitECanGpio();

	// Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	   DINT;


	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	   InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	   IER = 0x0000;
	   IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
	// This function is found in DSP2833x_PieVect.c.
	   InitPieVectTable();

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	   EALLOW;	// Allow access to EALLOW protected registers
	//   PieVectTable.EPWM1_INT = &PWM_GEN; // Use 3 channels of ePWM a and b channel as complementary pulses for a phase leg
	   PieVectTable.SEQ1INT = &AD_int; // Use 3 channels of ePWM a and b channel as complementary pulses for a phase leg
	   PieVectTable.ECAN0INTA = &ECAN_TX_TO; // Use 3 channels of ePWM a and b channel as complementary pulses for a phase leg
	   PieVectTable.ECAN1INTA = &ECAN_RX_T1; // Use 3 channels of ePWM a and b channel as complementary pulses for a phase leg

	   EDIS;   // Disable access to EALLOW protected registers

	   // Enable CPU INT3 which is connected to EPWM1-3 INT:
	   //   IER |= M_INT3;
	   	IER |= M_INT1;
	//   	IER |= M_INT9;


	   // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
	   //   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
	      PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

	// Initialize all the Device Peripherals:
	// This function is found in DSP2833x_InitPeripherals.c
	// InitPeripherals(); // Not required for this example

	// Initialize ePWM 6 Channels
	// While initiating PWM, make sure to have output to have low voltage
	   EALLOW; // before initiating PWM, stop clock first
	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	   EDIS;

	   InitPWM(); // Need Programming

	   EALLOW;
	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	//   SysCtrlRegs.PCLKCR0.bit.ECANENCLK=1;
	   EDIS;


	   Init_CAN();

	// Set ADC settings for sampling
	   Init_ADC();

	   Init_GPIO();


}

void Init_ADC()
{
	   InitAdc();


	   AdcRegs.ADCMAXCONV.all=15;
	   AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;
	   AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
	   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // 1  Cascaded mode

	   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;   // ADCINA0
	   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;   // ADCINA1
	   AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;   // ADCINA2
	   AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;   // ADCINA3
	   AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;   // ADCINA0
	   AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;   // ADCINA1
	   AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;   // ADCINA2
	   AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x8;   // ADCINB0
	   AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x0;   // ADCINA0
	   AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x1;   // ADCINA1
	   AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x2;   // ADCINA2
	   AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0x3;   // ADCINB1
	   AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0x4;   // ADCINA0
	   AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0x5;   // ADCINA1
	   AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0x6;   // ADCINA2
	   AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0x8;   // ADCINB2

	   AdcRegs.ADCTRL1.bit.CONT_RUN = 0;       // Setup continuous run

	  AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x1;   // Enable SOCA from ePWM to start SEQ1
	  AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0x1;     // Enable SEQ1 interrupt (every EOS)

	//   AdcRegs.ADCTRL2.all = 0x2000;	// Start AD Sampling and Conversion Right Here

}

void Init_GPIO()
{
    EALLOW;
	GpioCtrlRegs.GPCPUD.bit.GPIO69 = 0;   // Enable pullup on GPIO69
	GpioCtrlRegs.GPCPUD.bit.GPIO70 = 0;   // Enable pullup on GPIO70
	GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 0x00000000;  // Only GPIO69
	GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0x00000000;  // Only GPIO70
//	GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 0xFF; //Sampling time for GPIO16 to GPIO23 is 510*Tsysclk
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 0x2; //Qualification using 6 samples, sampling window = 510*Tsysclk*5 = 17us
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 0x2; //Qualification using 6 samples, sampling window = 510*Tsysclk*5 = 17us
    GpioCtrlRegs.GPCDIR.bit.GPIO69 = 0;   // GPIO69 input, detect the start/stop signal from push button
//	GpioDataRegs.GPCSET.bit.GPIO69 = 1;   // Load output latch
    GpioCtrlRegs.GPCDIR.bit.GPIO70 = 0;   // GPIO70 input, detect the output trigger from push button
//	GpioDataRegs.GPCSET.bit.GPIO70 = 1;   // Load output latch

	//GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;   // GPIO17 outputs
	//GpioDataRegs.GPASET.bit.GPIO17 = 1;   // Load output latch
    EDIS;
}

void InitPWM(void)
{
   // for PWM1
   // Setup TBCLK
   EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;           // Set timer period
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
   EPwm1Regs.CMPB = EPWM_DUTY;               // Set Compare B value

   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
//   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
//   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync down-stream module
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;         // Set PWM1A on event A, counter incrementing
   EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM1A on event A, counter decrementing

   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;
   EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;      // INT on Time Counter Period event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st of INTSEL

   // for PWM2
   // Setup TBCLK
   EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;           // Set timer period
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
   EPwm2Regs.CMPB = EPWM_DUTY;               // Set compare B value

   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
//   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
//   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM2A on event A, down count
   EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;           	// Clear PWM2A on event A, up count

   EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM2B on event B, up count
   EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM2B on event B, down count

   // for PWM3
   // Setup TBCLK
   EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;           // Set timer period
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm3Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
   EPwm3Regs.CMPB = EPWM_DUTY;               // Set Compare B value

   // Setup counter mode
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
//   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
//   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM3A on event A, down count
   EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM3A on event A, up count

   EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM3B on event B, up count
   EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM3B on event B, down count


   // for PWM4
   // Setup TBCLK
   EPwm4Regs.TBPRD = EPWM_TIMER_TBPRD/2-1;           // Set timer period
   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm4Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
   EPwm4Regs.CMPB = EPWM_DUTY;               // Set Compare B value

   // Setup counter mode
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up and down
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
//   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
//   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM3A on event A, down count
   EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;         // Clear PWM3A on event A, up count

   EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;           // Set PWM3B on event B, up count
   EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM3B on event B, down count\


   EPwm4Regs.ETSEL.bit.SOCAEN = 1;
   EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
   EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;

   // Set dead time
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm1Regs.DBFED = DEAD_TIME;
   EPwm1Regs.DBRED = DEAD_TIME;

   // Set dead time
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm2Regs.DBFED = DEAD_TIME;
   EPwm2Regs.DBRED = DEAD_TIME;

   // Set dead time
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm3Regs.DBFED = DEAD_TIME;
   EPwm3Regs.DBRED = DEAD_TIME;

}
