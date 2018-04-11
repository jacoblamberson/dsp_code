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
void Init_CAN(void);
void Init_CANMBoxes(void);

#pragma CODE_SECTION(ECAN_TX_TO, "ramfuncs");

struct ECAN_REGS ECanaShadow;




interrupt void ECAN_TX_TO()
{
//	EALLOW;
//	ECanaShadow.CANGIF1.all = ECanaRegs.CANGIF1.all;
	testshadow = ECanaRegs.CANGIF1.all;
//	EDIS;
	j=0;
	for (j=0;j<5000;j++){j++;}
	if (ECanaShadow.CANGIF1.bit.MTOF1 == 1)
//	if ((ECanaRegs.CANGIF1.all & 0x00020000) == 0x00020000)
	{
//TIME OUT!!
		ECanaShadow.CANGIF1.all = ECanaRegs.CANGIF1.all;
	}
	if (ECanaShadow.CANGIF1.bit.MTOF1 == 1)
	{
			    EPwm1Regs.CMPA.half.CMPA = 0000;    // Set up switch duty cycle
			    EPwm1Regs.CMPB = 10000;  // Set low switch duty cycle

			    EPwm2Regs.CMPA.half.CMPA = 0000;    // Set up switch duty cycle
			    EPwm2Regs.CMPB = 10000;  // Set low switch duty cycle

			    EPwm3Regs.CMPA.half.CMPA = 0000;    // Set up switch duty cycle
		    	EPwm3Regs.CMPB = 10000;  // Set low switch duty cycle

		    	asm ("      ESTOP0");
		    	  for(;;);
	}
	ECanaRegs.CANTA.bit.TA16 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}



void Init_CAN()
{

	InitECana(); // Initialize eCAN-A module
	EALLOW;
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
	EDIS;

	Init_CANMboxes();

	ECanaRegs.CANTRR.all = 0xFFFFFFFF;
	while(ECanaRegs.CANAA.all != 0xffffffff ) {}

	ECanaRegs.CANAA.all = 0xffffffff;

	 ECanaRegs.CANGIF0.all = 0x00013700;
	 ECanaRegs.CANGIF1.all = 0x00013700;

    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.

	    // Configure the eCAN for self test mode
    // Enable the enhanced features of the eCAN.
    EALLOW;
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.STM = 0;
    ECanaShadow.CANMC.bit.ABO = 1;
	ECanaShadow.CANMC.bit.SUSP = 1;
	ECanaShadow.CANMC.bit.MBCC = 1;
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    EDIS;

    ECanaRegs.CANTOC.all = 0x00010000;
    ECanaMOTORegs.MOTO16 = 1000;
    EALLOW;
    ECanaShadow.CANGIM.all = ECanaRegs.CANGIM.all;
    ECanaShadow.CANGIM.bit.EPIM = 1;
    ECanaShadow.CANGIM.bit.I0EN = 1;
    ECanaShadow.CANGIM.bit.I1EN = 1;
    ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;
    ECanaRegs.CANMIL.all = 0x00ff0000;
    ECanaRegs.CANMIM.all = 0x00ff0000;
	EDIS;
}
