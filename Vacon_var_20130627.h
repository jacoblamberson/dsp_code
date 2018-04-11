extern void sync_mot();

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

/**************Voltage Control Parameters**********/
/**PI on dq axis **/

#define KI_VS         5                //130VDC
#define KP_VS         0.016

//#define KI_VS         2                //300VDC
//#define KP_VS         0.0066

//#define KI_VS         1.5               //400VDC
//#define KP_VS         0.0048

//#define KI_VS         1                //600VDC
//#define KP_VS         0.0032
/**PR on 0 axis **/

#define kr_pr		  5
/**LPF **/
#define Taui           0.00016     //Current differential feedback LPF (1000Hz)
#define Tauv          0.00053      // Voltage feedback LPF
#define Taup          0.0159       //Active power LPF
#define Tau1          0.0159       //Dead time compensation LPF

/**************Current Control Parameters**********/
//#define KI_PLL            3200
//#define KP_PLL            180
#define KI_PLL            900
#define KP_PLL            53

//#define KI_VS         10 //4   //10
//#define KP_VS         0.005//16//6 //0.016  //0.0064
#define KI_IS         40
#define KP_IS         0.008

//#define Ki_i         35//15.1667//35@130V
//#define Kp_i         0.08//0.08//0.105//0.0455//0.105@130V
#define Ki_i         35//15.1667//35@130V
#define Kp_i         0.08//0.105//0.0455//0.105@130V


#define Tauw         0.0064 //for angle filtering
#define Tauc         0.0064//0.0032 //for DC voltage filtering 25Hz

//#define  Tau1           0.0003316
   
#define  Ts      0.0001


#define  EPWM_DUTY         3750
#define  ADC_MODCLK        0x3   // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#define  ADC_CKPS          0x1   // ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define  ADC_SHCLK         0xf   // S/H width in ADC module periods                        = 16 ADC clocks
#define  ADC_usDELAY       5000L
#define  PWM_PERIOD        0.0001     
#define  EPWM_TIMER_TBPRD  7500       //10kHz
#define  DEAD_TIME         300		//3us

#define  Lr                0.0005
#define  R                 0.02
#define  PI                3.1415926

/*
#define V_Rat       100         // Motor nameplate Volts(line-neutral peak)
#define V_Rat_Rcp   0.01       // Reciprocal of V_PU_Rat
#define I_Rat       20           // Motor nemeplate Amps(peak)
#define I_Rat_Rcp   0.05         // Reciprocal of I_PU_Rat
*/
#define Frq_Rat     376.9911     // Motor nameplate frequency(rad/sec)


extern double ki_tr_om;
extern double kp_tr_om;
extern double ki_tr_iq;
extern double kp_tr_iq;
extern double ki_tr_id;
extern double kp_tr_id;



/**Declare function outputs to global data **/
extern double theta;           /* rotor position              (rad)  */
extern double Da_Out;          /* Simulated Inverter output voltage phase a (volts) */
extern double Db_Out;          /* Simulated Inverter output voltage phase b (volts) */
extern double Dc_Out;          /* Simulated Inverter output voltage phase c (volts) */
	   
/** Declare local data ***/
extern double Spd_PU;            /* motor speed                                  (pu) */
extern double Vas;
extern double Vbs;
extern double Vcs;
extern double s_theta;
extern double c_theta;

extern double Vq_Fbk;
extern double Vd_Fbk;
extern double SG_K1;
extern double SG_K2;
extern double SG_K3;
extern double SG_K4;
extern double SG_K5;
extern double SG_K6;
extern double SG_K7;
extern double I0s;
extern double Iqs;
extern double Iq;
extern double Iq_pre;
extern double Ids;
extern double Id;
extern double Id_pre;
extern double Vqs;
extern double Vds;
extern double Vq;
extern double Vq_pre;
extern double Vds_Fbk_pre;
extern double Vqs_Fbk_pre;
extern double Vds_LPF;
extern double Vqs_LPF;
extern double Vds_LPF_pre;
extern double Vqs_LPF_pre;
extern double Vd;
extern double Vd_pre;
extern double Vt;
extern double Vt_pre;
extern double Vt_err;
extern double Vt_err_pre;
extern double Vfd;
extern double Vfd_pre;
extern double Pe;
extern double Omega;
extern double Dif_Omg;
extern double Dif_Omg_pre;
extern double Snd_Ctl;
extern double Gvn_in;
extern double Gvn_in_pre;
extern double Gvn_out;
extern double Gvn_out_pre;
extern double TB_out1;
extern double TB_out1_pre;
extern double TB_out2;
extern double TB_out2_pre;
extern double Inert_in;
extern double Inert_in_pre;
extern int TB_Loadstep;
extern double Fcn1_in;
extern double Fcn1_in_pre;
extern double Fcn1_out;
extern double Fcn1_out_pre;
extern double Fcn2_out;
extern double Fcn2_out_pre;
extern double PSS_out;
extern double PSS_out_pre;
extern double Error_Vd;
extern double Integra_Vd;
extern double Error_Vq;
extern double Integra_Vq;
extern double Error_I0;
extern double Integra_I0;
extern double D0_ref;
extern double Dd_temp;
extern double Dd_ref;
extern double Dq_temp;
extern double Dq_ref;
extern double Ids_LPF;
extern double Ids_LPF_pre;
extern double Iqs_LPF;
extern double Iqs_LPF_pre;
extern double Ids_FF;
extern double Iqs_FF;
extern double Ids_pre;
extern double Iqs_pre;
extern double ia_ref_dc;
extern double ib_ref_dc;
extern double ic_ref_dc;
extern double theta_pll;
extern double w_pll;
extern double w_LPF;
extern double Error_pll;
extern double It;
extern double Iqs_pu;
extern double Integra_pll;
extern double Iqs1_LPF1_pre;
extern double Iqs1_LPF1;
extern double Ids1;
extern double Iqs1;
extern double Iqs1_pre ;
extern double Ids1_LPF1_pre;
extern double Ids1_LPF1;
extern double Ids1_pre;
extern double Id_ref;
extern double Iq_ref;
extern double Iq_ZIP_ref;
extern double Id_ZIP_ref;
extern double Id_ZIP_LPF;
extern double Iq_ZIP_LPF;
extern double IMbase;
extern double Id_Motor_ref;
extern double Iq_Motor_ref;
extern double Iref;
extern double Pbase;
extern double Qbase;
extern double Pbasef;
extern double Qbasef;
extern double LPFc1;
extern double LPFc2;
extern double LPFv1;
extern double LPFv2;
extern double Vd_LPFc;
extern double Vq_LPFc;
extern double Vt_Fbk;
extern double Error_Id;
extern double Error_Iq;
extern double Integra_Id;
extern double Integra_Iq;
extern double Prop;
extern double Ctld_out;
extern double Ctlq_out;
extern double Ctld_open;
extern double Ctlq_open;


extern double t;

extern double Modulation_Index;
extern double CMP_a;
extern double CMP_b;
extern double CMP_c;

extern double testtest;
/**********************************common variables definination************************/
//extern double SampleTable7[BUF_SIZE];
extern double w;

////////////////////////////////////////////////////////////////////////
///////////////////ADC
///////////////////////////////////////////////////////////////////////

extern int AD_i,AD_flag;
extern int swcount;
extern double swdiv;
extern double AD_buffer[11][12];
extern int interrupt_count;
extern Uint16 ii, cycle_count;
extern double Average_AD1,Average_AD2,Average_AD3;

// IB5
extern double scale_ch0;          // CW: Only Iu, Iv, Iw are used in the program.
extern double scale_ch1;
extern double scale_ch2;
extern double scale_vdc;
extern double scale_vab;
extern double scale_vbc;
extern double scale_vca;
extern double scale_t1;
extern double scale_t2;
extern double scale_t3;

extern double offset_ch0;
extern double offset_ch1;
extern double offset_ch2;
extern double offset_vdc;
extern double offset_vab;
extern double offset_vbc;
extern double offset_vca;
extern double offset_t1;
extern double offset_t2;
extern double offset_t3;

extern double Ia,Ib,Ic;
extern double Vab,Vbc,Vca;
extern double Vdc;

// ADC start parameters
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz     //? Not used?
#define ADC_CKPS   0x1   // ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  0xf   // S/H width in ADC module periods                        = 16 ADC clocks
#define AVG        1000  // Average sample limit
#define ZOFFSET    0x00  // Average Zero offset
#define BUF_SIZE   1000  // Sample buffer size

//////////////////////////////////////////////////////////////
//////////////////CAN
/////////////////////////////////////////////////////////////
// Global variable for this example
extern Uint32  ErrorCount;
extern Uint32  PassCount;
extern Uint32  MessageReceivedCount;

extern Uint32  TestMbox1;
extern Uint32  TestMbox2;
extern Uint32  TestMbox3;
extern Uint32  TestMbox4;
extern Uint32  TestMbox5;
extern Uint32  TestMbox6;
extern Uint32  TestMbox7;
extern short income_mailbox;
extern volatile struct MBOX *Mailbox;
extern int MIV;

extern struct ECAN_REGS ECanaShadow;
extern unsigned long testshadow;

extern short Send_flag;
extern short sync_flag;


/////////////////////////////////////////////////////////////
/////////////////Data store
/////////////////////////////////////////////////////////////
extern double SampleTable1[BUF_SIZE];
extern double SampleTable2[BUF_SIZE];
extern double SampleTable3[BUF_SIZE];
extern double SampleTable4[BUF_SIZE];
extern double SampleTable5[BUF_SIZE];
extern double SampleTable6[BUF_SIZE];
extern int j, Sample_count, Save_flag;

/////////////////////////////////////////////////////////////
////////////////Synchro
/////////////////////////////////////////////////////////////
extern int temp_tsc;
extern int temp_ctr;
extern int prd_offset;
extern int prd;
