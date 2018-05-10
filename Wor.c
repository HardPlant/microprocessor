/*                    Wor_v4.c(channel 0)                                   *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F335)                                               *
 * Author:                  Young Min Kim                                                                       *
 *******************************************************************************************************
 * Revision history:     See end of file                                                               *
 *******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>
#include <RegSettings_Wor.h>
									  //Best !!!!!!!!!! Use with wor_v4(RX)

	//	   2channel(channel 0)	TX   Working Perfect!!!!!!!!!!!!!

//-------------------------------------------------------------------------------------------------------
//  DESCRIPTION:
//      This example is a simple implementation of a control system for a radio-controlled object.
//      The system is modeled as a simplex system with a transmitting remote 
//      control sending packets with control commands to the remotely controlled receiving object. 
//      WOR is used to reduce power consumption at the receiver side.
//      One evaluation board (EB) is set up as the remote control (TX unit), and the other as 
//      the remotely controlled object (RX/WOR unit). Data regarding movements of the joystick on 
//      the remote control are transmitted to the receiver and displayed on the LCD of this EB.
//      Please see App. Note 38 for more details.
//      
//      RX UNIT:
//          * Move joystick right to choose RX
//          * Push the S1 button to confirm your choice. 
//          After the unit has been configured, the yellow LED will be on.
//
//          GLED: Toggles when a packet has been received successfully.
//          RLED: Toggles when an invalid packet has been received. 
//          BLED: Toggles when a packet is received and an interrupt is given to the MCU.
//
//      TX UNIT:
//          * Move joystick left to choose TX
//          * Push the S1 button to confirm your choice. 
//          After the unit has been configured, the yellow LED will turn on.
//
//          BLED: Toggles when a packet has been sent.
//
//      If the TX unit is selected, you are now able to push the joystick button or move it up, 
//      down, left or right. This will start the packet burst. When all the packets (305) are transmitted
//      "Done" will be displayed at the LCD. One should not move the joystick to a new position before
//      the previous burst transmission has completed. 
//
//      If the RX unit is selected, it will automatically enter the WOR mode, waiting for packets. 
//
//      The LCD will continuously display how many valid packets have been received together with 
//      the position of the joystick on the remote control.
//      
//      The program can be terminated by pressing the S1 button after the setup has been done. 
//      This will turn off the yellow LED.
//
//      Current config. specs
//
//          Common Settings:
//              Packet format = 4B preamble + 4B sync word + 1B payload + 2B CRC = 11 Bytes
//
//          TX Unit Settings:
//              Packet interval:    1.0 ms
//              Packet send time:   ~ 11 B * 8 bit/B / 250 kbps = 352 us
//              Tx duty cycle:      ~ 352 us / 1.0 ms = 35.2 %
//
//          RX/WOR Unit Settings:
//              Event0 timeout:     300 ms
//              Event1 timeout:     ~ 1.4 ms (f_xosc = 26 MHz)
//              Rx timeout:         ~ 1.172 ms
//              Rx duty cycle:      ~ 1.172 ms / 300 ms = 0.391 %
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Defines


//-------------------------------------------------------------------------------------------------------

#define WTE_            0x40   //watchdog timer enable






                         	

//-------------------------------------------------------------------------------------------------------
// Defines

//#define GDO0_            P0_6
   
//-------------------------------------------------------------------------------------------------------
//#define GDO2_        P0_7     


//-------------------------------------------------------------------------------------------------------
// Global Variables

//char code  first_entered           _at_ 0xDF0;		
//char  xdata * idata ptr_first_entered=&first_entered	;	
//
//// // char code  lock_byte          		_at_ 0x0FFE;
//char code  lock_byte          		_at_ 0x0FFF ;
// char  xdata * idata ptr_lock_byte=&lock_byte	;
////BYTE code save1=0x03;
//BYTE code channel_no=0x03;
// BYTE code save0=0x34;
////BYTE code save2=0x01;
//  char temp_first;
//char temp33=0x33;
////UINT32 count= 0;
////UINT8 j=0;
////UINT32 packetsReceived = 0;
////char xdata * data pwrite;           // FLASH write pointer
// 	//   char code * data pread;             // FLASH read pointer
//  // unsigned char byte;
//
//// TX & RX FIFO buffer arrays
//BYTE txBuffer[3];	//varible pkt length
//
////BYTE xdata txBuffer[6];             // Fixed packet length: 1B payload
////BYTE rxBuffer[3];             // Max received packet length = 1B (All packets longer then 1 byte 
//                                    // will not be put in the RX FIFO due to packet length filtering 
////BYTE rxBuffer1[3]; 
//
//   
//BOOL packet_detected=FALSE;
//
////UINT8 save0=0x36;
////UINT8 save1=0x56;
//BYTE lock_byte_data =0xF9;
//UINT8 i=0;
//BOOL calibration_needed =FALSE;
//UINT8 temp;
//UINT8 temp1;
//UINT8 temp2;
//UINT8 state;
//BOOL external_interrupt=FALSE;
//BOOL toggle=FALSE;
//BOOL tx_bell=FALSE;
//UINT8 temp=0;
//BOOL switch_value=TRUE;
//UINT8 init_cnt=0;
//BOOL init=TRUE;
//BOOL switch_value1=FALSE;
//BOOL switch_value2=FALSE;
//BOOL switch_value3=FALSE;
//UINT8  sw_count=0;
//BYTE temp2;
//UINT8 temp1;
//BOOL done=FALSE;
//BOOL 	first=TRUE;
//BOOL idle1=FALSE;
////--------------------
////-----------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------------------
// Function declarations
//void initTransmitter(void);
void runTransmitter(void);
//void FLASH_ByteWrite(FLADDR addr, char byte);
//void initReceiver(void);
void runReceiver(void);
//BOOL ReceivePacket(BYTE *rxBuffer, UINT8 *length);
void  SendPacket(BYTE *txBuffer, UINT8 size) ;
void POWER_UP_RESET1_CCxxx0() ;
void RESET1_CCxxx0() ;
BYTE halSpiReadStatus(BYTE addr) ;
void halSpiStrobe(BYTE strobe) ;
  void halSpiWriteReg(BYTE addr, BYTE value) ;
 void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count) ;
//-------------------------------------------------------------------------------------------------------


//Letter Tables
UINT8 table_blank[] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0};
int table_A[] = {0,1,1,1,1, 1,0,1,0,0, 0,1,1,1,1};
int table_B[] = {1,1,1,1,1, 1,0,1,0,1, 0,1,0,1,0};
int table_C[] = {0,1,1,1,0, 1,0,0,0,1, 1,0,0,0,1};
int table_D[] = {1,1,1,1,1, 1,0,0,0,1, 0,1,1,1,0};
int table_E[] = {1,1,1,1,1, 1,0,1,0,1, 1,0,1,0,1};
int table_F[] = {1,1,1,1,1, 1,0,1,0,0, 1,0,1,0,0};
int table_G[] = {0,1,1,1,0, 1,0,1,0,1, 0,0,1,1,0};
int table_H[] = {1,1,1,1,1, 0,0,1,0,0, 1,1,1,1,1};
int table_I[] = {0,0,0,0,1, 1,0,1,1,1, 0,0,0,0,1};
int table_J[] = {1,0,0,0,0, 1,0,0,0,1, 1,1,1,1,1};
int table_K[] = {1,1,1,1,1, 0,0,1,0,0, 0,1,0,1,1};
int table_L[] = {1,1,1,1,1, 0,0,0,0,1, 0,0,0,0,1};
int table_M[] = {1,1,1,1,1, 0,1,1,0,0, 0,1,1,1,1};
int table_N[] = {1,1,1,1,1, 1,0,0,0,0, 0,1,1,1,1};
int table_O[] = {0,1,1,1,0, 1,0,0,0,1, 0,1,1,1,0};
int table_P[] = {1,1,1,1,1, 1,0,1,0,0, 0,1,0,0,0};
int table_Q[] = {0,1,1,1,1, 1,0,0,1,1, 0,1,1,1,1};
int table_R[] = {1,1,1,1,1, 1,0,1,0,0, 0,1,0,1,1};
int table_S[] = {0,1,0,0,1, 1,0,1,0,1, 1,0,0,1,0};
int table_T[] = {1,0,0,0,0, 1,1,1,1,1, 1,0,0,0,0};
int table_U[] = {1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1};
int table_V[] = {1,1,1,1,0, 0,0,0,0,1, 1,1,1,1,0};
int table_W[] = {1,1,1,1,0, 0,0,1,1,0, 1,1,1,1,0};
int table_X[] = {1,1,0,1,1, 0,0,1,0,0, 1,1,0,1,1};
int table_Y[] = {1,1,0,0,0, 0,0,1,0,0, 1,1,1,1,1};
int table_Z[] = {1,0,0,1,1, 1,0,1,0,1, 1,1,0,0,1};
int table_1[];
int table_2[];
int table_3[];
int table_4[];
int table_5[];
int table_six[];
int table_seven[];
int table_eight[];
int table_nine[];
int table_zero[];

void delay(UINT8 delay){ // 8bit, 2^8  = 256 * Clock Time
	while(delay--);
}

UINT8 dot_time = 3;

void setup(){
//크로스바 활성화
	XBR0 = 0x40;
	XBR1 = 0x40;
//포트 설정 초기화	  
	P0MDIN	= 0xFF;	
	P1MDIN	= 0xFF;	
	P0MDIN	= 0x00;	
	P1MDIN	= 0x00;	

	P0MDOUT = 0xFF;
	P1MDOUT = 0xFF;

	P0SKIP = 0x00;		
	P1SKIP = 0x00;		
//포트 조작	
	P1 = 0x02; // 0000 0010	
}

/* print_letter
 * P0을 직접 조작하는 함수
 * current_status : table로 가는 포인터
 * phase : table에서 몇 번째로 가는지
 */
void print_letter(int* current_status, int phase){
	P0 = current_status[phase];
}

int temp_phase=0;
int* status_table[12]={table_A,table_B}; //,table_A,table_A,table_A,table_A,table_A,table_A,table_A,table_A,table_A,table_A,};

/* get_current_status
 * current_status가 시퀸셜에 따라 한 letter table을 가리키도록 하는 함수
 * current_status : 테이블로 가는 포인터
 * (status_table) : 시퀀스들을 나열해놓은 테이블로 가는 포인터의 배열
 * (temp_phase)	: 현재 차례(시퀀스)
 */
void get_current_status(int** current_status){

	if(temp_phase==12) temp_phase=0;
	phase = status_table[temp_phase];
}
/* loop
 * 메인 루프, current_status()를 갱신 후 phase(3단계)를 따라 print_letter()를 호출
 *
 *
 */
void loop(){
	int* current_status;
	int phase;
	while (1){
		get_current_status(&current_status);
		for(phase=0;phase<3;phase++){
			print_letter(current_status,phase);
			delay(dot_time);
		}
	}
}


//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This function takes care of all the MCU initialization and the radio settings which is common
//      for both the receiver and the transmitter.
//-------------------------------------------------------------------------------------------------------
void main (void) {
 

    #ifdef STAND_ALONE

        // Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
								OSCICN = 0x83;
			CLKSEL=0x00;	
			OSCLCN = 0x00; 
	
	
    #endif
	setup();
	loop();	
}
//
//	XBR0 = BM_SPI0E | BM_URT0E;
//
//    XBR1= BM_XBAR ;
//
//    P0SKIP = GDO0_ | GDO2_;	 // *************     //define 부분도 추가
////    P1SKIP = 0x02; 
//
//
//	// watchdog timer disable
//    PCA0MD &= (~WTE_); 
//
//// 		  while(1);
//	   	
//
//	 	
////	 RSTSRC &=0xEF;		//software reset
//			halSetupTimer01(TIMER_1, SYSCLK_DIV_1, MODE_1, INT_OFF);
// 			halSetupTimer01(TIMER_0, SYSCLK_DIV_1, MODE_1, INT_OFF);
//	//			halSetupTimer23(TIMER_2, SYSCLK_DIV_1, MODE_1, INT_OFF);
//	//		   	halSetupTimer23(TIMER_3, SYSCLK_DIV_1, MODE_1, INT_OFF);
//				TMR2H=0x00;
//			TMR2L=0xEC;
//			TR2=0;
//    
//
//	
//
//			  
//	
// 	P0MDIN=0xFF;
//	P1MDIN=0x00;
//	P0MDOUT =SCLK_ | SI_ | CSn_;
//	P1MDOUT   = 0x00;
//
//
//
//
//        
//	SPI0CFG = BM_MSTEN; 
//    SPI0CN = BM_NSSMD1;
//    SPI0CKR   = 0x05; 
//  // P0_1=1;
//    SPI_ENABLE(); 
//
//
//
//    // Reset CC2500 and write RF settings to config registers
// //   POWER_UP_RESET_CCxxx0();
//		  
//
//	 POWER_UP_RESET_CCxxx0();
//
//    // Write register settings
//    halSpiWriteReg(CCxxx0_FSCTRL1,  0x07);
//    halSpiWriteReg(CCxxx0_FSCTRL0,  0x00);
//  //  halSpiWriteReg(CCxxx0_FREQ2,    0x5C);
//  //  halSpiWriteReg(CCxxx0_FREQ1,    0xC4);
//  //  halSpiWriteReg(CCxxx0_FREQ0,    0xEC);
//          halSpiWriteReg(CCxxx0_FREQ2,    0x5D);
//    halSpiWriteReg(CCxxx0_FREQ1,    0x31);
//    halSpiWriteReg(CCxxx0_FREQ0,    0x3B);
//   //    halSpiWriteReg(CCxxx0_FREQ2,    0x5D);
//  //  halSpiWriteReg(CCxxx0_FREQ1,    0x31);
//  //  halSpiWriteReg(CCxxx0_FREQ0,    0x3B);
//
// //   halSpiWriteReg(CCxxx0_MDMCFG4,  0x2D);
//    halSpiWriteReg(CCxxx0_MDMCFG4,  0x2D);
// //   halSpiWriteReg(CCxxx0_MDMCFG3,  0x6B);
//	  halSpiWriteReg(CCxxx0_MDMCFG3,  0x3B);
////    halSpiWriteReg(CCxxx0_MDMCFG2,  0x73);
//	halSpiWriteReg(CCxxx0_MDMCFG2,  0x73);
//    halSpiWriteReg(CCxxx0_DEVIATN,  0x00);
//    halSpiWriteReg(CCxxx0_FREND1,   0xB6);
//	halSpiWriteReg(CCxxx0_FREND0,   0x10);
// //   halSpiWriteReg(CCxxx0_MCSM1 ,   0x01);
//    halSpiWriteReg(CCxxx0_MCSM0 ,   0x08);			  //
//    halSpiWriteReg(CCxxx0_FOCCFG,   0x1D);
//    halSpiWriteReg(CCxxx0_BSCFG,    0x1C);
//    halSpiWriteReg(CCxxx0_AGCCTRL2, 0xC7);
//	halSpiWriteReg(CCxxx0_AGCCTRL1, 0x00);
//   halSpiWriteReg(CCxxx0_AGCCTRL0, 0xB2);
//    halSpiWriteReg(CCxxx0_FSCAL3,   0xEA);
//	halSpiWriteReg(CCxxx0_FSCAL1,   0x00);
//    halSpiWriteReg(CCxxx0_FSCAL0,   0x11);
//      halSpiWriteReg(CCxxx0_IOCFG2,   0x06);
//    halSpiWriteReg(CCxxx0_IOCFG0,   0x06);    
//
//   halSpiWriteReg(CCxxx0_CHANNR,   channel_no); 
//    halSpiWriteReg(CCxxx0_PATABLE, 0xC0);
//      halSpiWriteReg(CCxxx0_PATABLE, 0xFF);
//					 halSpiStrobe(CCxxx0_SIDLE);
//			 halSpiStrobe(CCxxx0_SPWD);
//
//		   P0MDIN=0xFF;
//		    P1MDIN=0x00;
//    	P1=0x00;
//	
//	  P0=0x48;
//	 
//	  	
//	P1MDOUT=0x00;
//	P0MDOUT=0x00;
//	CR=0;	
//
//
//		  				ENABLE_GLOBAL_INT(INT_ON);
//					       		SETUP_GDO2_INT(EDGE, LOW);                  // Enables external interrupt on low edge
//    		INT_SETFLAG(INUM_EXTERNAL1, INT_CLR);       // Clears the interrupt flag
//			INT_ENABLE(INUM_EXTERNAL1, INT_ON); 
//
//	//	  while(1);
//	   /*  	 
//   		temp_first= first_entered;
//			  
//
//		if(temp_first!= temp33  ){
//				OSCLCN = 0x80;
//			CLKSEL=0x02;
// 			OSCICN = 0x00;
//	
//	   	PCON =PCON | 0x01;	 
//		PCON =PCON | 0x01;
//
//	  	
//				  
//	
//	
//				halWait(100);
//			   	PSCTL |= 0x01;
//				PSCTL &= 0xFD;
//				FLKEY = 0xA5;
//				FLKEY= 0xF1;
//				*ptr_first_entered =0x33 ;		 //Not first entered
//				PSCTL &= 0xFE;
//
//	
//
//	
//	
//
//			 
//		
//				halWait(100);
//			   	PSCTL |= 0x01;
//				PSCTL &= 0xFD;
//				FLKEY = 0xA5;
//				FLKEY= 0xF1;
//			
//				*ptr_lock_byte =0xF9 ;		 //Not first entered
//			
//				PSCTL &= 0xFE;
//				
//	 }
//	 */
//
////	 ENABLE_GLOBAL_INT(INT_OFF);
//
//
//		 	P0MDIN=0xFF;
//	P1MDIN=0x00;
//	P0MDOUT =SCLK_ | SI_ | CSn_;
//	P1MDOUT   = 0x00;
//
//	   	INT_ENABLE(INUM_TIMER1, INT_ON); 
//	INT_ENABLE(INUM_TIMER0, INT_ON); 
//			 
//
//			 
//
//			
//			
//    // Calibrating frequency synthesizer (FS) at startup
//    halSpiStrobe(CCxxx0_SCAL);
////	RSTSRC |=0x04;	   //missing clock detector reset enable
//		
//
//	    halSpiWriteReg(CCxxx0_PKTCTRL1, 0x08);
//    	halSpiWriteReg(CCxxx0_PKTCTRL0, 0x44);
//		  halSpiWriteReg(CCxxx0_PKTLEN,   0x03);
//		 		   	halSpiWriteReg(CCxxx0_SYNC0,save0);		  //make 2 Byte ID code	 as 2 Sync Bytes
//    		halSpiWriteReg(CCxxx0_SYNC1,save0);
//
//   			  		INT_ENABLE(INUM_TIMER1, INT_OFF); 
//	 	TR1=0;
//					 			  
//					TR2=0;
//		
//		   INT_ENABLE(INUM_TIMER1, INT_ON); 
//
//
//
//					 	INT_ENABLE(INUM_EXTERNAL1, INT_OFF);  
//	//	  ENABLE_GLOBAL_INT(INT_OFF);
//	 L1:	
//
//			P0MDIN=0xFF;
//		   	P1MDIN=0xFF;
//		   	P0MDOUT =SCLK_ | SI_ | CSn_;
//
//		  			
//	  				
//				P1=0x00F;	
//			  P0 = 0xFF;
////		   	P1MDOUT=0xFF;
////		P1SKIP=0xFF;
////		 P1=0x10;
//			   SPI0CKR   = 0x01; 
//
//			OSCLCN = 0x80;
//			CLKSEL=0x02;	
//			OSCICN = 0x00;
//
//				OSCICN = 0x83;
//			CLKSEL=0x00;	
//			OSCLCN = 0x00; 
//			/*
//	NSSMD0 = 0;
//
//    while (P0_1);
//    SPI0DAT =CCxxx0_SIDLE;
//
//    SPI_WAIT();
//    NSSMD0 = 1;
//	*/
//			 halSpiStrobe(CCxxx0_SIDLE);
//
//			  
//
//	  
//		  	
//
//	
//	
//
//
//
//	
//	 
//
//
////	 halSpiStrobe(CCxxx0_SFTX);
////			while(1);
//
//      halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer,1);
//
//	 
//
//	 //  	 halSpiStrobe(CCxxx0_SCAL);		 //to supply steady TX output power
//	//	 	halSpiStrobe(CCxxx0_SIDLE);
//	/*
//		    NSSMD0 = 0;
//
//    while (P0_1);
//    SPI0DAT = CCxxx0_STX;
//    SPI_WAIT();
//    NSSMD0 = 1;
//	*/
//	   halSpiStrobe(CCxxx0_STX);
////	  	  ENABLE_GLOBAL_INT(INT_ON);
//
//	   	INT_ENABLE(INUM_EXTERNAL1, INT_ON); 
//		 
//	//	   	temp2=halSpiReadReg(CCxxx0_SYNC0);
//	//		 while(1);	
//
//			OSCLCN = 0x83;
//			CLKSEL=0x02;
// 			OSCICN = 0x00;			  
//	 //  	  	halSpiStrobe(CCxxx0_SIDLE);
//	//	  while(!packet_detected);
// 			/*
//		   	OSCICN = 0x83;
//			CLKSEL=0x00;
// 			OSCLCN = 0x00;
//
// 	 	   */
// 
//
//		 //  	TH1 = 0xB9;						//900 ms	Interval
//    	//	TL1 = 0xB0;
//
//			
//			   INT_ENABLE(INUM_TIMER1, INT_ON); 
//			INT_ENABLE(INUM_EXTERNAL1, INT_OFF);  
//	//	  ENABLE_GLOBAL_INT(INT_OFF);
//
//	//		  ENABLE_GLOBAL_INT(INT_ON);
//		//			if( sw_count < 3 &&	 TMR2H > 0x50 ) sw_count=0;
//				
//						   				   	//	   	P1MDOUT=0xFF;
//	//	P1SKIP=0xFF;
//	//	 P1&=~0x10;
//
//						  external_interrupt=FALSE;
//					
//
//
//					TH1 = 0xE0;						//900 ms	Interval
//    				TL1 = 0xB0; 
//			
//		
//					TR0=0;
//							halSpiStrobe(CCxxx0_SIDLE);
//	//				temp1=halSpiReadStatus(CCxxx0_MARCSTATE);
//	
//         	halSpiStrobe(CCxxx0_SPWD);
//					  					P1MDOUT=0x00;
//					P0MDOUT=0x00;
//					
//					P0MDIN=0xFF;
//						P1MDIN=0x00;	
//		
//				   	P0=0x48;
//				  	P1SKIP=0x00;	  
//					P1=0x00;
//
//			//		}
//					CR=0;	
//	
//		//	NSSMD0=1;
//			OSCLCN = 0x80;
//			CLKSEL=0x02;
// 			OSCICN = 0x00;
//		
//				 //  	if( external_interrupt)			 TMR3CN |=0x04;	
//
//
//
//	//		TR2=1;
//			TR1=1;
//		 	//	INT_ENABLE(INUM_EXTERNAL1, INT_ON);  
//
//				PCON =PCON | 0x01;
//
//		
//
//		 
//				goto L1;
// //   }
//}// Main
//
//
//
//
//
// 
////-------------------------------------------------------------------------------------------------------
////  void TIMER1_ISR(void)
////
////  DESCRIPTION:
////      This interrupt occurs every 1000 us on the transmitter unit. The timer
////      registers are reloaded and a packet is sent.
////-------------------------------------------------------------------------------------------------------
//void TIMER1_ISR(void) interrupt INUM_TIMER1 {
//
//	
//		 	PCON =PCON & 0xFE;
//		//	RSTSRC |=0x10;
//}// TIMER1_ISR
///*
//void TIMER2_ISR(void) interrupt INUM_TIMER2{
//
//	
//		 //	PCON =PCON & 0xFE;
//		//	if( sw_count < 3 &&	 TMR2H > 0x30 ) sw_count=0;
//
//}// TIMER1_ISR
// */ 
//
////-------------------------------------------------------------------------------------------------------
////  void EXTERNAL1_ISR(void)
////
////  DESCRIPTION:
////      This interrupt service routine occurs whenever the radio detects a packet using Wake-on-Radio
////-------------------------------------------------------------------------------------------------------
//void EXTERNAL1_ISR(void) interrupt INUM_EXTERNAL1{
//   
//	packet_detected=TRUE;
//
//
//    INT_SETFLAG(INUM_EXTERNAL1, INT_CLR);   // Clear interrupt flag
// 
//
//
//}// EXTERNAL1_ISR
//
//
//  /*
//void FLASH_ByteWrite (FLADDR addr, char byte)
//{
//   bit EA_SAVE = EA;                   // preserve EA
//   char xdata * data pwrite;           // FLASH write pointer
//
//   EA = 0;                             // disable interrupts
//
//   // change clock speed to slow, then restore later
//
//   VDM0CN = 0x80;                      // enable VDD monitor
//
//
//   RSTSRC = 0x02;                      // enable VDD monitor as a reset source
//
//   pwrite = (char xdata *) addr;
//
//   FLKEY  = 0xA5;                      // Key Sequence 1
//   FLKEY  = 0xF1;                      // Key Sequence 2
//   PSCTL |= 0x01;                      // PSWE = 1
//
//
//   VDM0CN = 0x80;                      // enable VDD monitor
//
//
//   RSTSRC = 0x02;                      // enable VDD monitor as a reset source
//
//   *pwrite = byte;                     // write the byte
//
//   PSCTL &= ~0x01;                     // PSWE = 0
//
//   EA = EA_SAVE;                       // restore interrupts
//}
//  */
//  /*
//void EXTERNAL0_ISR(void) interrupt INUM_EXTERNAL0{  
//	ENABLE_GLOBAL_INT(INT_OFF);
//	PCON =PCON & 0xFE;		 //in the middle of waiting timer interrupt 
////	TR1=0;
//			//  	external_interrupt=TRUE;
//
//					OSCLCN=0x80;  //10KHz
//				CLKSEL=0x02;
//				OSCICN=0x00;  //10KHz
//				 
//					 TMR3CN |=0x04;	
//
//	 	P1=0xFF;
//	   P0MDIN=0xFF;
//	  if(P0_6)	 P0=0xFF;
//	  P1MDIN=0xFF;
//	  CR=1;
//		//	TR0=0;
//			P1MDOUT   = 0x00;
//		
//		
//			   PCA0MD |= 0x08; 	   //time base is sysclk =10KHz
//				PCA0CPH0 =2;
//
//
//
//	  
//		
//			
//	 external_interrupt=TRUE;	
//
//   		first=TRUE;
//			
//	 
//	 	halWait(5);	// 24 ms
//
//		 	   
//	
//  sw_count++;
//
//	  	 	if(sw_count==1){
//	
//			TMR3H=0x01;
//			TMR3L=0xEC;
//						  // normal timer mode1 as in timer1
//				//	INT_ENABLE(INUM_TIMER3, INT_ON); 
//				
//
//			 }
//		
//	   temp1= TMR3H;
//
//	   if(temp1>0xFE) temp1=0xFE;
//	
//				 if(  temp1 > 0x10 ){
//			
//			TMR3H=0x01;
//			TMR3L=0xEC;
//		   	if(sw_count==2){
//			  sw_count=1;
//
//
//			}
//			else if	 (sw_count==3){
//				 sw_count=1;
//
//			}
//		}	  
//
//		 
//		if(sw_count ==3 ){
//				 
//				 sw_count=0;
//
//				
//			   	if(  temp1 < 0x10 ){
//					 halWait(400);
//		  			 
//			    	P1MDOUT   = 0x03;
//					CR=1;
//					halWait(50);
//					P1MDOUT   = 0x00;
//					halWait(50);
//					P1MDOUT   = 0x03;
//					halWait(50);
//					P1MDOUT   = 0x00;
//				    CR=0;
//
//
//		//	 if(packet_detected){							// short pressing	
//
//										   // normal state
//					P1=0xFF;
//				
//				
//
//					idle1=TRUE;
//
//			
//	
//		
//	   	}			//timer
//	 
//
//		
//	}	//  sw_count=3;
//
//
////	INT_ENABLE(INUM_EXTERNAL0, INT_OFF); 
//
//	//	TR1=1;	
////	}	
//	INT_SETFLAG(INUM_EXTERNAL0, INT_CLR);   // Clear interrupt flag
//
//	ENABLE_GLOBAL_INT(INT_ON);	
//		
//
//}// EXTERNAL0_ISR
//
//  */
