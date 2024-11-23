#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // Number of receive attempts before giving up

 // Initalization of global variables
int stepCount = 0;
int motorOn = 0;
int home = 0;

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                           // activate clock for Port G
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};                  // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                          // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                         // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                            // enable digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01;                                         // disable analog functionality on PNG

    return;
}

void PortM_Init(void){
    //Use PortM pins (PM0-PM3) for output to control Stepper Motor
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                         // activate clock for Port M
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};                // allow time for clock to stabilize
    GPIO_PORTM_DIR_R |= 0x1F;                                         // configure Port M pins (PM0-PM3) as output
	GPIO_PORTM_AFSEL_R &= ~0x1F;                                        // disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x1F;                                           // enable digital I/O on Port M pins (PM0-PM3)
  GPIO_PORTM_AMSEL_R &= ~0x1F;                                        // disable analog functionality on Port M pins (PM0-PM3)
    return;
}

void PortN_Init(void){
    //Use PortM pins (PN0-PN1) for output for onboard LEDs
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                         // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};                // allow time for clock to stabilize
    GPIO_PORTN_DIR_R |= 0x03;                                         // configure Port N pins (PN0-PN1) as output
  GPIO_PORTN_AFSEL_R &= ~0x03;                                        // disable alt funct on Port N pins (PN0-PN1)
  GPIO_PORTN_DEN_R |= 0x03;                                           // enable digital I/O on Port N pins (PN0-PN1)
  GPIO_PORTN_AMSEL_R &= ~0x03;                                        // disable analog functionality on Port N pins (PN0-PN1)
    return;
}

void PortH_Init(void){
	//Use PortH pins (PH0-PH1) for input for pushbuttons
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                             // Activate the clock for Port H
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};                  // Allow time for clock to stabilize
		
	GPIO_PORTH_DIR_R = 0b00000000;       								                 // Enable PH0 and PH1 as inputs 
  GPIO_PORTH_DEN_R = 0b00001111;													           	 // Enable PH0 and PH1 as digital pins
	return;
}

void spin(){                                       // Full-step Stepping Method spin clockwise function
    uint32_t delay = 1;                            // 10ms delay between each step
                                                                       
        GPIO_PORTM_DATA_R = 0b00010011;
        SysTick_Wait10ms(delay);                                       
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00011001;
        SysTick_Wait10ms(delay);

				stepCount = (stepCount + 1);                // 'stepCount' is used to stop spinning motor when a measurement need to be taken
				home = (home + 1);                          // 'home' is used to keep track of displacement, used to spin CCW and return home 
}

void spinCCW(){                                    // This function is solely used when a full rotation CW is complete to spin CCW & untangle wires 
    uint32_t delay = 1;                            // 10ms delay between each step

    for(int i=0; i<512; i++){                      // One rotation has 2048 steps, so the upper bound is 512 since there are 4 phases
        GPIO_PORTM_DATA_R = 0b00011001;            // within the loop (2048/4=512)
        SysTick_Wait10ms(delay);                         
        GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00010011;
        SysTick_Wait10ms(delay);
    }
}

void goHome() {                                    // This is used to spin CCW back to home position when pushbutton is pressed to stop system
    uint32_t delay = 1;                            // 10ms delay between each step
	
    for(int i=0; i<home; i++){                     // The upper bound is the number of steps clockwise the stepper motor has already                           
        GPIO_PORTM_DATA_R = 0b00011001;            // taken so it goes in the CCW direction the same number of steps to reach the
        SysTick_Wait10ms(delay);                   // original/home position            
        GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00010011;
        SysTick_Wait10ms(delay);
    }
		home = 0;                                     // Resets home variable to 0 because stepper motor has returned home
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                     // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;              //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                    // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			                       //address of the ToF sensor as an I2C slave peripheral
int status=0;

void systemStart(void) {
	// Initialization of all variables that will catch respective information from the ToF sensor
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	// Sends serial data through UART to print text to screen
	UART_printf("Program Begins\r\n");
	sprintf(printf_buffer,"2DX ToF Program Code \r\n");
	UART_printf(printf_buffer);

  // Those basic I2C read functions can be used to check your own I2C functions
	status = VL53L1X_GetSensorId(dev, &wordData);                          // Gets model data from the ToF sensor
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);                                            // Sends serial data through UART to print text to screen

	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();                                                       // Tells environments that the ToF is booted
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");               // Prints that the ToF is booted to PC screen with UART
	status = VL53L1X_ClearInterrupt(dev);                                 // Clear interrupt has to be called to enable next interrupt

  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);                                    // Collect the status of the ToF sensor
	
	status = VL53L1_RdByte(dev, 0x010F, &byteData);                        //For collecting model ID & displaying on PC using UART
	sprintf(printf_buffer,"Model ID: %x\r\n", byteData);
	UART_printf(printf_buffer);
	
	status = VL53L1_RdByte(dev, 0x0110, &byteData);                        //For collecting module type & displaying on PC using UART
	sprintf(printf_buffer,"Module Type: %x\r\n", byteData);
	UART_printf(printf_buffer);
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);                        //For collecting both model ID and type
	sprintf(printf_buffer,"Model ID & Module Type: %x\r\n", wordData);
	UART_printf(printf_buffer);

    status = VL53L1X_StartRanging(dev);                                  // Function has to be called to enable the ranging

		while (stepCount <= 512)
		{
		while((GPIO_PORTH_DATA_R )==0b00000001) {                            // Checks whether the 'stop' button is pressed
			motorOn = 0;                                                       // Motor status is turned off
			stepCount = 0;                                                     // Resets step count
			return;                                                            // Re-enters main function
			while((GPIO_PORTH_DATA_R )==0b00000001) {}                         // Debouncing
		}
		spin();
		if (stepCount % 16 == 0)                                             // Continues to spin motor until a measurement point is reached
		{                                                                    // Executes everything within 'if statement' before continuing spin
		GPIO_PORTN_DATA_R |= 0b00000010;
		SysTick_Wait10ms(5);
		GPIO_PORTN_DATA_R &= 0b11111101;
			
	  while (dataReady == 0){                                              // Waits until the ToF sensor's data is ready
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		// Reads the data values from ToF sensor
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);			
		if (RangeStatus != 0) {                                              // If a surface is out of range, automatically set distance to 4m
			Distance = 4000;
		}
		// Continue to read the rest of the data values from the ToF sensor
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
	
	  status = VL53L1X_ClearInterrupt(dev);                                // 8 clear interrupt has to be called to enable next interrupt
		
		// Print the resulted readings to UART
		sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
  }
}
		spinCCW();                                                           // Spin CCW a full rotation once a full CW rotation is completed
  
	VL53L1X_StopRanging(dev);
 // Resets all tracker variables
	motorOn = 0;
	stepCount = 0;
	home = 0;
	return;                                                               // Returns to main function
}

int main(void) {
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortH_Init();
	PortN_Init();
	
	while(1) {                                                          // Infinite while loop runs within main function, polling is used
		while((GPIO_PORTH_DATA_R )==0b00000001) {                         // Turns Motor status off if 'stop' button is pressed
			motorOn = 0;
			while((GPIO_PORTH_DATA_R )==0b00000001) {}                      // Debouncing
		}
		while((GPIO_PORTH_DATA_R )==0b00000010) {                         // Turns Motor status on if 'start' button is pressed
			motorOn = 1;
			while((GPIO_PORTH_DATA_R )==0b00000010) {}                      // Debouncing
		}
		if(motorOn) {                                                     // Checks Motor status
			GPIO_PORTN_DATA_R |= 0b00000001;                                // If Motor status is on, PN0 (oboard LED) will go high
			systemStart();                                                  // If Motor status is on, the system will begin processes
		}                                                                 // to collect data from ToF sensor
		else {
			GPIO_PORTN_DATA_R &= 0b11111110;                                // If Motor status is off, PN0 (onboard LED) will go low
		}
		
		if(home>0) {                                                      // Whenever in the main function, we assume that the motor is off
			goHome();                                                       // Therefore, the motor will spin back to the home position if not already there
			home = 0;                                                       // Clears the home variable once the motor is back at home position
		}	
	}
}