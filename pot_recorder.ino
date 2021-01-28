#define debug
//#define setRange_noCallibrate	8000 //25000

//#define print_file_after_recorded
#define LCD_active_only_on_input // if not LCD also will be active while play and rec  
#define use_play_shift //adds input slider to playback data
//#define LCD_active_always
#define LCD_upd_dt 100 //frame time
#define nextLCD_off_il_DT 20 //number of LCD updates, when LCD stay active after set nextLCD_off_il. Each upd cause nextLCD_off_il--;


#define acc_settings_M_M	35000
uint32_t acc_settings_M=35000;
uint32_t spd_us_setings_m=50;

//#define tstfast
//#define save_mem 

//#define testMotor
#define useFastAccelStepper
#define useUS1 //for fast control w hand

#define useESP32
#ifdef useESP32
#define btn_limit_near_p			12//2
#define btn_limit_far_p				14//4
#define pot_pos_shift_p				34 // has no internal pull up/dn : GPIO 34, 35, 36, 39 ? 34-39 
#define pot_precission_playSpd_p	35

#define btns_p						32 //32 pull UP seems cant work w analogRead //Using analogRead attaches the GPIO pin you're using to the ADC channel and disconnects it from the pull-up. 

/*
E	B
D	Gr
S	R
*/
//TODO fix upload noise on ESP32
//F no ansver https://electronics.stackexchange.com/questions/542899/what-are-the-most-reliable-pins-for-stepper-driver-enable-step-dir-for-esp32
#define enable_p					13//2//12 
#define dir_p						15//13//15 //TODO to 13
#define step_p						2//4//14 //  14,15 seems not good (at start PWM)

#define CSchipSelect_p				5	//SD CardReader SPI cs ss=5, mo=23, mi=19, sck=18

#define US0_t_p						33
#define US0_e_p						25
#define US1_t_p						26
#define US1_e_p						27

#define	SKIP_NONE	//TODO fix when SKIP_NONE available on ESP32

char filename[] = "/d0.txt";
char filename_rec_time[] = "/d0rec_time.txt"; //!opt2 concat
#else
#define btn_limit_far_p				A3	//on CNC shield: limit sw Y+ or X-
#define pot_pos_shift_p			A0	//INPUT_PULLUP 		//on CNC shield: abort
#define pot_precission_playSpd_p	A1	//INPUT_PULLUP //on CNC shield: hold
#define btns_p						A2	//4 btns to GDN w 10k resistors between btns
#define enable_p					8  	//motor shield //The enable pin [pin 8] should be set LOW to enable the steppers. //on CNC shield: EN (top left pin)
#define dir_p						5	//motor shield  // 5 on CNC shield v3 for 1st socket
#define CSchipSelect_p				10	//SD CardReader SPI cs ss=10, mo=11, mi=12, sck=13
char filename[] = "dd0.txt";
char filename_rec_time[] = "dd0rec_time.txt"; 
#endif

#include <EEPROM.h>

#define tab	Serial.print("\t");
//----------------------- SD  do not connect to 5v or test if 3.3v real voltage. Do not work if 3.6v or 32kb cluster (need test again to exclude).
bool bSD=true; //also if card err = fallback to false = use EEPROM
//#define useSdFat
#ifdef useSdFat
#include <SdFat.h>
#else
#include <SD.h>	
// file name to use for writing
File txtFile; // File object to represent file
String buffer; // string to buffer output


#endif

unsigned long lastMillis = 0;
//------------------------ 
//IIC ESP32 SDA=21 SCK=22
#define useOLED
#ifdef useOLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <splash.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

//for an SSD1306 display connected to I2C (SDA, SCL pins)
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1	//4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
//------------------------

//1.8 degrees (200 steps per revolution), library defines speed as steps per second


#ifdef useFastAccelStepper
//https://github.com/gin66/FastAccelStepper/blob/master/src/FastAccelStepper.h
#ifndef useESP32
#define step_p						9  // ! 9 or 10 for FastAccelStepper
#define btn_limit_near_p			2 	//on CNC shield: coolEN 
#define US0_t_p						3
#define US0_e_p						4
#define US1_t_p						6
#define US1_e_p						7
#define Speed_MAX	50000 //##
#endif

#include "FastAccelStepper.h"
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
#define Speed_callib	500
#define Speed_callib_M	77
#else
#define step_p						2	// 2 on CNC shield v3 for 1st socket
#define btn_limit_near_p			9 	//on CNC shield: coolEN 
#define US0_t_p						3
#define US0_e_p						6
#define US1_t_p						4
#define US1_e_p						7
#define Speed_MAX	3999
#include <AccelStepper.h>
#define motorInterfaceType 1 // must be set to 1 when using a driver
AccelStepper stepper = AccelStepper(motorInterfaceType, step_p, dirPin);

#include "TimerOne.h"

#define Speed_callib	999
#define Speed_callib_M	3999
#endif

//------------------------US
#include <NewPing.h>
#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 50 // Maximum distance (in cm) to ping.
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
	NewPing(US0_t_p, US0_e_p, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
	NewPing(US1_t_p, US1_e_p, MAX_DISTANCE), 
};

volatile int US_pos_v=-1;
volatile int US_input_v=-1;
volatile uint16_t US_input_v_good_i=0;
		
TaskHandle_t US_Read_Task;
void US_Read_Task_f( void * pvParameters ){
  Serial.print("US_Read_Task_f running on core ");
  Serial.println(xPortGetCoreID());

long v0_last_read_t=0;
long v1_last_read_t=0;

  //ping	ping_cm
  for(;;){
	  //while(bRec) delay(1000); //TODO tst if volatile need
	  
	  int v0=sonar[0].ping();
	  if(v0>600 && v0<4000)
	  {
		  if(millis()>v0_last_read_t+1000)
				US_pos_v=v0;
		  else
				US_pos_v=US_pos_v*0.5+v0*0.5;
			v0_last_read_t=millis();
	  }
	  //else US_pos_v =-1;
	  delay(40); // Wait 50ms between pings (about 20 pings/sec). 29ms@2m should be the shortest delay between pings.
	
	  int v1=sonar[1].ping();
	  if(v1>600 && v1<3000)
	  {
		   if(millis()>v1_last_read_t+1000)
				US_input_v=v1;
		  else
				US_input_v=US_input_v*0.5+v1*0.5;
			v1_last_read_t=millis();
		 US_input_v_good_i++;
	  }
	  else
	  {
		  US_input_v =-1;
		  US_input_v_good_i=0;
	  }
	  delay(40);
  } 
}

int us0_m=-1; //range
int us0_M=-1;
int stepper_m, stepper_M;

bool bDir=true;
int us0_range,stepper_range;

bool bbtn_limit_near, bbtn_limit_far;

bool bPlayCycling=false;

uint32_t rec_time=0;
uint32_t rec_time_in_slot_file;
uint32_t rec_time_last=0;
uint32_t rec_time_loaded=0;
uint16_t record_dataN=0;
long t_last=0;

long sw_last_t=0;
long nextWait_t=10000;
//============================================================================================================================
void setup() {
	delay(100);
  #ifdef debug
  Serial.begin(115200);
  #endif
  
  pinMode(enable_p, OUTPUT);
  pinMode(btn_limit_near_p, INPUT_PULLUP);
  pinMode(btn_limit_far_p, INPUT_PULLUP);
  
  //pinMode(pot_pos_shift_p, INPUT_PULLUP);
  //pinMode(pot_precission_playSpd_p, INPUT_PULLUP);
  #ifdef useESP32
   			WiFi.mode( WIFI_OFF );
			WiFi.forceSleepBegin();
  #else
	   pinMode(btns_p, INPUT_PULLUP);
  #endif

#ifdef useOLED
	if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
		#ifndef save_mem
		Serial.println(F("oled alloc fail"));
		#endif
	}
	#ifndef save_mem
	display.clearDisplay(); display.display();
	#endif
#endif

if(bSD)
{
#ifdef useSdFat
#else
  
  #ifdef useESP32
	buffer.reserve(1024);
  #else
	buffer.reserve(512); //!! 1024 // reserve some bytes for String used as a buffer
  #endif
  
  if (!SD.begin(CSchipSelect_p)) {
    Serial.println("Card failed, or not present, use EEPROM"); //TODO# blink, use EEPROM
	#ifdef useOLED
		display.setTextSize(2);             // Normal 1:1 pixel scale
		display.setTextColor(SSD1306_WHITE);
		display.setCursor(0,0);             // Start at top-left corner
		display.print("SD err, use EEPROM");
	#endif
    // don't do anything more:
    bSD=false;
  }
#endif
}

#ifdef useFastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(step_p);
  if (stepper) {
    stepper->setDirectionPin(dir_p);
    stepper->setEnablePin(enable_p);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeed(1000);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);
  }
#else
Timer1.initialize(100);         // initialize timer1, mks period
//Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
#endif

// Serial.print("main running on core ");
// Serial.println(xPortGetCoreID());	
xTaskCreatePinnedToCore(
                    US_Read_Task_f,   /* Task function. */
                    "US_Read",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task , 0 is the lowest */
                    &US_Read_Task,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

	//inputTest();
					
	#ifdef setRange_noCallibrate
	stepper_m=0;
	stepper_M=setRange_noCallibrate;
	#else
		calibrateRange();
	#endif

	us0_range=us0_M-us0_m;
	stepper_range=stepper_M-stepper_m;
	
		#ifndef save_mem
		Serial.print("stepper_range: ");
		Serial.print(stepper_m);		tab
		Serial.print(stepper_M);		tab
		Serial.print(stepper_range);	tab
		Serial.print("us0_range: ");
		Serial.print(us0_m);		tab
		Serial.print(us0_M);		tab
		Serial.print(us0_range);	tab
		Serial.println();
		while(stepper_range<11) 
		{
			Serial.println("stepper_range too small, calibrateRange again");
			calibrateRange();
		}
		#endif

// ~33cm=26527 @1/8
 rec_time_in_slot_file=get_rec_time_in_slot_file();
}

void apply_Pos_SpeedUS_Acceleration(int32_t p, uint32_t us, uint32_t acc){
#ifdef useFastAccelStepper
stepper->setSpeed(us);			// 5...10000
stepper->setAcceleration(acc); //....25k @1/8    ....110k@1/16
stepper->moveTo(p);
//stepper->applySpeedAcceleration(); // do not need after moveTo
#endif
}
void setAcceleration(uint32_t v){ //TODO 2 opt check if compiler do it or use some macro
	#ifdef useFastAccelStepper
	stepper->setAcceleration(v); // New value will be used after call to move/moveTo/stopMove/applySpeedAcceleration
	stepper->applySpeedAcceleration();
	#else
	stepper.setAcceleration(v);
	#endif
}
void setMaxSpeed(uint32_t v){
	#ifdef useFastAccelStepper
	stepper->setSpeed(1000000/v);
	stepper->applySpeedAcceleration();
	#else
	stepper.setMaxSpeed(v);
	#endif
}
void setMaxSpeed_us(uint32_t v){
	#ifdef useFastAccelStepper
	stepper->setSpeed(v);
	stepper->applySpeedAcceleration();
	#else
	stepper.setMaxSpeed(1000000/v);
	#endif
}
void moveTo(int32_t v){
	#ifdef useFastAccelStepper
	stepper->moveTo(v);
	#else
	stepper.moveTo(v);
	#endif
}
int32_t currentPosition(){
	#ifdef useFastAccelStepper
	return stepper->getCurrentPosition();
	#else
	return stepper.currentPosition();
	#endif
}
int32_t setCurrentPosition(uint32_t v){
	#ifdef useFastAccelStepper
	stepper->setCurrentPosition(v); // ! on ESP32  does not consider the steps of the current command => recommend to use only in standstill
	#else
	stepper.setCurrentPosition(v);
	#endif
}

#ifdef debug
long nextPrintDebug_t=0;
#endif

//---------- this also mean modes: buttons act different in different mode, see https://pavel-b-kr12.github.io/recorder_for_potentiometer/index.htm
bool bRec=false; 
bool bPlay=false;
bool bPause=false;
//----------
byte slot=0;
byte slot2=0; //2nd play slot to mix and fade between


int32_t pot_pos_shift_a_last=0;
int32_t pot_precission_playSpd_a_last=0;

int32_t pos_to_mov=-1; //0........stepper_M //TODO fix mega328  0....255

uint32_t pos_to_mov_step=0; //stepper_m......stepper_M
int32_t pos_to_mov_last=-1;
uint32_t pos_to_mov_spd=-1;
uint32_t pos_to_mov_spd_last=-1;


long nextUpd_LCD_t=0;
int16_t nextLCD_off_il=nextLCD_off_il_DT*10; //number of LCD updates, while >=0 display stay active
bool bLCD_on=true;

long next_btns_read_t=0;
bool bWait_btns_off=false;

long pot_read_next_t=0;
#ifdef useESP32
uint16_t pot_pos_shift_a;
uint16_t pot_precission_playSpd_a;
#else
byte pot_pos_shift_a;
byte pot_precission_playSpd_a;
#endif

void stop(){
		stepper->setAcceleration(acc_settings_M_M);
		stepper->applySpeedAcceleration();
		stepper->stopMove();
		delay(200);
}

int getUSData_avg(byte N, byte E)
{
	//if(E<2) return sonar[i].ping();
	
	int val=0;
	delay(333); //wait motor stop
	for(byte i=0;i<E;i++)
	{
		val+=sonar[N].ping();			delay(10);
	}
	return val/E;
}

long US_next_t=0;
int32_t spd=3000;
int32_t acc=11000;
#ifndef useFastAccelStepper
void callback()
{
  //digitalWrite(10, digitalRead(10) ^ 1);
  if(digitalRead(btn_limit_near_p) && digitalRead(btn_limit_far_p))
	stepper.run();
	//stepper.runSpeed(); //^^nwp
  //else
	//Timer1.stop();
	  //noInterrupts();
}
#endif
void check_lim_sw(){
	bbtn_limit_near=!digitalRead(btn_limit_near_p);
	bbtn_limit_far=!digitalRead(btn_limit_far_p);
	check_wrong_lim_sw();
	
	if(bbtn_limit_near)
	{
		stop();
		
		//stepper->setAcceleration(500);
		stepper->applySpeedAcceleration();
		moveTo(2000);
		//stepper->runForward();
		Serial.println("runForward");
		
		#ifndef tstfast
		while(!digitalRead(btn_limit_near_p)){ delay(1);}
		delay(10);
		stop();
		#endif
		
		stepper_m=0;
		stepper->forceStopAndNewPosition(0); //instant stop
		Serial.println("forceStopAndNewPosition(0)");
		
		//us0_m=getUSData_avg(0,8);
		#ifndef save_mem
				Serial.print("set near, US=");	Serial.println(us0_m);
		#endif

		stepper->setCurrentPosition(0);
		Serial.println(stepper->getCurrentPosition());
		stepper->setAcceleration(acc);
		stepper->applySpeedAcceleration();
	}
	else 
	if(bbtn_limit_far)
	{
		stop();
		//stepper->setAcceleration(500);
		stepper->applySpeedAcceleration();
		moveTo(stepper->getCurrentPosition()-2000);
		//stepper->runBackward();
		Serial.println("runBackward");
		#ifndef tstfast
	while(!digitalRead(btn_limit_far_p)) { delay(1);}  //stepper->backwardStep(); //while(stepper->getCurrentPosition()>stepper_M);
		stop();
		#endif	
		stepper_M=stepper->getCurrentPosition()-50;
		//us0_M=getUSData_avg(0,8);
		#ifndef save_mem
				Serial.print("set far = ");	Serial.print(stepper_M); Serial.print(" , US=");Serial.println(us0_M);
		#endif	
		stepper->setAcceleration(acc);
		stepper->applySpeedAcceleration();
		//delay(2000);
	}
}
void check_wrong_lim_sw(){
if(bbtn_limit_near && bbtn_limit_far) 
{
	#ifdef useFastAccelStepper
	stop();
	//stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
	stepper->disableOutputs();
	#else
	Timer1.stop();
	#endif

	while(bbtn_limit_near && bbtn_limit_far)
	{
		bbtn_limit_near=!digitalRead(btn_limit_near_p);
		bbtn_limit_far=!digitalRead(btn_limit_far_p);
		
		display.clearDisplay();
		draw_lim_sw();
		display.print("Err all lim sw pressed");
		display.display();
		delay(50);
	}
	stepper->enableOutputs();
	bPause=true;
}
}

uint32_t us1_v=-1;
uint16_t us1_avg=555;
long next_US_read_t=0;
int us1_pos_for_PlayMode()
{
	if(millis()>next_US_read_t)
	{
		next_US_read_t=millis()+10;
		
		us1_v=sonar[1].ping();
		
		if(us1_v>200 &&us1_v<2000)
		{
			us1_v=map(us1_v, 200, 2000, 0, stepper_range);
			//if(v>0 &&v<stepper_range)
			{
				//Serial.println(v);
				int32_t d=us1_avg-us1_v;
				if(abs(d)>200)
				{
					us1_avg=us1_avg*0.95+us1_v*0.05;
				}
				else
					us1_avg=(us1_avg+us1_v)/2;
				
				//pos_to_mov=us1_avg;
			}
			return us1_avg;
			
		}
		else return -1;
	}
}


long callibrateDestination_M=50000; //positive num = mov from near to far
void calibrateRange()
{
	//setMaxSpeed(Speed_MAX); //1000? is max for this lib, also 1000 is peak of noise for some motors. //But up to 6k also work at mega328
	//stepper.setSpeed(3000);

	acc=20000;
	stepper->setSpeed(Speed_callib);			// 5...10000
	stepper->setAcceleration(acc);
	stepper->applySpeedAcceleration();
	
	#ifndef save_mem
	Serial.println("callibrate start w check lim");
	#endif
	check_lim_sw();  //TODO удлинить чтоб не бились по инерции

	/*
	#ifndef useFastAccelStepper
	if(bbtn_limit_near) stepper.runToNewPosition(400); //TODO callibrateDestination=-callibrateDestination
	if(bbtn_limit_far) stepper.runToNewPosition(-400);
	#endif
*/

	//byte pot_pos_shift_a=(analogRead(pot_pos_shift_p)+analogRead(pot_pos_shift_p))/8; //0..255
	

			//Serial.print(bbtn_limit_near);Serial.print(bbtn_limit_far);tab
			//Serial.println(millis());
	
	#ifndef save_mem
	Serial.println("go to callibrate near");
	#endif
	
	moveTo(-callibrateDestination_M);
	//wait lim sw
	while(!bbtn_limit_near) //TODO if bbtn_limit_far - change near<>far
	{
		check_lim_sw();
		if(stepper->getCurrentPosition()<-callibrateDestination_M+100)
		{
			#ifndef save_mem
			Serial.println("cant find limit_near"); delay(30);
			#endif
		}
		
	}
			#ifndef useFastAccelStepper
			Timer1.stop();
				//stepper.disableOutputs();
				//stepper.stop();
				//stepper.runToPosition();
			Timer1.restart();
			#endif
	#ifndef save_mem
	#ifdef useOLED
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0,0);             // Start at top-left corner
	display.print("US min=");display.print(us0_m);
	display.display();
	#endif
	#endif

	#ifndef save_mem
	Serial.println("go to callibrate far");
	#endif
	moveTo(callibrateDestination_M);
	Serial.println("moveTo(callibrateDestination_M);");
	while(!bbtn_limit_far)
	{
		check_lim_sw();
		if(stepper->getCurrentPosition()>callibrateDestination_M-100)
		{
			#ifndef save_mem
			Serial.println("cant find limit_far"); delay(30);
			#endif
		}
		
					//Serial.println(stepper->getCurrentPosition());
		if(stepper->getCurrentPosition()>2000 && stepper->getCurrentPosition()<17000)
		{
			stepper->setSpeed(Speed_callib_M);		/*Serial.println("fst"); */ delay(10);
		}
		else 
			stepper->setSpeed(Speed_callib);
		stepper->applySpeedAcceleration();
	}
	#ifndef save_mem
	#ifdef useOLED
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0,16);             // Start at top-left corner
	display.print("US max=");display.print(us0_M);
	display.display();
	#endif
	#endif
	
											stepper->setSpeed(50);
											stepper->setAcceleration(35000);
											stepper->applySpeedAcceleration();

}


int pos_from_US(int v){
	return map(v, us0_m, us0_M, stepper_m, stepper_M);
}

uint16_t rec_ps=0;
long rec_ps_0_t;

void record(uint32_t rec_time,	uint32_t pos_to_mov)
{
	if(rec_ps==0)
	{
		rec_ps_0_t=millis();
	}
	rec_ps++;
	if(millis()>rec_ps_0_t+1000)
	{
		Serial.print(rec_ps);tab
		Serial.print(rec_ps_0_t);tab
		Serial.print(millis());tab
		Serial.println(millis()-rec_ps_0_t);
		rec_ps=0;
	}
	
	
	if(bSD) //!opt
	{
	  //unsigned long now = millis();
	  //if ((now - lastMillis) >= 5)
	  {
		// add a new line to the buffer
		buffer += rec_time;
		buffer += "\t";
		buffer += pos_to_mov; //(uint16_t)((analogRead(A0)+analogRead(A0))/4);
		buffer += "\n";

		//lastMillis = now;
		
		#ifndef save_mem
		//buffer is long do not print it Serial.println(buffer);
				   // Serial.print(rec_time);tab
				   // Serial.print(pos_to_mov);Serial.println();
		#endif
	  }


																							
	  // check if the SD card is available to write data without blocking
	  // and if the buffered data is enough for the full chunk size
	  #ifdef useESP32
		unsigned int chunkSize = 512; //TODO fix as txtFile.availableForWrite(); will be available
		if (chunkSize && buffer.length() >= chunkSize)
	  #else
		unsigned int chunkSize = txtFile.availableForWrite();
		if (chunkSize && buffer.length() >= 256) //256 for save memory on mega328 it seems 388 is lim here 
	  #endif
	  {
																							#ifndef save_mem
																									   // Serial.print(chunkSize);tab
																									   // Serial.print(buffer.length());
																									   // Serial.println();
																							#endif
	  
																							#ifndef save_mem
																									   //Serial.print(buffer);Serial.println();
																							#endif
		// write to file and blink LED
		//digitalWrite(LED_BUILTIN, HIGH);	//not for mega328 because 13 is clock pin SPI
		#ifdef useESP32
		txtFile.write((const uint8_t*)buffer.c_str(), chunkSize);
		#else
		txtFile.write(buffer.c_str(), chunkSize); //TODO2 test if need or all work as in useESP32
		#endif
		//digitalWrite(LED_BUILTIN, LOW);	//not for mega328 because 13 is clock pin SPI

		// remove written data from buffer
		buffer.remove(0, chunkSize);

	  }
	}
	else
	{
		if(record_dataN>=EEPROM.length()-1)
		{
			bRec=false;
			rec_time=0;
			record_dataN=0;
			return;
		}
		
		EEPROM.update(record_dataN, rec_time/128);
		EEPROM.update(record_dataN+1, pos_to_mov/4);
		
		record_dataN+=2;
	
					// Serial.print(record_dataN);	tab
					// Serial.print(rec_time/128);	tab
					// Serial.print(pos_to_mov/16);	tab
					// Serial.println();
	}
	nextLCD_off_il=nextLCD_off_il_DT;
}

int err_i=0;
void play()
{
	if(bSD)
	{
		#ifdef useSdFat
		#else

		while (txtFile.available()) {
			if(err_i>100)
			{
				Serial.print("???"); 	Serial.println(err_i); 
				break;
			}
		  //Serial.write(txtFile.read());
		  long play_data_t=txtFile.parseInt(SKIP_NONE);
		  if(!txtFile.available()) {Serial.println("b1?"); break;}
		  if(play_data_t<0|| play_data_t>3600000 || play_data_t>rec_time_loaded+720000) {Serial.println("c1 play_data_t?"); Serial.println(play_data_t); err_i++; continue;} //1h time limit, 12 min idle time limit //!! t < t_last
		  if(txtFile.read()!='\t')  {Serial.println("c2? !='\t'"); err_i++; continue;} //probably err
		  if(!txtFile.available())  {Serial.println("b2?"); break;}
		  long play_data_v=txtFile.parseInt(SKIP_NONE);
		  if(!txtFile.available())  {Serial.println("b4?"); break;}
		  if(play_data_v<0 || play_data_v>stepper_range*2)  {Serial.println("c3? play_data_v>stepper_range*2");Serial.println(play_data_v);   err_i++; continue;} 
		  //!! lim //*2 because lim is not static. TODO recalculate , write range in file or use 0...999
		  if(txtFile.read()!='\n')  {Serial.println("c4?"); err_i++; continue;} //probably err


			rec_time_loaded=play_data_t;  
			pos_to_mov=play_data_v;
			
		#ifndef save_mem
				  // Serial.print(rec_time);tab
				  // Serial.print(rec_time_loaded);tab
				  // Serial.print(pos_to_mov);Serial.println();
		#endif
			
					// Serial.print(record_dataN);	tab
					// Serial.print(rec_time/128);	tab
					// Serial.print(rec_time_loaded);	tab
					// Serial.print(pos_to_mov);	tab

			nextLCD_off_il=nextLCD_off_il_DT;
		  return;
		}
		//else //TODO cyclic mode
		{
					// Serial.println("EOF? rec_time_loaded, rec_time_in_slot_file:");
					// Serial.println(rec_time_loaded);
					// Serial.println(rec_time_in_slot_file);
			txtFile.close();
			if(!bPlayCycling) bPlay=false;
			else playStart();
		}

		#endif
	}
	else
	{
		if(record_dataN>=EEPROM.length()-1)
		{
					bPlay=false;
			rec_time=0;
			record_dataN=0;
			
			return;
		}
		
		pos_to_mov= EEPROM.read(record_dataN)*128;
		rec_time_loaded= EEPROM.read(record_dataN+1)*4;
		
		if(record_dataN!=0 && rec_time_loaded==0) //end marker found //TODO record №1 >0
		{
					bPlay=false;
			rec_time=0;
			record_dataN=0;
			
			return;
		}
		
		record_dataN+=2;
		
		// Serial.print(rec_time);	tab
		// Serial.print(t);	tab
		// Serial.print(v);	tab

				// Serial.print(record_dataN);	tab
				// Serial.print(rec_time/128);	tab
				// Serial.print(rec_time_loaded);	tab
				// Serial.print(pos_to_mov);	tab
		
		nextLCD_off_il=nextLCD_off_il_DT;
	}
	// Serial.println();
}

bool bSettingsMode=false;
bool bSettingsMode_SetMaxAcc=false;
bool bSettingsMode_SetMaxSpd=false;


float spd_playback;

uint32_t get_rec_time_in_slot_file()
{
	 File file = SD.open(filename_rec_time);
	 if(!file) return 0;
	 long t=file.parseInt();
	 file.close();
	 return t;
}

int32_t pos_to_mov_recorded=-1; //-1 mean not recorded jet, as pos_to_mov >0


void playStart()
{
			rec_time=0;
			rec_time_loaded=0;
			rec_time_last=0;
			record_dataN=0;

				if(bSD)
				{
				#ifdef useSdFat
				#else
					#ifdef print_file_after_recorded
					Serial.println("==================================");
					
					Serial.println("rec_time:");
					Serial.println(get_rec_time_in_slot_file());

					  File fff = SD.open(filename);
					  if (fff) {
						while (fff.available()) {
						  Serial.write(fff.read());
						}
						fff.close();
					  }	else {
						Serial.println("error opening "+fff);
					  }
					   /* //print parsed
					   Serial.println("++++++++++++++++++++++++++++++++++");
					  	 fff = SD.open(filename);
						while (fff.available()) {
						  //Serial.write(fff.read());
						  long m=fff.parseInt(SKIP_NONE);
						  if(fff.read()!='\t') continue; //probably err
						  long a=fff.parseInt(SKIP_NONE);
						  if(fff.read()!='\n') continue; //probably err

								  Serial.print(m);tab
								  Serial.print(a);Serial.println();
						  
						}
						fff.close();
						*/
					  Serial.println("==================================");
					  #endif
					  err_i=0;
					  
				  txtFile = SD.open(filename);
				  if (!txtFile) {
					Serial.print("err open ");
					Serial.println(filename);
					//while (1);
					//bSD=false; Serial.println("use EEPROM");
					}
				#endif
				}
				t_last=millis();
}	
void endRec()
{
	if(bSD)
	{
		#ifdef useESP32
		txtFile.write((const uint8_t*)buffer.c_str(), buffer.length());
		#endif
		txtFile.close();
		
		rec_time_in_slot_file=rec_time;
		//------- write last max (total) rec_time to filename_rec_time
		File ff = SD.open(filename_rec_time, FILE_WRITE);
		if (ff) {
			ff.print(rec_time);
			ff.close();
		}
		else
		{
			Serial.print("err open ");	Serial.println(filename_rec_time);
		}
		//-------
		#ifdef print_file_after_recorded
			Serial.println("end rec");
			Serial.println("rec_time:");
			Serial.println(rec_time);
			Serial.println(get_rec_time_in_slot_file());

			ff = SD.open(filename);
			if (ff) {
				while (ff.available()) {
				  //Serial.write(ff.read());
				  long m=ff.parseInt(SKIP_NONE);
				  if(ff.read()!='\t') continue; //probably err
				  long a=ff.parseInt(SKIP_NONE);
				  if(ff.read()!='\n') continue; //probably err

						  Serial.print(m);tab
						  Serial.print(a);Serial.println();
				}
				ff.close();
			}
			else {
			Serial.print("error opening "); Serial.println(filename);
			}
		#endif
	}
	else
	{
		if(record_dataN<EEPROM.length()) //end marker
			EEPROM.update(record_dataN, 0);
	}
}
void loop() {
	
check_lim_sw();		

//if(millis()>pot_read_next_t)
{
	#ifdef useESP32
	uint32_t a=analogRead(pot_pos_shift_p);
	uint32_t a1=analogRead(pot_precission_playSpd_p);
	a+=analogRead(pot_pos_shift_p);
	a1+=analogRead(pot_precission_playSpd_p);
	a+=analogRead(pot_pos_shift_p);
	a1+=analogRead(pot_precission_playSpd_p);
	a+=analogRead(pot_pos_shift_p);
	a1+=analogRead(pot_precission_playSpd_p);
	a+=analogRead(pot_pos_shift_p);
	a1+=analogRead(pot_precission_playSpd_p);
	a+=analogRead(pot_pos_shift_p);
	a1+=analogRead(pot_precission_playSpd_p);
	
	pot_pos_shift_a=constrain(map( a, 0, 4095*6, 0, stepper_range), 0, stepper_range); //pot_pos_shift_a*0.5+constrain(map( a, 0, 4095*3, 0, stepper_range), 0, stepper_range)/2; //0..stepper_range
	pot_precission_playSpd_a=constrain(map( a1, 0, 4095*6, 0, stepper_range), 0, stepper_range); //pot_precission_playSpd_a*0.5+constrain(map( a1, 0, 4095*3, 0, stepper_range), 0, stepper_range)/2; //0..stepper_range
	
				// Serial.print(analogRead(pot_pos_shift_p));tab
				// Serial.print(stepper_range);tab
				// Serial.print(map( analogRead(pot_pos_shift_p), 0, 4095, 0, stepper_range));tab
				// Serial.print(constrain(map( analogRead(pot_pos_shift_p), 0, 4095, 0, stepper_range), 0, stepper_range));tab
				// Serial.print(pot_pos_shift_a);
			// Serial.println();
			
	#else
	pot_pos_shift_a=(analogRead(pot_pos_shift_p)+analogRead(pot_pos_shift_p))/8; //0..255
	pot_precission_playSpd_a=(analogRead(pot_precission_playSpd_p)+analogRead(pot_precission_playSpd_p))/8; //0..255
	#endif
}

// off display while nothing changed
if(bbtn_limit_near||bbtn_limit_far) nextLCD_off_il=nextLCD_off_il_DT;

//--------------------------------- single press Rec to start set spd, acc , while in slot 9
uint16_t btns_atmp;
bool bRkey_tmp=false;
if(bSettingsMode)
{
	btns_atmp=analogRead(btns_p);
	bRkey_tmp=btns_atmp<100;
}
//---------------------------------	

if(abs(pot_pos_shift_a_last-pot_pos_shift_a)>stepper_range/256) //!opt  stepper_range/2048
{
		// Serial.print(pot_pos_shift_a_last);tab
		// Serial.print(pot_pos_shift_a);tab
		// Serial.print(abs(pot_pos_shift_a_last-pot_pos_shift_a));tab
		// Serial.print(abs(pot_precission_playSpd_a_last-pot_precission_playSpd_a));tab
		// Serial.print(stepper_range/256);tab
		// Serial.println();
		
	pot_pos_shift_a_last=pot_pos_shift_a;
	nextLCD_off_il=nextLCD_off_il_DT;
	
	if(bSettingsMode && bRkey_tmp)	bSettingsMode_SetMaxAcc=true; //this is switch
}
if(bSettingsMode_SetMaxAcc)
{
	acc_settings_M =map(pot_pos_shift_a,0,stepper_range, 1000, acc_settings_M_M);
}
	
if(abs(pot_precission_playSpd_a_last-pot_precission_playSpd_a)>stepper_range/256) //!opt  stepper_range/2048
{
	pot_precission_playSpd_a_last=pot_precission_playSpd_a;
	nextLCD_off_il=nextLCD_off_il_DT;
	
	if(bSettingsMode && bRkey_tmp)		bSettingsMode_SetMaxSpd=true; //this is switch
}
if(bSettingsMode_SetMaxSpd)
{
	spd_us_setings_m =map(pot_precission_playSpd_a,0,stepper_range, 0, 1000);
}


//read btns, sw modes
if(millis()>next_btns_read_t)
{
int btns_a=analogRead(btns_p);
		// Serial.print(analogRead(pot_pos_shift_p));tab
		// Serial.print(analogRead(pot_precission_playSpd_p));tab
		// Serial.print(analogRead(btns_p));tab
		// Serial.println();

#ifdef useESP32
if(btns_a<3800)	//2847 2505 1840 0
#else
if(btns_a<900)  //off = 1006
#endif
{
	if(bWait_btns_off) return;
	bWait_btns_off=true;
	nextLCD_off_il=nextLCD_off_il_DT*5;
	next_btns_read_t=millis()+200;
	
	if(btns_a<70) //=14 for all R between btns = 10k
	{
		bRec=!bRec;
		if(bRec)
		{
			bPlay=false;
			if(txtFile)	txtFile.close();
			
			//if(!bPlay) //if bPlay - continue record from current rec_time TODO get record_dataN
			{  //start recording from 0
				rec_time=0;
				rec_time_last=0;
				record_dataN=0;
				
				if(bSD)
				{
				#ifdef useSdFat
				#else
				// Check to see if the file exists:
				  if (SD.exists(filename)) { //// delete the file: //!! TODO slots
					#ifdef debug
					Serial.println("exists, removing..");
					Serial.println(filename);
					#endif
					SD.remove(filename);
				  } else {

				  }
 
				  txtFile = SD.open(filename, FILE_WRITE);
				  if (!txtFile) {
					Serial.print("err open ");	Serial.println(filename);
					//while (1);
					//bSD=false; Serial.println("use EEPROM");
					}
				#endif
				}
				t_last=millis();
			}
		}
		else
		{
			endRec();
		}
		
		bPlay = false; //TODO record while play to diferrent slot,  TODO play 2 slots together fade between them (select 2 slots while bPause)
	}
	else
	#ifdef useESP32
	if(btns_a>100 &&btns_a<2000)  //1840
	#else
	if(btns_a>100 && btns_a<300)  //227
	#endif
	{						
		bPlay=!bPlay;					Serial.print("bPlay="); Serial.println(bPlay);
		
		if(bRec) //TODO play from slot2
		{
			bRec=false;
			endRec();
		}
		if(bPlay)
		{
			playStart();
		}
		else
		{
			if(bSD)
			{
				if(txtFile)	txtFile.close();
			}
		}
	}
	else
	#ifdef useESP32
	if(btns_a>2222 && btns_a<2600)  //2505
	#else
	if(btns_a>300 && btns_a<400)  //363
	#endif
	{
		if(bPause)
		{
			slot2++; if(slot2>9)slot2=0;
		}
		else
		{
			if(!bRec)
			{
				slot++; if(slot>9)slot=0;
				filename[2]=slot+'0'; //convert digit to char
				filename_rec_time[2]=slot+'0';
				rec_time_in_slot_file=get_rec_time_in_slot_file();
				//# if(bPlay) close, open  file
				bSettingsMode=slot==9;
			}
		}
	}
	else
	#ifdef useESP32
	if(btns_a>2600)  //2847
	#else
	if(btns_a>400)  //456
	#endif
	{
		if(bSettingsMode)
			bPlayCycling=!bPlayCycling;
		else
			bPause=!bPause;
	}
}
else
{
	if(bWait_btns_off)
	{
		bWait_btns_off=false;
		next_btns_read_t=millis()+200;
	}
	else
		next_btns_read_t=millis()+20;
}

}
//=====================================================================
if(!bPlay) // get pos_to_mov, spd, acc from pots values
{
	pos_to_mov_spd_last=pos_to_mov_spd; //need last value to calculate speed because record of actual pot position, but not of motor pos
	pos_to_mov_last=pos_to_mov;

	if(bPause)
	{
		/*  //test US control //can interferre w us1_pos_for_PlayMode();
		if(millis()>next_US_read_t)
		{
			next_US_read_t=millis()+10;
			
			setAcceleration(4000);
			uint16_t v=sonar[1].ping();
			
			if(v>200 &&v<2000)
			{
				v=map(v, 200, 2000, 0, stepper_range);
				//if(v>0 &&v<stepper_range)
				{
					Serial.println(v);
					
					if(abs(us1_avg-v)>200)
					{
						us1_avg=us1_avg*0.95+v*0.05;
					}
					else
						us1_avg=(us1_avg+v)/2;
					
					pos_to_mov=us1_avg;
				}
			}
		}
		*/
	}
	else
	{
		#ifdef useESP32
			pos_to_mov=   //## TODO calc pot range in steps 
			pot_pos_shift_a_last //use _last to be able to avoid small move
			+pot_precission_playSpd_a_last/16;
			
			pos_to_mov_spd=pos_to_mov-pos_to_mov_last;
			/* TODO tst , now it calc below
			
			uint32_t us=pos_to_mov_spd<10?1000:(10+1000/pos_to_mov_spd);
			stepper->setSpeed(us);
			uint32_t acc=constrain(pos_to_mov_spd*10, 1000, 90000);
			stepper->setAcceleration(acc);
			 //TODO tst , now it calc below
			 //stepper->applySpeedAcceleration();
			*/
		#else
			setAcceleration(50000);
			pos_to_mov=
			pot_pos_shift_a_last*4
			+pot_precission_playSpd_a_last/4-32;
			pos_to_mov_spd=pos_to_mov-pos_to_mov_last;
		#endif
	}
	//if(pos_to_mov<0) pos_to_mov=0; //##
}
/*
Serial.print(bRec);tab
Serial.print(bPlay);tab
Serial.print(slot);tab
Serial.print(bPause);tab
Serial.print(nextLCD_off_il);tab
//Serial.println();
*/
//================================================================================================
if(!bPause) //TODO off or do not off motor
{
uint32_t pos_to_mov_temp=pos_to_mov;
if(!bSettingsMode)
{
	if(bRec || bPlay)
	{
		if(!bPause)
		{
				uint16_t dt=millis()-t_last;
				
				if(bPlay)
				{	 //playback speed from pot_precission_playSpd_a
					spd_playback=((float)pot_precission_playSpd_a/stepper_range)*3.0;
					if(spd_playback>0.1 && spd_playback<2.5) 
					{
						dt*=spd_playback;
					}
				}

				rec_time+=dt;
				t_last=millis();
				
				if(bRec)
				{
					if(
					pos_to_mov!=pos_to_mov_recorded &&
					(
					rec_time>rec_time_last+50 || 
					(  abs(pos_to_mov_recorded-pos_to_mov)>20 && (rec_time>rec_time_last+5) ) //speed changed so faster record rate
					//abs(pos_to_mov_spd-pos_to_mov_spd_last)>5  //TODO test is better
					)
					)
					{
						rec_time_last=rec_time;
						record(rec_time,	pos_to_mov);
						pos_to_mov_recorded=pos_to_mov;
					}
				}
				
				if(bPlay) //TODO 
				{
					if(rec_time>=rec_time_loaded)
					{
						play(); //update pos_to_mov and rec_time_loaded
					}
					/*
										if(dt>0)
										{
							 									Serial.print(dt);tab
																Serial.print(rec_time);tab
																Serial.print(rec_time_loaded);tab
																Serial.print(pos_to_mov);Serial.println();
										}*/
																
					#ifdef use_play_shift
						#ifdef useUS1
						//int us1_pos=us1_pos_for_PlayMode();
						//if(us1_pos>=0) //if us1 active controlled - use it for stepper pos
						if(US_input_v>700 && US_input_v<2500 && US_input_v_good_i>3)
							pos_to_mov_temp=map(US_input_v,700,2500,0, stepper_range);
						else //use playback pos + shift it with pot value (slider) //TODO if 
						#endif
						{
							pos_to_mov_temp=pos_to_mov;
							uint32_t m=stepper_range*0.02;
							uint32_t M=stepper_range*0.98;
							if(pot_pos_shift_a_last>m &&  pot_pos_shift_a_last<M)
							{
								int32_t sh=map(pot_pos_shift_a_last,m,M, -stepper_range*0.4, stepper_range*0.4);
								if(pos_to_mov_temp+sh>0)
								pos_to_mov_temp+=sh;
								else pos_to_mov_temp=0;
							}
							
							
						}
					#endif
					// #ifndef save_mem
					 // #ifdef useUS1
					 // Serial.print(us1_pos);tab
					 // #endif
					 // Serial.print(rec_time);tab
					 // Serial.print(rec_time_loaded);tab
					 // Serial.print(pos_to_mov);tab
					 // Serial.print(pos_to_mov_temp);tab
					 // Serial.println();
					// #endif
				}
		}
		else
			t_last=millis();
	}

}
if(slot==8 && US_input_v>600 && US_input_v<2500) pos_to_mov_temp=map(US_input_v,600,2500,0, stepper_range); //US test control when select slot 8
pos_to_mov_step=constrain(pos_to_mov_temp, 0, stepper_M);


	
int32_t s=currentPosition()-pos_to_mov_step;
//TODO btn press for smooth or fast-acc movement. , save this inf 
#ifdef useFastAccelStepper
int32_t ss=abs(s)/20;
ss=pow(ss,1.6);
spd= 200000/(10+ss); //constrain(map(, 0, stepper_range/4, 4000,0), spd_us_setings_m, 2000); 
if(bPlay) spd/=spd_playback;

//acc=abs(s)>1000?70000:25000;
//acc= 100+pow(abs(s)/4,2)*10;//+90000/(1+spd/4);
//-------------------
//spd= pow(abs(s)/20,2);
//acc = spd*4;
//acc = acc_settings_M;
//spd=1000000/(50+spd);
//spd=50;
//-------------------

spd=constrain(spd, spd_us_setings_m, 9000); 
//2F I think it will be good if available sample that show how to properly move stepper to position of the input potentiometer  with speed and acceleration proportional to difference between target and actual positions.
#else
spd=constrain(abs(s),spd_us_setings_m, 10000); //max 4000 steps per second at a clock frequency of 16 MHz 
acc=spd*4;
#endif

acc=constrain(acc,100, acc_settings_M);

//apply_Pos_SpeedUS_Acceleration(pos_to_mov_step, spd, acc); //TODO fix for AccelStepper //TODO apply only if changed
//apply_Pos_SpeedUS_Acceleration(analogRead(pot_pos_shift_p)*5, 50, 5000); //TODO fix for AccelStepper //TODO apply only if changed
stepper->setSpeed(spd);
//stepper->setAcceleration(15000); //BUG! https://github.com/gin66/FastAccelStepper/issues/40
//stepper->moveTo(analogRead(pot_pos_shift_p)*5);
stepper->moveTo(pos_to_mov_step);
//setAcceleration(10+(float)pot_precission_playSpd_a*1000);



#ifdef debug
/*
if(millis()>nextPrintDebug_t)
{
nextPrintDebug_t=millis()+1000;
Serial.print(pot_pos_shift_a);tab
Serial.print(map(pot_pos_shift_a_last,stepper_range*0.02,stepper_range*0.98, -stepper_range*0.4, stepper_range*0.4));tab
//Serial.print(pot_pos_shift_a_last);tab
Serial.print(pos_to_mov);tab
Serial.print(pos_to_mov_temp);tab
Serial.print(currentPosition());tab
Serial.print(pos_to_mov_step);tab
Serial.print(s*10);tab
Serial.print(ss);tab
Serial.print(spd);tab
Serial.print(acc);tab
Serial.println();
}
*/
#endif


} //bPause
else t_last=millis();

LCD_upd();
}
