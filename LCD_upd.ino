
void draw_lim_sw(){
#ifdef useOLED
if(bbtn_limit_near) display.drawLine(0,0, 0, 31, SSD1306_WHITE);
if(bbtn_limit_far) display.drawLine(127,0, 127, 31, SSD1306_WHITE);
#endif
}

#define txw	5

void LCD_upd()
{
#ifdef useOLED
  #ifndef LCD_active_always
  if(nextLCD_off_il<0 && slot<7) //slots 7-9 always active
  {
    if(bLCD_on)
    {
		bLCD_on=false;
		display.clearDisplay();
		display.display();
    }
  }
  else
  #endif
  if(millis()>nextUpd_LCD_t)
  {
	#ifndef LCD_active_always
	nextLCD_off_il--;
	if(!bLCD_on)bLCD_on=true;
	#endif
	nextUpd_LCD_t=millis()+LCD_upd_dt;
		
	 
	display.clearDisplay();

	display.setTextSize(1);             // Normal 1:1 pixel scale
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0,0);             // Start at top-left corner
	display.print(slot);
	
	if(slot==8) //US control mode for test
	{
		display.setCursor(0,16);
		display.print("US1: ");
		display.print(US_input_v);
		display.print("  US0: ");
		display.print(US_pos_v);
	}
	else
	{
		if(slot2!=0 && slot2!=slot)
		{
		display.print(",");
		display.print(slot2);
		}
		display.print(" ");
		uint32_t s=rec_time_in_slot_file/1000;
		byte m=s/60;
		display.print(m);
		display.print(":");
		display.print(s%60);
		display.print(" ");
		if(bRec)
		 display.print(" REC");
		// else 
		if(bPlay)
		 display.print(" play");
		if(bPause)
		 display.print(" ||");
		if(bSettingsMode)
		 display.print(" settings");
			 
		draw_lim_sw();
		if(bPlayCycling) 
		{
			display.setCursor(100,10);
			display.print("reP");
		}
			
		if(bSettingsMode)
		{
			display.setCursor(0,10);
			display.print("spd us min = ");
			display.print(spd_us_setings_m);
			
			display.setCursor(0,20);
			display.print("acc Max = ");
			display.print(acc_settings_M);
		}
		else if(!bPlay && !bRec)
		{
			//slot          actual pos
			//pot1   pot2   desired pos
			display.setCursor(0,16);
			display.print((int)((float)pot_pos_shift_a/stepper_range*99));
			display.setCursor(6*txw,16); display.print("+");
			display.setCursor(8*txw,16);
			display.print((int)((float)pot_precission_playSpd_a/stepper_range*99));
			display.setCursor(15*txw,16); display.print("=");
			display.print(pos_to_mov_step);
			display.setCursor(16*txw,0); 
			display.print(pos_from_US(US_pos_v));
		}
		else if(bRec)
		{
			//slot
			//time   desired pos
			display.setCursor(0,16);
			display.print(rec_time/100);
			display.setCursor(16*txw,16);
			display.print(pos_to_mov);
		}
		else if(bPlay)
		{
			//slot                  actual pos    acc
			//time   playback spd   desired pos   file time   
			display.setCursor(0,16);
			display.print(rec_time/100);
			display.setCursor(7*txw,16);
			display.print(spd_playback);
			display.setCursor(16*txw,16);
			display.print(pos_to_mov);
			display.setCursor(16*txw,0); 
			display.print(pos_from_US(US_pos_v));

			display.setCursor(25*txw,16);
			display.print(rec_time_loaded/100);
		}

	}
	display.display();

  }
#endif
}
