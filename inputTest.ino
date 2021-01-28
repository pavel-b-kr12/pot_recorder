
void inputTest()
{
	
	 while(!Serial.available()) //Type any character to break.
	 {
		Serial.print(!digitalRead(btn_limit_near_p));Serial.print("\t");
		Serial.print(!digitalRead(btn_limit_far_p));Serial.print("\t");
		
		Serial.print(pot_pos_shift_p);Serial.print("\t");
		Serial.print(pot_precission_playSpd_p);Serial.print("\t");
		Serial.print(btns_p);Serial.print("\t");
		// Serial.print(analogRead(34));Serial.print("\t");
		// Serial.print(analogRead(35));Serial.print("\t");
		// Serial.print(analogRead(32));Serial.print("\t");
		// Serial.print(analogRead(13));Serial.print("\t");
		Serial.print(analogRead(pot_pos_shift_p));Serial.print("\t");
		Serial.print(analogRead(pot_precission_playSpd_p));Serial.print("\t");
		//Serial.print("btns ");
		Serial.print(analogRead(btns_p));Serial.print("\t");
		//Serial.print("US ");
		Serial.print(sonar[0].ping());Serial.print("\t"); delay(40);
		//Serial.print("US ");
		Serial.print(sonar[1].ping());Serial.print("\t");
		Serial.println();
		delay(40); 
	 }
	 return;
	 
	
	
	//stepper.runToNewPosition(-5000);
	setMaxSpeed(1000);
	//moveTo(-5000);
	setAcceleration(4000);
	//delay(2000);
	//stepper->forceStopAndNewPosition(0);
	
	long pos_last=0;
	while(1)
	{
		while(bbtn_limit_near && bbtn_limit_far)
		{
			bbtn_limit_near=!digitalRead(btn_limit_near_p);
			bbtn_limit_far=!digitalRead(btn_limit_far_p);
			Serial.print(bbtn_limit_near);Serial.print("\t");
			Serial.print(bbtn_limit_far);Serial.print("\t");
		}

		//pot_pos_shift_read(); //todo
		//pot_precission_playSpd_read();

		Serial.print(pot_pos_shift_a);Serial.print("\t");
		Serial.print(pot_precission_playSpd_a);Serial.print("\t");

		//int pos=pot_pos_shift_a*50-2000;
		int pos=pot_pos_shift_a-4000;
		//setMaxSpeed(max(abs(currentPosition()-callibrateDestination),10000));
		int spd=abs(currentPosition()-pos)*2;
		//spd=min(spd,Speed_MAX);
		//spd=max(spd,50);

		int acc=spd*4;
		acc=min(acc,100000);
		acc=max(acc,100);
		//setAcceleration(acc);
		//setMaxSpeed(spd);
		
		pot_precission_playSpd_a=constrain(pot_precission_playSpd_a,1340,8180);
		
		spd=10+pot_precission_playSpd_a;
		//setMaxSpeed(spd);
		//acc=5000;

		//spd=map(pot_precission_playSpd_a, 1340,8000, 10,1000);
		acc=map(pot_precission_playSpd_a, 1340,8000, 4000,120000); // max for microsteps 1/8 ~= 25k , for microsteps 1/16 ~= 80-110k @ a4988 max current
		spd=0;
		//setAcceleration(acc);
		stepper->setSpeed(10+spd);
		stepper->setAcceleration(acc);
		stepper->applySpeedAcceleration();

		if(abs(pos-pos_last)>40)
		{
			pos_last=pos;
			moveTo(pos);
		}
		//stepper.run();

		Serial.print(pos);Serial.print("\t");
		Serial.print(spd);Serial.print("\t");
		Serial.print(acc);Serial.print("\t");
		Serial.println();
		delay(100);
	}
	
}