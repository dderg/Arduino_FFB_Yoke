 #if defined(LANGUAGE_EN) // language englisch

/******************************************
  Into
*******************************************/
void LcdPrintIntro() {
  lcd.setCursor(0, 0);
  lcd.print(F("Hi"));
}

void LcdPrintPosition() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Pitch: "));
  lcd.print(counterPitch.read());
  }

/******************************************
  show adjustment values (max force, pwm)
*******************************************/
void LcdPrintAdjustmendValues() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Roll  Force: "));
  lcd.print(adjForceMax[MEM_ROLL]);

  lcd.setCursor(0, 1);
  lcd.print(F("  PWM Mi-Ma: "));
  lcd.print(adjPwmMin[MEM_ROLL]);
  lcd.print(F("-"));
  lcd.print(adjPwmMax[MEM_ROLL]);
    
  lcd.setCursor(0, 2);
  lcd.print(F("Pitch Force: "));
  lcd.print(adjForceMax[MEM_PITCH]);
  
  lcd.setCursor(0, 3);
  lcd.print(F("  PWM Mi-Ma: "));
  lcd.print(adjPwmMin[MEM_PITCH]);
  lcd.print(F("-"));
  lcd.print(adjPwmMax[MEM_PITCH]);
}

/******************************************
  show debug mode
*******************************************/
void LCDPrintDebugMode(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Debug Mode: On"));
}

/******************************************
  show calibration middle
 *******************************************/
void LcdPrintCalibrationMiddle(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("prs btn to clbrt"));
}

/******************************************
  show calibration start
*******************************************/
void LcdPrintCalibrationAxesStart(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Please move all axes"));
  lcd.setCursor(0, 1);
  lcd.print(F("to the end positions"));
}

/******************************************
  show calibration axes values update
*******************************************/
void LcdPrintCalibrationAxesUpdate(int poti_roll_min,int poti_roll_max, int poti_pitch_max, int poti_pitch_min){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Left  end: ")); //Left  end
  lcd.print(poti_roll_min);
  
  lcd.setCursor(0, 1);
  lcd.print(F("Right end: ")); //right end
  lcd.print(poti_roll_max);

  lcd.setCursor(0, 2);
  lcd.print(F("Up    end: ")); // up end
  lcd.print(poti_pitch_max);

  lcd.setCursor(0, 3);
  lcd.print(F("Down  end: ")); // down end
  lcd.print(poti_pitch_min);
}
#endif
