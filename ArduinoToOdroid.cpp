#include <ArduinoToOdroid.h>


/*
 * Software License Agreement
 *
 * Copyright (c) 2021, Applied Robotics, Inc.
 * All rights reserved.
 *
 */

Odroid::Odroid(){
  mainString = " ";
  WheelDiffVelocity[0] = 0;
  WheelDiffVelocity[1] = 0;
  TargetPoint[0] = 0.0;
  TargetPoint[1] = 0.0;
  TargetPoint[2] = 0.0;
  TargetVelocity[0] = 0;
  TargetVelocity[1] = 0;
  TargetVelocity[2] = 0;
   Serial.begin(57600);
   
//   Serial.setTimeout(1);
 }
Odroid::~Odroid(){
  
  Serial.end();
 }

void Odroid::IKsolv(){

  if (isOmni){

    WheelAngularVelocity[0] = (1/R*(TargetVelocity[0]
      -TargetVelocity[1]+(L1+L2)*(TargetVelocity[2]/2))*1);

      WheelAngularVelocity[1] = (1/R*(TargetVelocity[0]
        +TargetVelocity[1]-(L1+L2)*(TargetVelocity[2]/2))*(-1));

      WheelAngularVelocity[2] = (1/R*(-TargetVelocity[0]
        -CurrentVelocity[1]-(L1+L2)*(TargetVelocity[2]/2))*(-1));

      WheelAngularVelocity[3] = (1/R*(-TargetVelocity[0]
        +TargetVelocity[1]+(L1+L2)*(TargetVelocity[2]/2))*1);
  }
  else{

    WheelDiffVelocity[0] = (2*TargetVelocity[0] - L1*TargetVelocity[2])/(2*R);
      WheelDiffVelocity[1] = (2*TargetVelocity[0] + L1*TargetVelocity[2])/(2*R);
  }
 }
void Odroid::FKsolv(){
  if (isOmni){
      RealVelocity[0] = (CurrentVelocity[0]+CurrentVelocity[1]+CurrentVelocity[2]+CurrentVelocity[3])*(R/4);
      RealVelocity[1] = (-CurrentVelocity[0]-CurrentVelocity[1]+CurrentVelocity[2]+CurrentVelocity[3])*(R/4);
      RealVelocity[2] = (-CurrentVelocity[0]+CurrentVelocity[1]-CurrentVelocity[2]+CurrentVelocity[3])*(R/(2*(L1+L2)));
  }
  else{
      RealVelocity[0] = (R*(CurrentVelocity[0]+CurrentVelocity[1]))/2;
      RealVelocity[1] = 0.0;
      RealVelocity[2] = (R*(CurrentVelocity[0]-CurrentVelocity[1]))/L1;
  }
}

void Odroid::testParsing(){
  byte dividerIndex = mainString.indexOf(';');
  String buf1 = mainString.substring(0, dividerIndex);
  parsingLocal(buf1);
  mainString = mainString.substring(dividerIndex + 1);
  if (mainString != NULL) testParsing();
}

void Odroid::parsingLocal(String tempString){
  
  byte dividerIndex = tempString.indexOf(':');
  String buf1 = tempString.substring(0, dividerIndex);
  String buf2 = tempString.substring(dividerIndex + 1);


  if (buf1.equals("tv")){
    if(isOmni){

      dividerIndex = buf2.indexOf(',');
      buf1 = buf2.substring(0, dividerIndex);
      TargetVelocity[0] = buf1.toFloat();
      String buf3 = buf2.substring(dividerIndex + 1);
      dividerIndex = buf3.indexOf(',');
      buf1 = buf3.substring(0, dividerIndex);
      buf2 = buf3.substring(dividerIndex + 1);
      TargetVelocity[1] = buf1.toFloat();
      TargetVelocity[2] = buf2.toFloat();
    }
    else{

//      dividerIndex = buf2.indexOf(',');
//      buf1 = buf2.substring(0, dividerIndex);
//      String buf3 = buf2.substring(dividerIndex + 1);
//      TargetVelocity[0] = buf1.toFloat();
//      TargetVelocity[1] = 0;
//      TargetVelocity[2] = buf3.toFloat();

      dividerIndex = buf2.indexOf(',');
      buf1 = buf2.substring(0, dividerIndex);
      TargetVelocity[0] = buf1.toFloat();
      String buf3 = buf2.substring(dividerIndex + 1);
       dividerIndex = buf3.indexOf(',');
      buf1 = buf3.substring(0, dividerIndex);
      buf2 = buf3.substring(dividerIndex + 1);
      TargetVelocity[1] = 0;
      TargetVelocity[2] = buf2.toFloat();
    }
  }

  if (buf1.equals("rp")){

    dividerIndex = buf2.indexOf(',');
    buf1 = buf2.substring(0, dividerIndex);
    CurrentPoint[0] = buf1.toFloat();
    String buf3 = buf2.substring(dividerIndex + 1);
    dividerIndex = buf3.indexOf(',');
    buf1 = buf3.substring(0, dividerIndex);
    buf2 = buf3.substring(dividerIndex + 1);
    CurrentPoint[1] = buf1.toFloat();
    CurrentPoint[2] = buf2.toFloat();
  }

  if(buf1.equals("s")){
    state = buf2;
  }
 }
void Odroid::parsingGlobal(){

  String bufString = Serial.readString();
  byte dividerIndex = bufString.indexOf(';');
  String buf1 = bufString.substring(0, dividerIndex);
  String buf2 = bufString.substring(dividerIndex + 1);
  parsingLocal(buf1);
  parsingLocal(buf2);
 
 }
bool Odroid::onReceivingListener(){
  if (Serial.available() > 10){
    //parsingGlobal();
    mainString = Serial.readString();
    testParsing();
    return true;
  }else return false;
 }

bool Odroid::pushCurrentPoint(){
   // Итоговая строка с данными, отправляемая в одроид.
  Serial.print("tp,");
  for(int i = 0; i<3; i++){
    Serial.print(TargetPoint[i]);
    if(i < 2)
      Serial.print(",");
  }
  Serial.print(";");
  return true;    // Проверка на дошло/не дошло (доработать?).
 }
bool Odroid::pushCurrentVelocity(){
  String inputString = "cv,";   // Итоговая строка с данными, отправляемая в одроид.
  Serial.print("cv,");
  for(int i = 0; i<3; i++){
    Serial.print(RealVelocity[i]);
    if(i < 2)
      Serial.print(",");
  }
  Serial.print(";");
  return true;    // Проверка на дошло/не дошло (доработать?).
 }
bool Odroid::pushString(){
  pushCurrentPoint();
  FKsolv();
  pushCurrentVelocity();
  Serial.println();
  return true;
}
float* Odroid::getTargetVelocity(){
  return TargetVelocity;
 }
float* Odroid::getCurrentPoint(){
  return CurrentPoint;
 }
float* Odroid::getWheelAngularVelocity(){
  return WheelAngularVelocity;
}
float* Odroid::getWheelDiffVelocity(){
  IKsolv();
  return WheelDiffVelocity;
}

bool Odroid::init(String type){
  if (type.equals("omni")){

    isOmni = true;
    R  = 0.05; L1 = 0.25; L2 = 0.25;

    return true;
  }
  else if(type.equals("diff")){

    isOmni = false;
    R  = 0.05; L1 = 0.25; L2 = 0;

    return true;
  }
  else{
    return false;     // Какая-то ошибка.
  }
 }
bool Odroid::init(String type, float R){
  if (type.equals("omni")){

    isOmni = true;
    this->R  = R; L1 = 0.25; L2 = 0.25;

    return true;
  }
  else if(type.equals("diff")){

    isOmni = false;
    this->R  = R; L1 = 0.25; L2 = 0;

    return true;
  }
  else{
    return false;     // Какая-то ошибка.
  }
 }
bool Odroid::init(String type, float R, float L1){
  if (type.equals("omni")){

    isOmni = true;
    this->R  = R; this->L1 = L1; L2 = 0.25;

    return true;
  }
  else if(type.equals("diff")){

    isOmni = false;
    this->R  = R; this->L1 = L1; L2 = 0;

    return true;
  }
  else{
    return false;     // Какая-то ошибка.
  }
 }
bool Odroid::init(String type, float R, float L1, float L2){
  if (type.equals("omni")){

    isOmni = true;
    this->R  = R; this->L1 = L1; this->L2 = L2;

    return true;
  }
  else if(type.equals("diff")){

    isOmni = false;
    this->R  = R; this->L1 = L1; L2 = 0;

    return true;
  }
  else{
    return false;     // Какая-то ошибка.
  }
 }

//--------------------------------- Пока не реализуем ---------------------------------//
// bool Odroid::moveTo(float x, float y, float alpha){
    
//  String inputString = "moveto,";   // Итоговая строка с данными, отправляемая в плату и парсящаяся там.
//  inputString += x; inputString += ",";
//  inputString += y; inputString += ",";
//  inputString += alpha;
  
          
//  Serial.println(inputString);
  
  
//  return true;          // Возврат успеха/провала на Ардуино (доработать).
//  }
 
//  // Реализация функции включения навигации.
// bool Odroid::startSLAM(){
   
//  String inputString = "startSLAM";   // Итоговая строка с данными, отправляемая в плату и парсящаяся там.
//  Serial.println(inputString);
  
  
//  return true;          // Возврат успеха/провала на Ардуино (доработать).
   
//  }
  
//  // Реализация функции выключения навигации.
// bool Odroid::stopSLAM(){

//  String inputString = "stopSLAM";    // Итоговая строка с данными, отправляемая в плату и парсящаяся там.
//  Serial.println(inputString);
  
//  return true;          // Возврат успеха/провала на Ардуино (доработать).
   
//  }
 
//    // Реализация функции включения навигации.#include <ArduinoToOdroid.h>

/*
 * Software License Agreement
 *
 * Copyright (c) 2021, Applied Robotics, Inc.
 * All rights reserved.
 *
 */
