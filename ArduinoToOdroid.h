#pragma once
#include <Arduino.h>


/*
 * Software License Agreement
 *
 * Copyright (c) 2021, Applied Robotics, Inc.
 * All rights reserved.
 *
 */

class Odroid {

private:

  bool isOmni;

  
public:
  
  Odroid(); //Конструктор и деструктор.
   ~Odroid();

  String state;
  String mainString;

  bool onReceivingListener(); // Функция-слушатель для принятия данных с одроида.
  float* CurrentVelocity = new float[3];  // Текущая скорость vX, vY, wZ.
  float* TargetVelocity  = new float[3];  // Заданная скорость.vX, vY, wZ.
  float* CurrentPoint    = new float[3];  // Текущая точка.
  float* TargetPoint     = new float[3];  // Заданная точка.
  float* RealVelocity    = new float[3];

  float* WheelAngularVelocity = new float[4]; // Скорости колес омниплатформы.
  float* WheelDiffVelocity    = new float[2]; // Скорости колес дифф.платформы.
 
  float R, L1, L2;            // Радиус колес, расстояние между ними и между задними.

  void parsingLocal(String tempString); // Парсер строки.
  void parsingGlobal();
  void IKsolv();    // решение ОЗК (из таргетных скоростей платформы получаем таргетные скорости колес).
  void FKsolv();   // решение ПЗК (из реальных скоростей колес получаем реальную скорость платформы)
  float* getWheelAngularVelocity(); // Получение скоростей колес омниплатформы.
  float* getWheelDiffVelocity();    // Получение скоростей колес дифф.платформы.
  float* getTargetVelocity();     // Получение целевой скорости робота.
  float* getCurrentPoint();     // Получение текущей скорости робота.
  float* getStateRobot();       // Получение всех данных робота. (Пока непонятно, как реализовать).
  void testParsing();
  
  bool init(String name); // Инициализация робота и перегрузки функций.
  bool init(String name, float R);
  bool init(String name, float R, float L1);
  bool init(String name, float R, float L1, float L2);
  
  bool pushCurrentPoint();    // Отправка текущей точки.
  bool pushCurrentVelocity();   // Отправка текущей скорости.
  bool pushString();
  //--------------------------------- Пока не реализуем ---------------------------------//
  // bool startSLAM();              // Запуск навигации.
  // bool stopSLAM();             // Остановка навигации.
  // bool moveTo(float x, float y, float alpha);  // Двигаться в данную точку.

};
