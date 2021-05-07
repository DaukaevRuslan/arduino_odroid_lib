#include "ArduinoToOdroid.h"
#include <DynamixelWorkbench.h>
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" 
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   
#define BAUDRATE  1000000  // скорость опроса датчиков
#define DXL_1    1  // передний левый
#define DXL_2    2  // передний правый
DynamixelWorkbench dxl_wb;
uint8_t dxl_id[4] = {DXL_1, DXL_2};
Odroid odroid;
float* wheelDiffVelocity = new float[2];
bool Flag = true;
int state = 0;
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(5);
  // put your setup code here, to run once:
  odroid.init("diff", 0.033, 0.16);
  uint16_t model_number = 0;
  bool result = false ;
  const char *log;  
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  for(int i = 0; i<2; i++){
    result = dxl_wb.ping(dxl_id[i], &model_number, &log); 
    dxl_wb.wheelMode(dxl_id[i], 0, &log);  
  }
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Velocity");
  
}

void loop() {
  // put your main code here, to run repeatedly:
   wheelDiffVelocity = odroid.getWheelDiffVelocity();
   move_os(wheelDiffVelocity[0], wheelDiffVelocity[1]);
   read_joint_state(odroid.CurrentVelocity);
   if (odroid.state.equals("empty") && state == 0){
    odroid.TargetPoint[0] = 0.4;
    odroid.TargetPoint[1] = 0.1;
    odroid.TargetPoint[2] = 0.0;
    state = 1;
   }
   if (odroid.state.equals("finished") && state == 1){
    odroid.TargetPoint[0] = 0.0;
    odroid.TargetPoint[1] = 0.0;
    odroid.TargetPoint[2] = 0.0;
    state = 2;
   }
   odroid.onReceivingListener();
   odroid.pushString();
   delay(100);
}
void move_os(float vr,float vl){//функция движения, принимает необходимую скорость по оси X, Y и вокруг оси Z
  const char *log;
  bool result = false;
  int64_t wheel_value[2] = {0, 0};
  double wheel_angular_velocity[2] = {0.0, 0.0};
  int32_t wheel_angular_velocity_real[2] = {0, 0};
  wheel_angular_velocity[0] = vr;//расчитываем скорости для каждого из моторов с учетом направления его вращения
  wheel_angular_velocity[1] = vl;
  for(int i = 0; i < 2; i++){
    wheel_angular_velocity_real[i] = (int32_t)(wheel_angular_velocity[i]*9.24/0.229);
//    result = dxl_wb.writeRegister((uint8_t)dxl_id[i], "Goal_Velocity", wheel_angular_velocity_real[i]);
  }
  result = dxl_wb.syncWrite((uint8_t)0, &wheel_angular_velocity_real[0], &log);
 // Serial.println(ztring(wheel_angular_velocity_real[0])+String(wheel_angular_velocity_real[1])+String(wheel_angular_velocity_real[2])+String(wheel_angular_velocity_real[3]));
//  / Serial.println(String(wheel_angular_velocity[0])+String(wheel_angular_velocity[1])+String(wheel_angular_velocity[2])+String(wheel_angular_velocity[3]));
}
void read_joint_state(float* current_velocity){
  const char* log;
  for(int i = 0; i<2; i++){
    uint32_t get_data = 0;
    dxl_wb.readRegister(dxl_id[i], (uint16_t)128, (uint16_t)4, &get_data, &log);
    current_velocity[i] = (int32_t)get_data*0.229/9.24;
  }
}
