
#include <FlexCAN_T4.h>
#include "Motor_Control_Tmotor.h"
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

CAN_message_t msgR;

int Motor_ID = 1;
int CAN_ID = 3;
double torque_command = 0;
double velocity_command = 0;
double position_command = 0;

float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff = 0;

Motor_Control_Tmotor m1(Motor_ID, CAN_ID);

double current_time = 0;
unsigned long beginning = 0;

void setup() {
  Serial.begin(115200);
  initial_CAN();
  beginning = micros();
}

void loop() {
  current_time = micros() - beginning;
  position_command = 90.0 * sin(current_time / 1000000.0 );
  Position_Control_Example();
  delay(2);
}

void Position_Control_Example()
{
  p_des = position_command * PI / 180;
  v_des = 0; //dont change this
  kp = 30; //max 450 min 0
  kd = 1.5; //max 5 min 0
  t_ff = 0; //dont change this
  m1.send_cmd( p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();

  double v1 = 90;
  double v2 = -v1;
  Serial.print(v1);
  Serial.print("   ");
  Serial.print(v2);
  Serial.print("   ");
  Serial.print(m1.pos * 180 / PI);
  Serial.print("   ");
  Serial.print(position_command);
  Serial.println("   ")                 ;

}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
  m1.initial_CAN();
  m1.exit_control_mode();
  delay(200);
  m1.exit_control_mode();
  delay(1000);
  m1.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  position_command = 0;
  Position_Control_Example();
  //delay(200);
}

void receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    int id = msgR.buf[0];
    Serial.print(msgR.id, HEX );
    if (id == Motor_ID)
    {
      m1.unpack_reply(msgR);
    }
  }
}
