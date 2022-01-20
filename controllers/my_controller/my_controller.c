/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/gyro.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28
//#define robot

/* sleep(float duration,robot robot){
  float end_time=robot.getTime()+duration;
  while (robot.step(TIME_STEP) != -1 && robot.getTime() < end_time)
    continue;
}
*/
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
 void wb_gyro_enable(WbDeviceTag tag, int sampling_period);
 
 void go_forward(WbDeviceTag left_motor , WbDeviceTag right_motor){
  
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
   
 }
 
 void turn_around(WbDeviceTag left_motor , WbDeviceTag right_motor, float duration ){
    // once we are near an object turn in place  
    float left_speed  = 0.25*MAX_SPEED;
    float right_speed = -0.25*MAX_SPEED;
    
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    
    // duration is our best guess for how long it'll turn 180 degrees 
    float end_time= wb_robot_get_time()+duration;
    while (wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < end_time)
    continue;
 }

void final_state(WbDeviceTag left_motor, WbDeviceTag right_motor, double ps_values [], WbDeviceTag ps[]){
  while (wb_robot_step(TIME_STEP) != -1) {
    for (int i = 0; i < 8 ; i++)
       ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    
    // if we are in the 5cm, spin
    if(ps_values[0]>=100 || ps_values[7]>=100){
      wb_motor_set_velocity(left_motor, 0.25*MAX_SPEED);
      wb_motor_set_velocity(right_motor, -0.25*MAX_SPEED);
    }else if(ps_values[5] >= 90){
      final_state_2(left_motor, right_motor, ps_values, ps);
      return;
    } else {
      // go forward 
      go_forward(left_motor, right_motor);
    }
  }
}

void final_state_2(WbDeviceTag left_motor, WbDeviceTag right_motor, double ps_values [], WbDeviceTag ps[]){
   while (wb_robot_step(TIME_STEP) != -1) {
     for (int i = 0; i < 8 ; i++)
       ps_values[i] = wb_distance_sensor_get_value(ps[i]);
       
     if(ps_values[5] >= 90){
        go_forward(left_motor, right_motor);
      } else {
       return;
      }
   
   }
}
 
int main(int argc, char **argv) {
// initialize the webot api 
  wb_robot_init();
  
  // initialize the device 
  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };  
  
  for (i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  // initialize the motos 
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  int collisions = 0;
  while (wb_robot_step(TIME_STEP) != -1) {

  // read sensor outputs
   double ps_values[8];
   for (i = 0; i < 8 ; i++)
     ps_values[i] = wb_distance_sensor_get_value(ps[i]);
     
   /* 
      States 1 and 2
      our best guess for within 0.05 m of that objects 
      0 and 7 are the front sensors 
      as we get closer the values increase 
   */ 
   if ((ps_values[0]>=100 || ps_values[7]>=100) && collisions < 1){
        
     // once we are near an object turn in place  
     turn_around(left_motor, right_motor, 2.825);
     
     go_forward(left_motor, right_motor);
     
     collisions = collisions + 1;
   }
   
   // State 3
   if(collisions == 1){
      // final_state(left_motor, right_motor, ps_values[5]);          
      final_state(left_motor, right_motor, ps_values, ps);
      collisions = collisions + 1;
    }
    
    // Final State 4
    if(collisions > 1){
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
