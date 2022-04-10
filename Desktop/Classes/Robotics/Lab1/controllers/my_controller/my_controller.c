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

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
// read sensor outputs
   double ps_values[8];
   for (i = 0; i < 8 ; i++)
     ps_values[i] = wb_distance_sensor_get_value(ps[i]);
     
     
     if (ps_values[0]>=100 || ps_values[7]>=100){
       float left_speed  = 0.25*MAX_SPEED;
       float right_speed = -0.25*MAX_SPEED;

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        /*
        vector<vector<vector<string>>> some_vector
        double gyrovalues[] = wb_gyro_get_values();
        zG=gyr
        */
        //printf(wb_gyro_get_values());
        
        
       float duration=2.825;
        float end_time= wb_robot_get_time()+duration;
        while (wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < end_time)
          continue;
      wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, -1*right_speed);
        
        // if(ps_values[3]==ps_values[4]){
        // wb_motor_set_velocity(left_motor,0);
        // wb_motor_set_velocity(right_motor,0);
        // }
        
     }
  
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
