#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"

//global variabless
pros::task_t lift_task;

int lift_target;

void lift(void *ignore)
{
 while (true)
 {
    float Kp = 0.45;  //TESTING NEEDED
    float Kd = 0.7;		//TESTING NEEDED
    float Ki = 0.25;    //TESTING NEEDED
    float proportional, integral, derivative;

    float error;
    int last_error = 0;
    float final_power;
    float encoder_avg;

    int timeout = 100;

    int max_speed = 110; //90

    int failsafe = 2000;    //2000
    int initial_millis = pros::millis();

    int integral_limit = 30;
    unsigned int net_timer;

    bool timer_turn = true;


    net_timer = pros::millis() + timeout;

      while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

        encoder_avg = potentiometer.get_value();
        error = lift_target - encoder_avg;


        derivative = (error - last_error)*Kd;
        last_error = error;
        proportional = error*Kp;

        final_power = proportional + derivative;

            if (final_power > max_speed){
              final_power = max_speed;
            }

            if (final_power < -max_speed){
              final_power = -max_speed;
            }


        arm.move(final_power);


        if (timer_turn == true)
        {
          net_timer = pros::millis() + timeout;
        }

        if (fabs(error) < 2)
        {   //if less than 2 degrees
          timer_turn = false;
        }


        pros::delay(20); //20

  }
arm.move(0);
//     printf("correction turn %f\n", correction_turn);
// printf("encoder avg %d\n", (left_encoder.get_value() + right_encoder.get_value())/2);
}

    }
