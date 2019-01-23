#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <math.h>
/**
 * @file    main.c
 * @brief
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

///Katri Rasio,  Eveliina Schuurman, Jesse Väärälä///

//////////DEBUGGER///////////////////////////////
#if 0

    struct sensors_ ref;
    struct sensors_ dig;

    TickType_t tick;
    int counter = 0;
    struct accData_ data;
    uint8_t LSM303D_Start(void);


    void zmain(void) {
        while(SW1_Read() == 1) {
        vTaskDelay(100);


            if(SW1_Read() == 0) {

                if(!LSM303D_Start()){
                    printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
                    while(1) vTaskDelay(10);
                }
                else printf("Device Ok...\n");

                vTaskDelay(500);

                IR_Start();
                vTaskDelay(100);
                printf("IR started\n");

                IR_flush();
                vTaskDelay(500);
                printf("IR flushed\n");

                reflectance_start();
                vTaskDelay(100);
                printf("Reflect sensors started\n");

                motor_start();
                vTaskDelay(100);
                printf("Motors started\n");

                Ultra_Start();
                vTaskDelay(100);
                printf("Ultras started\n");

                printf("Waiting for IR..\n");
                IR_wait();
                BatteryLed_Write(1);



                for (;;) {
                    tick = xTaskGetTickCount();
                    reflectance_read(&ref);
                    reflectance_digital(&dig);
                    LSM303D_Read_Acc(&data);


                    // gets data for excel
                    if (counter % 50) {
                        //printf("%u\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", tick, ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3, dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
                        print_mqtt("Zumo046/%10d %10d %10d\n", data.accX, data.accY, data.accZ);
                    }
                    counter++;
                }
            }
        }
    }
#endif

//////////VIIVAN SEURANTA///////////////////////////////
#if 0

    // parameters for drive are set here
    int speed_on_track = 220;
    int speed;
    float weight1 = 1;
    float weight2 = 9;
    float weight3 = 10;
    int alt_thresh = 12000;
    //

    float L3, L2, L1, R1, R2, R3;
    float l3norm, l2norm, l1norm, r1norm, r2norm, r3norm;
    float lavg;
    float ravg;
    float L_norm, R_norm;
    float L, R;
    float l3_lim = 0, l2_lim = 0, l1_lim = 0, r1_lim = 0, r2_lim = 0, r3_lim = 0;
    int L_sum, R_sum;
    int L_power = 0, R_power = 0;
    int d;

    float sensor_max = 25000; // default values without calibration
    float sensor_min = 4000;

    bool flag_L, flag_R;

    struct sensors_ ref;
    struct sensors_ dig;
    TickType_t start, end;

    int dir = 0;


// line follower function
void follow_line (void) {

    //get data from sensors for control
    reflectance_read(&ref);
    reflectance_digital(&dig);

    //limiting data between sensor_min & sensor_max
    l3_lim = ref.l3;
    l2_lim = ref.l2;
    l1_lim = ref.l1 + 2000; // LED difference compensated
    r1_lim = ref.r1 + 2000;
    r2_lim = ref.r2;
    r3_lim = ref.r3;

    if (l1_lim < sensor_min) {
        l1_lim = sensor_min;
    } else if (l1_lim > sensor_max) {
        l1_lim = sensor_max;
    }
    if (l2_lim < sensor_min) {
        l2_lim = sensor_min;
    } else if (l2_lim > sensor_max) {
        l2_lim = sensor_max;
    }
    if (l3_lim < sensor_min) {
        l3_lim = sensor_min;
    } else if (l3_lim > sensor_max) {
        l3_lim = sensor_max;
    }
    if (r1_lim < sensor_min) {
        r1_lim = sensor_min;
    } else if (r1_lim > sensor_max) {
        r1_lim = sensor_max;
    }
    if (r2_lim < sensor_min) {
        r2_lim = sensor_min;
    } else if (r2_lim > sensor_max) {
        r2_lim = sensor_max;
    }
    if (r3_lim < sensor_min) {
        r3_lim = sensor_min;
    } else if (r3_lim > sensor_max) {
        r3_lim = sensor_max;
    }

    // assing weights to sensor data
    L3 = (l3_lim-sensor_min)*weight3;
    L2 = (l2_lim-sensor_min)*weight2;
    L1 = (l1_lim-sensor_min)*weight1;
    R1 = (r1_lim-sensor_min)*weight1;
    R2 = (r2_lim-sensor_min)*weight2;
    R3 = (r3_lim-sensor_min)*weight3;

    // set r2 & l2 to max if only one sensor sees black
    if ((dig.r3 == 1 || dig.l3 == 1) && (ref.l2 < (sensor_max-1000) && dig.l1 == 0 && dig.r1 == 0 && ref.l2 < (sensor_max-1000))) {
        if (dig.l3 == 1) {
            L2 = (sensor_max-sensor_min)*weight2;
        }
        if (dig.r3 == 1) {
            R2 = (sensor_max-sensor_min)*weight2;
        }
    }

    L_sum = L3 + L2 + L1;
    R_sum = R1 + R2 + R3;

    // normalize data from sensors
    L_norm = ((float) L_sum)/((sensor_max-sensor_min) * (weight1 + weight2 + weight3));
    R_norm = ((float) R_sum)/((sensor_max-sensor_min) * (weight1 + weight2 + weight3));

    //counting motor values for left & right
    L_power = (speed-(L_norm*speed));
    R_power = (speed-(R_norm*speed));

    //limiter for motor power values to avoid negative values
    if (L_power < 0) {
        L_power = 0;
    }
    if (R_power < 0) {
        R_power = 0;
    }
    //finally: drive according to calculations above
    motor_turn(L_power, R_power, 10);
}

// median filter function
uint16_t middle_of_3(uint16_t a, uint16_t b,uint16_t c) {
    uint16_t middle;

    if ((a <= b) && (a <= c)) {
        middle = (b <= c) ? b : c;
    } else if ((b <= a) && (b <= c)) {
        middle = (a <= c) ? a : c;
    } else {
        middle = (a <= b) ? a : b;
    }
    return middle;
}
void turn_90(int dir) {
    int x, y;
    //turn left
    if (dir == 0) {
        x = 1;
        y = 0;
    }
    //turn right
    if (dir == 1) {
        x = 0;
        y = 1;
    }
    MotorDirLeft_Write(x);
    MotorDirRight_Write(y);

    do {
        reflectance_digital(&dig);
        motor_turn(155,155,1);
    } while (dig.l1 == 0 || dig.r1 == 0);
    motor_turn(0,0,1);

    MotorDirLeft_Write(0);
    MotorDirRight_Write(0);
    motor_forward(0,0);

    vTaskDelay(20);

}


void zmain(void) {

    vTaskDelay(10000);
    BatteryLed_Write(1);
    while(SW1_Read() == 1) {

        vTaskDelay(100);
        if(SW1_Read() == 0) {

            vTaskDelay(500);
            printf("START\n");

            //starting reflectance sensors
            reflectance_start();
            //printf("Reflectance sensors started\n");
            //starting motors
            motor_start();
            //printf("Motors started\n");

            // set center sensor threshold to 11000 and others to 9000
            reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);

            vTaskDelay(500);

            //setup sensor_min (white)
            reflectance_read(&ref);

            uint16_t a = ref.r3;
            uint16_t b = ref.r2;
            uint16_t c = ref.r1;
            float sensor_min = (middle_of_3(a,b,c) + 1000);

            //setup sensor_max (black)
            a = ref.l3;
            b = ref.l2;
            c = ref.l1;
            float sensor_max = (middle_of_3(a,b,c) - 4000); // -4000 narrows the range, so it hits max more easily

            //printf("Sensor min set to: %f\n", sensor_min);
            //print_mqtt("Zumo046","Sensor min set to: %f", sensor_min);
            //printf("Sensor max at %f\n", sensor_max);
            //print_mqtt("Zumo046","Sensor max set to: %f", sensor_max);

            //variables for all-black-counter (dig. = 111111)
            int ab_counter = 0;

            IR_Start();
            //printf("IR Started\n");
            //print_mqtt("Zumo046","IR Started");
            vTaskDelay(100);
            IR_flush();

            //debug printing for waiting IR command
            printf("Waiting for IR...\n");
            //print_mqtt("Zumo046","Waiting for IR...");
            IR_wait();

            for(;;){
                //get data from sensors for control
                reflectance_read(&ref);
                reflectance_digital(&dig);

                //speed before first line
                if (ab_counter == 0) {
                    speed = 40;
                } else {
                    speed = speed_on_track;
                }

                //set flags
                if (dig.l3 == 1) {
                    flag_L = true;
                    flag_R = false;
                    dir = 0;
                }
                if (dig.r3 == 1) {
                    flag_R = true;
                    flag_L = false;
                    dir = 1;
                }

                //turn if all sensors white (dig. = 000000) using alternate threshold
                if (ref.l3 < alt_thresh && ref.l2 < alt_thresh && ref.l1 < (alt_thresh + 2000) && ref.r1 < (alt_thresh + 2000) && ref.r2 < alt_thresh &&  ref.r3 < alt_thresh) {

                    // flag based turning replaced with turn_90 function
//                    do {
//
//                        int right = 0;
//                        int left = 0;
//                        if (flag_L == true) {
//                            right = speed;
//                            left = 0;
//                        }
//                        if (flag_R == true) {
//                            right = 0;
//                            left = speed;
//                        }
//                        motor_turn(left, right, 10);
//                        reflectance_digital(&dig);
//
//                    } while (dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0);

                    turn_90(dir);
                }

                // calling follow_line function
                follow_line();


                //encountering all black
                if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1){
                    //print_mqtt("Zumo046","It's all black, mate and counter is %d", ab_counter);
                    if (ab_counter == 0) {
                        //stopping at first line
                        motor_forward(0, 0);

                        print_mqtt("Zumo046/ready","line");
                        IR_wait();
                         //start tick counting
                        start = xTaskGetTickCount();

                        print_mqtt("Zumo046/start"," %d", start);
                        speed = speed_on_track;

                    } // this is the end, my only friend
                    else if (ab_counter == 2) {
                        end = xTaskGetTickCount();

                        motor_stop();
                        print_mqtt("Zumo046/stop","%d", end);
                        int aika = end - start;
                        print_mqtt("Zumo046/time","%d", aika);
                        break;
                    }
                    do {
                        reflectance_digital(&dig);
                        motor_forward(speed,1);
                    } while (dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1);
                    ab_counter = ab_counter + 1;
                }
            }
        }
    }
}
#endif

//////////LABYRINTTI///////////////////////////////
 #if 0
    float L3, L2, L1, R1, R2, R3;
    float l3norm, l2norm, l1norm, r1norm, r2norm, r3norm;
    float lavg, ravg;
    float L, R;
    int L_power = 0, R_power = 0;
    int d, a, b, c;

    float sensor_max = 25000;
    float sensor_min = 4000;

    bool flag_L, flag_R, flag_F;

    struct sensors_ ref;
    struct sensors_ dig;
    TickType_t start, end, racetime;

uint16_t middle_of_3(uint16_t a, uint16_t b,uint16_t c) {
    uint16_t middle;

    if ((a <= b) && (a <= c)) {
        middle = (b <= c) ? b : c;
    } else if ((b <= a) && (b <= c)) {
        middle = (a <= c) ? a : c;
    } else {
        middle = (a <= b) ? a : b;
    }
    return middle;
}

void turn_90(int dir) {
    int x, y;
    //turn left
    if (dir == 0) {
        x = 1;
        y = 0;
    }
    //turn right
    if (dir == 1) {
        x = 0;
        y = 1;
    }
    MotorDirLeft_Write(x);
    MotorDirRight_Write(y);

    // turn away from the line
    if(dig.l1 == 1 || dig.r1 == 1) {
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,40);
        } while (dig.l1 == 1 || dig.r1 == 1);
        motor_turn(0,0,1);
    }
    // turn until line appears
    do {
        reflectance_digital(&dig);
        motor_turn(185,185,1);
    } while (dig.l1 == 0 || dig.r1 == 0);
    motor_turn(0,0,1);

    MotorDirLeft_Write(0);
    MotorDirRight_Write(0);
    motor_forward(0,0);

    vTaskDelay(100);
    // check for obstacle at left
    for (int i = 0; i < 3; i++) {
            c = b;
            b = a;
            a = Ultra_GetDistance();
    }

    d = middle_of_3(a, b, c);
    //print_mqtt("Zumo046","%d", d);
    if (d < 15) {

        MotorDirLeft_Write(0);
        MotorDirRight_Write(1);
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,40);
        } while (dig.l1 == 1 || dig.r1 == 1);
        motor_turn(0,0,1);
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,1);
        } while (dig.l1 == 0 || dig.r1 == 0);
        motor_turn(0,0,1);
        flag_L = false;

        MotorDirLeft_Write(0);
        MotorDirRight_Write(0);
        motor_forward(0,0);
    }

    // check for obstacle at front
    for (int i = 0; i < 3; i++) {
            c = b;
            b = a;
            a = Ultra_GetDistance();
    }
    d = middle_of_3(a, b, c);
    //print_mqtt("Zumo046","%d", d);
    if (d < 15) {

        MotorDirLeft_Write(0);
        MotorDirRight_Write(1);
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,40);
        } while (dig.l1 == 1 || dig.r1 == 1);
        motor_turn(0,0,1);
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,1);
        } while (dig.l1 == 0 || dig.r1 == 0);
        motor_turn(0,0,1);
        flag_L = false;

        MotorDirLeft_Write(0);
        MotorDirRight_Write(0);
        motor_forward(0,0);
    }

    //check for obstacle at right
    for (int i = 0; i < 3; i++) {
            c = b;
            b = a;
            a = Ultra_GetDistance();
    }
    d = middle_of_3(a, b, c);
    //print_mqtt("Zumo046","%d", d);
    if (d < 15) {

        MotorDirLeft_Write(0);
        MotorDirRight_Write(1);
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,40);
        } while (dig.l1 == 1 || dig.r1 == 1);
        motor_turn(0,0,1);
        do {
            reflectance_digital(&dig);
            motor_turn(185,185,1);
        } while (dig.l1 == 0 || dig.r1 == 0);
        motor_turn(0,0,1);
        flag_L = false;

        MotorDirLeft_Write(0);
        MotorDirRight_Write(0);
        motor_forward(0,0);
    }
    //resetting flags to false
    flag_L = false;
    flag_R = false;
}

void follow_line (void) {

    reflectance_read(&ref);
    reflectance_digital(&dig);

    // normalize data from sensors
    l3norm = (((float) ref.l3) - sensor_min)/(sensor_max - sensor_min);
    l2norm = (((float) ref.l2) - sensor_min)/(sensor_max - sensor_min);
    l1norm = (((float) ref.l1) - sensor_min)/(sensor_max - sensor_min);
    r1norm = (((float) ref.r1) - sensor_min)/(sensor_max - sensor_min);
    r2norm = (((float) ref.r2) - sensor_min)/(sensor_max - sensor_min);
    r3norm = (((float) ref.r3) - sensor_min)/(sensor_max - sensor_min);

    // assing weights to sensor data
    L3 = l3norm;        R3 = r3norm;
    L2 = l2norm/2;      R2 = r2norm/2;
    L1 = l1norm/3;      R1 = r1norm/3;

    // average sensor data for each side
    lavg = ((L3 + L2 + L1)/3);
    ravg = ((R3 + R2 + R1)/3);

    // scale averages to 0-255 & inverse for motors
    R = lavg*255;
    L = ravg*255;

    L_power = (L*(L/R)*2);
    R_power = (R*(R/L)*2);

    // limit the power to motors
    if (L_power > 255) {
        L_power = 255;
    }
    if (R_power > 255) {
        R_power = 255;
    }
motor_turn(L_power, R_power, 1);
}
void zmain(void)
{
    vTaskDelay(10000);
    BatteryLed_Write(1);
    while(SW1_Read() == 1) {
        vTaskDelay(100);

        if(SW1_Read() == 0) {

            vTaskDelay(500);
            IR_Start();
            IR_flush();

            vTaskDelay(500);
            reflectance_start();
            motor_start();
            Ultra_Start();

            reflectance_set_threshold(11000, 11000, 13000, 13000, 11000, 11000);

            IR_wait();
            // drive until on the black line
            do {
                follow_line();
            } while (dig.r3 == 0 && dig.l3 == 0);
            motor_forward(0,1);
            print_mqtt("Zumo046/ready","maze");
            IR_wait();
            start = xTaskGetTickCount();
            print_mqtt("Zumo046/start","%d", start);

            // drive over the black line
            do {
                motor_forward(120,1);
            } while (dig.r3 == 1 && dig.l3 == 1);
            motor_forward(120,200);
            flag_L = false; flag_R = false;

            // starting to solve the maze
            for(;;){
                reflectance_read(&ref);
                reflectance_digital(&dig);

                //assigning flags
                if (dig.l1 == 1 || dig.r1 == 1) {
                    flag_F = true;
                }

                if ((dig.l3 == 1) || (dig.r3 == 1)){
                    do {
                        motor_forward(70, 1);
                        reflectance_digital(&dig);
                        if (dig.r3 == 1) {
                            flag_R = true;
                        }
                        if (dig.l3 == 1) {
                            flag_L = true;
                        }
                    } while (dig.r3 == 1 || dig.l3 == 1);

                    // move forward after the black line
                    motor_forward(70, 155); //head on with full batteries

                    reflectance_digital(&dig);

                    // if no line at front turn flag down
                    if (dig.l1 == 0 && dig.r1 == 0) {
                        flag_F = false;
                    }

                    //checking obstacle
                    for (int i = 0; i < 3; i++) {
                        c = b;
                        b = a;
                        a = Ultra_GetDistance();
                    }
                    d = middle_of_3(a, b, c);
                    //print_mqtt("Zumo046/%d", d);

                    vTaskDelay(50);
                    if (d < 15) {
                        flag_F = false;
                    }
                }

                //flags tell us where to go
                if (flag_L == true) {
                    motor_forward(0,10);
                    turn_90(0);
                } else if (flag_F == true) {
                    follow_line();
                } else if (flag_R == true) {
                    motor_forward(0,10);
                    turn_90(1);
                }

                //THE END
                if (flag_L == 0 && flag_R == 0 && dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0){
                    motor_forward(0,1);
                    motor_stop();
                    end = xTaskGetTickCount();
                    int racetime = end - start;
                    print_mqtt("Zumo046/stop","%d", end);
                    print_mqtt("Zumo046/time","%d", racetime);
                    return;
                }
            }
        }
    }
}
#endif

//////////SUMO///////////////////////////////
#if 1

void turn_90(int dir) {
    int x = 0, y = 0;
    if (dir == 0) {
        x = 1;
        y = 0;
    }
    else if (dir == 1) {
        x = 0;
        y = 1;
    }
    motor_forward(50, 200);
    MotorDirLeft_Write(x);
    MotorDirRight_Write(y);
    motor_turn(185,185,400);
    MotorDirLeft_Write(0);
    MotorDirRight_Write(0);
    motor_forward(50, 10);
}

int16_t middle_of_3(int16_t a, int16_t b, int16_t c)
    {
     int16_t middle;

     if ((a <= b) && (a <= c)){
       middle = (b <= c) ? b : c;
     }else if ((b <= a) && (b <= c)){
       middle = (a <= c) ? a : c;
     }else{
       middle = (a <= b) ? a : b;
     }
     return middle;
    }
void zmain(void) {

    TickType_t start, end, racetime;

    int d;
    int sense = 19000; // sensitivity for HIT
    int random = 0;
    int vector_sum = 0;
    int16_t filter_accX = 0, filter_accY = 0;
    struct sensors_ dig;
    struct accData_ data;
    uint8_t LSM303D_Start(void);
    int16_t a_x = 0, b_x = 0, c_x = 0, a_y = 0, b_y = 0, c_y = 0;

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        while(1) vTaskDelay(10);
    }
    else printf("Device Ok...\n");

    motor_start();
    Ultra_Start();
    reflectance_start();
    // set center sensor threshold to 11000 and others to 9000
    reflectance_set_threshold(11000, 11000, 13000, 13000, 11000, 11000);
    IR_Start();
    IR_flush();

    IR_wait();

    // drive to black line
    do{
        reflectance_digital(&dig);
        motor_forward(60,1);
    } while (dig.r3 == 0 && dig.l3 == 0);

    motor_forward(0, 0);
    motor_stop();
    print_mqtt("Zumo046/ready","zumo");
    IR_wait();
    motor_start();

    // drive over line
    motor_forward(255,500);
    start = xTaskGetTickCount();
    print_mqtt("Zumo046/start","%d", start);


    for(;;){

        d = Ultra_GetDistance();

        // median filtering data from ACC sensor
        for (int i = 0; i < 3; i++) {
            LSM303D_Read_Acc(&data);
            c_x = b_x;
            b_x = a_x;
            a_x = data.accX;

            c_y = b_y;
            b_y = a_y;
            a_y = data.accY;
        }
        filter_accX = middle_of_3(a_x, b_x, c_x);
        filter_accY = middle_of_3(a_y, b_y, c_y);

        vector_sum = sqrt((filter_accX*filter_accX)+(filter_accY*filter_accY));

        // check if HIT
        if(vector_sum > sense) {
            end = xTaskGetTickCount();
            Beep(150,30);
            print_mqtt("Zumo046/hit","%d", end);
        }

        // if line is detected, make a "random" turn
        reflectance_digital(&dig);
        if (dig.l3 == 1 || dig.l2 == 1 || dig.l1 == 1 || dig.r1 == 1 || dig.r2 == 1 || dig.r3 == 1) {
            random = xTaskGetTickCount() % 2;
            motor_backward(230,200);
            turn_90(random);
        }

        // if opponent is seen, run away. The robot is a pacifist.
        if (d < 22){
            random = xTaskGetTickCount() % 2;
            turn_90(random);
        }

        // if nothing else, just drive
        motor_forward(255,5);

        // send end time
        if(SW1_Read() == 0){
            vTaskDelay(100);
            motor_forward(0,10);
            motor_stop();
            end = xTaskGetTickCount();
            racetime = end - start;
            print_mqtt("Zumo046/stop","%d", end);
            print_mqtt("Zumo046/time","%d", racetime);
            return;
        }

    }
}
#endif
