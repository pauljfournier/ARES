#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <math.h>
#include <time.h>
#include "util.h"
#include <fcntl.h>
#include <semaphore.h>
#include "client.h"
#include <pthread.h>

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

int * position;
sem_t * semPos;
extern uint16_t msgId;
int s;

///Basic function for turning on place//
void turn(int rot){
  ///Init value
  uint8_t sn1;
  uint8_t sn2;
  int max_speed;
  int pulse;
  ev3_search_tacho_plugged_in(65,0,&sn1,0);
  ev3_search_tacho_plugged_in(66,0,&sn2,0);
  get_tacho_max_speed(sn1,&max_speed);
  get_tacho_count_per_rot(sn1,&pulse);
  ///init motors
  set_tacho_ramp_up_sp(sn1,0);
  set_tacho_ramp_down_sp(sn1,0);
  set_tacho_ramp_up_sp(sn2,0);
  set_tacho_ramp_down_sp(sn2,0);
  set_tacho_speed_sp(sn1,max_speed/3);
  set_tacho_speed_sp(sn2,max_speed/3);
  set_tacho_position_sp(sn1,floor(WHEEL_ROT*rot*pulse/360));
  set_tacho_position_sp(sn2,floor(-WHEEL_ROT*(rot*pulse/360)));
  set_tacho_stop_action_inx(sn1,TACHO_HOLD);
  set_tacho_stop_action_inx(sn2,TACHO_HOLD);
  set_tacho_command_inx(sn1, TACHO_RUN_TO_REL_POS);
  set_tacho_command_inx(sn2, TACHO_RUN_TO_REL_POS);
}
///Basic function to forwardRadar or backward by x centimeters///
void forward(int dist){
  ///Init value
  uint8_t sn1;
  uint8_t sn2;
  int max_speed;
  int pulse;
  ev3_search_tacho_plugged_in(65,0,&sn1,0);
  ev3_search_tacho_plugged_in(66,0,&sn2,0);
  get_tacho_max_speed(sn1,&max_speed);
  get_tacho_count_per_rot(sn1,&pulse);
  ///init motors
  set_tacho_ramp_up_sp(sn1,0);
  set_tacho_ramp_down_sp(sn1,0);
  set_tacho_ramp_up_sp(sn2,0);
  set_tacho_ramp_down_sp(sn2,0);
  set_tacho_speed_sp(sn1,max_speed/3);
  set_tacho_speed_sp(sn2,max_speed/3);
  set_tacho_position_sp(sn1,floor(WHEEL_FOR*dist*pulse/360));
  set_tacho_position_sp(sn2,floor(WHEEL_FOR*dist*pulse/360));
  set_tacho_stop_action_inx(sn1,TACHO_HOLD);
  set_tacho_stop_action_inx(sn2,TACHO_HOLD);
  set_tacho_command_inx(sn1, TACHO_RUN_TO_REL_POS);
  set_tacho_command_inx(sn2, TACHO_RUN_TO_REL_POS);
}

void forwardRadar(int dist){
  uint8_t sn1;
  uint8_t sn2;
  int max_speed;
  ev3_search_tacho_plugged_in(65,0,&sn1,0);
  ev3_search_tacho_plugged_in(66,0,&sn2,0);
  get_tacho_max_speed(sn1,&max_speed);

  forward(dist);

  uint8_t sn_sonar;
  float dist_sonar;
  ev3_sensor_init();
  ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0);

  int n = floor(dist/4) + 1;
  for (int i = 0 ; i<n ; i++){
    get_sensor_value0(sn_sonar, &dist_sonar);
    printf("%f\n", (float) dist_sonar );

    /*if (dist_sonar<300){
      coastMotors();
      break;
    }*/
    if (dist_sonar<160){
      coastMotors();
      break;
    }
    Sleep(250);
  }
  addPosition(dist);
}

///Basic function to rotate the obstacle arm///
void arm(int rot){
  ///Init value
  uint8_t sn;
  int max_speed;
  int pulse;
  ev3_search_tacho_plugged_in(67,0,&sn,0);
  get_tacho_max_speed(sn,&max_speed);
  get_tacho_count_per_rot(sn,&pulse);
  ///init motor
  set_tacho_ramp_up_sp(sn,0);
  set_tacho_ramp_down_sp(sn,0);
  set_tacho_speed_sp(sn,max_speed/6);
  set_tacho_position_sp(sn,floor(-ARM_ROT*rot*pulse/360));
  set_tacho_stop_action_inx(sn,TACHO_HOLD);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
}


void printGyroValues(){
  uint8_t sn_gyro;
  float dist_gyro;
  while(true){
    ev3_sensor_init();
    ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0);
    get_sensor_value0(sn_gyro, &dist_gyro );
    printf("Value 0 is %f \n", dist_gyro);
    get_sensor_value1(sn_gyro, &dist_gyro );
    printf("Value 1 is %f \n", dist_gyro);
    get_sensor_value2(sn_gyro, &dist_gyro );
    printf("Value 1 is %f \n", dist_gyro);
  }
}

void gyroRotation(double rot){
  //integral init
  float ang_abs0;
  float ang_abs;
  float angle = 0;
  uint8_t sn_gyro;
  ev3_sensor_init();
  ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0);
  //tacho motor init
  uint8_t sn1;
  uint8_t sn2;
  int max_speed;
  ev3_search_tacho_plugged_in(65,0,&sn1,0);
  ev3_search_tacho_plugged_in(66,0,&sn2,0);
  get_tacho_max_speed(sn1,&max_speed);
  set_tacho_ramp_up_sp(sn1,0);
  set_tacho_ramp_down_sp(sn1,0);
  set_tacho_ramp_up_sp(sn2,0);
  set_tacho_ramp_down_sp(sn2,0);
  if (rot <= 0){
    set_tacho_speed_sp(sn1,-max_speed/10);
    set_tacho_speed_sp(sn2,max_speed/10);
  }
  else{
    set_tacho_speed_sp(sn1,max_speed/10);
    set_tacho_speed_sp(sn2,-max_speed/10);
  }
  set_tacho_stop_action_inx(sn1,TACHO_HOLD);
  set_tacho_stop_action_inx(sn2,TACHO_HOLD);
  set_tacho_time_sp(sn1,5000);
  set_tacho_time_sp(sn2,5000);
  set_tacho_command_inx(sn1, TACHO_RUN_TIMED);
  set_tacho_command_inx(sn2, TACHO_RUN_TIMED);
  get_sensor_value0(sn_gyro, &ang_abs0 );
  //integral loop
  if (rot <= 0){
    while(angle > rot){
      get_sensor_value0(sn_gyro, &ang_abs );
      angle = ang_abs0-ang_abs;
    }
  }
  else{
    while(angle < rot){
      get_sensor_value0(sn_gyro, &ang_abs );
      angle = ang_abs0-ang_abs;
    }
  }
  holdMotors();
  addDirectionPosition(rot);
}

///Basic function to throw The Obstacle
void throwObs() {
  arm(180);
  Sleep(SLEEP);
  arm(-180);

}

////Basic function that stops all motors at once
void holdMotors(){
  uint8_t sn1;
  uint8_t sn2;
  uint8_t sn3;
  ev3_search_tacho_plugged_in(65,0,&sn1,0);
  ev3_search_tacho_plugged_in(66,0,&sn2,0);
  ev3_search_tacho_plugged_in(67,0,&sn3,0);
  set_tacho_stop_action_inx(sn1,TACHO_HOLD);
  set_tacho_stop_action_inx(sn2,TACHO_HOLD);
  set_tacho_stop_action_inx(sn3,TACHO_HOLD);
  set_tacho_command_inx(sn1,TACHO_STOP);
  set_tacho_command_inx(sn2,TACHO_STOP);
  set_tacho_command_inx(sn3,TACHO_STOP);

}

void coastMotors(){
  uint8_t sn1;
  uint8_t sn2;
  uint8_t sn3;
  ev3_search_tacho_plugged_in(65,0,&sn1,0);
  ev3_search_tacho_plugged_in(66,0,&sn2,0);
  ev3_search_tacho_plugged_in(67,0,&sn3,0);
  set_tacho_stop_action_inx(sn1,TACHO_COAST);
  set_tacho_stop_action_inx(sn2,TACHO_COAST);
  set_tacho_stop_action_inx(sn3,TACHO_COAST);
  set_tacho_command_inx(sn1,TACHO_STOP);
  set_tacho_command_inx(sn2,TACHO_STOP);
  set_tacho_command_inx(sn3,TACHO_STOP);

}
////Function to go to absolute coordinates in an l shape /////
void goToL(int x, int y, int originX, int originY, int angle){
  turn(-angle);
  angle -= angle;
  Sleep(SLEEP);
  forward(y-originY);
  originY = y;
  Sleep(SLEEP);
  turn(90);
  angle += 90;
  Sleep(SLEEP);
  forward(x-originX);
  originX = x;
  Sleep(SLEEP);
}


///Function to initialize sensors///content of function place in catipaineSonar, useless for now
void initSensors(uint8_t sn_sonar){
  ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0);
}

void catipaineSonar(uint8_t sn_sonar){
  float dist_sonar;
  int a = 0;
  int b = 0;
  int angle = 0;

  while(true){
    ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0);
    get_sensor_value0(sn_sonar, &dist_sonar );
    printf("LE MUR EST A %f MILIMETRES AAA \n", dist_sonar);
    if (dist_sonar < 250){
      coastMotors();
      //manageL(a,b,angle);
      break;
    }
  }
}

bool canIMove(int dist){
  float dist_sonar;
  uint8_t sn_sonar;
  //Initizialition of the radar here but would be better in the main
  ev3_sensor_init();
  ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0);
  get_sensor_value0(sn_sonar, &dist_sonar );
  printf("dist = %f\n", dist_sonar);
  if(dist_sonar<(dist*10)){
    return false;
  }
  else{
    return true;
  }
}

float distObst(){
  float dist_sonar;
  uint8_t sn_sonar;
  //Initizialition of the radar here but would be better in the main
  ev3_sensor_init();
  ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0);
  get_sensor_value0(sn_sonar, &dist_sonar );
  return dist_sonar;
}

///Function to detect if object is less than 5 cm
bool isObjectLess5(){
	printf("is obj less 5\n");
	int angle =0;
	while(distObst()<=200 && angle<40){
		turn(5);
		Sleep(400);
		angle+=5;
		printf("%d\n",angle);
	}
	int gangle= angle;
	turn(-angle);
	Sleep(400);
	while(distObst()<=200 && angle<40){
		turn(-5);
		Sleep(400);
		angle+=5;
		printf("%d\n",angle);
	}
	turn(angle-gangle);
	Sleep(400);
	return(angle <=35);
}

///function to turn around object of less than 5 cm
void turnAround5(){
	gyroRotation(-270);
	Sleep(1000);
	forward(30);
	Sleep(1000);
	gyroRotation(-90);
	Sleep(1000);
	forward(16);
	Sleep(1000);
	gyroRotation(-90);
	Sleep(1000);
}

///Function to detect Moveable object
bool isMoveable(){
	while(floor(distObst())>180){
		forward(2);
		Sleep(1000);
	}
	if(isObjectLess5()){
		turnAround5();
		while(floor(distObst())>180){
			forward(2);
			Sleep(1000);
		}
		return isObjectLess5();
	}
	return false;
}

//dance!!!!!
void dance1(){
	forward(-8);
	Sleep(800);
	turn(180);
	Sleep(1000);
	forward(-3);
	Sleep(1000);
	turn(90);
	Sleep(800);
	arm(180);
	turn(-450);
	Sleep(800);
	arm(-180);
	Sleep(800);
}

///Basic function to rotate the weapon arm///
void weaponArm(int rot){
  ///Init value
  uint8_t sn;
  int max_speed;
  int pulse;
  ev3_search_tacho_plugged_in(68,0,&sn,0);
  get_tacho_max_speed(sn,&max_speed);
  get_tacho_count_per_rot(sn,&pulse);
  ///init motor
  set_tacho_ramp_up_sp(sn,0);
  set_tacho_ramp_down_sp(sn,0);
  set_tacho_speed_sp(sn,max_speed);
  set_tacho_position_sp(sn,floor(-ARM_ROT*rot*pulse/360));
  set_tacho_stop_action_inx(sn,TACHO_HOLD);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
}

void attac(){
  weaponArm(90);
  Sleep(500);
  weaponArm(-90);
}


void movementTest(){
  bool obstacle;
  int randomRotation;
  srand(time(NULL));
  while(true){
    if(canIMove(20)){
      forwardRadar(20);
      Sleep(SLEEP);
    }
    else{
      randomRotation=((rand()%3)+1)*90*(((rand()%2)*2)-1);
      gyroRotation(randomRotation);
      Sleep(SLEEP);
    }
  }
}

void writePosition(int x, int y, int direction){
	sem_wait(semPos);
	position[0] = x;
	position[1] = y;
	position[2] = direction;
	sem_post(semPos);
}

int * sendPosition(){
	char string_tmp[58];
	printf("Send position to server\n");
	*((uint16_t *) string_tmp) = msgId++;
	string_tmp[2] = TEAM_ID;
	string_tmp[3] = 0xFF;
	string_tmp[4] = MSG_POSITION;
	sem_wait(semPos);
	string_tmp[5] = position[0]/5;          /* x */
	string_tmp[6] = 0x00;
	string_tmp[7] = position[1]/5;              /* y */
	string_tmp[8]= 0x00;
	sem_post(semPos);
	write(s, string_tmp, 9);
}

void addPosition(int x){
	sem_wait(semPos);
	switch(position[2]){
		
		case 0:
			position[0] = position[0]+x;
			break;
		case 1:
			position[1] = position[1]+x;
			break;
		case 2:
			position[0] = position[0]-x;
			break;
		case 3:
			position[1] = position[1]-x;
			break;
	}
	sem_post(semPos);
	
}

void addDirectionPosition(int rot){
	rot=(rot/90);
	sem_wait(semPos);
	position[2] = (position[2]+rot+4)%4;
	printf("position DIR : %d \n", position[2]);
	sem_post(semPos);
}
