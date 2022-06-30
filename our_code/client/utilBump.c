#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <math.h>
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))
#define WHEEL_ROT (6/2.9)*0.933
#define WHEEL_FOR 2.9*2*M_PI*1.17
#define ARM_ROT 1

static bool _check_pressed( uint8_t sn )
{
	int val;

	if ( sn == SENSOR__NONE_ ) {
		return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
	}
	return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}

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
///Basic function to forward or backward by x centimeters///
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
  set_tacho_speed_sp(sn1,2*max_speed/3);
  set_tacho_speed_sp(sn2,2*max_speed/3);
  set_tacho_position_sp(sn1,floor(WHEEL_FOR*dist*pulse/360));
  set_tacho_position_sp(sn2,floor(WHEEL_FOR*dist*pulse/360));
  set_tacho_stop_action_inx(sn1,TACHO_HOLD);
  set_tacho_stop_action_inx(sn2,TACHO_HOLD);
  set_tacho_command_inx(sn1, TACHO_RUN_TO_REL_POS);
  set_tacho_command_inx(sn2, TACHO_RUN_TO_REL_POS);
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
  set_tacho_speed_sp(sn,max_speed/3);
  set_tacho_position_sp(sn,floor(ARM_ROT*rot*pulse/360));
  set_tacho_stop_action_inx(sn,TACHO_HOLD);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
}

////Basic function that stops all motors at once
void stopMotors(){
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

////Detect touch
void touched(int timem){
  int currentTime = 0
  while(currentTime!= timem){
  	if(_check_pressed( sn_touch )) break;
	currentTime+=100;
	Sleep(100);
  }
  if(currentTime!= timem){
  	stopMotors();
	printf( "BUMP !!!!\n" );
  }
}

////////////Main function for testing purposes////
int main(int argc, char const *argv[]) {
  #ifndef __ARM_ARCH_4T__
  	/* Disable auto-detection of the brick (you have to set the correct address below) */
  	ev3_brick_addr = "192.168.0.204";

  #endif
  	if ( ev3_init() == -1 ) return ( 1 );

  #ifndef __ARM_ARCH_4T__
  	printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

  #else
  	printf( "Waiting tacho is plugged...\n" );

  #endif
  	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

  	printf( "*** ( EV3 ) Hello! ***\n" );

	////init all sensors
	ev3_sensor_init();

        printf( "Found sensors:\n" );
        for ( i = 0; i < DESC_LIMIT; i++ ) {
                if ( ev3_sensor[ i ].type_inx != SENSOR_TYPE__NONE_ ) {
                        printf( "  type = %s\n", ev3_sensor_type( ev3_sensor[ i ].type_inx ));
                        printf( "  port = %s\n", ev3_sensor_port_name( i, s ));
                        if ( get_sensor_mode( i, s, sizeof( s ))) {
                                printf( "  mode = %s\n", s );
                        }
                        if ( get_sensor_num_values( i, &n )) {
                                for ( ii = 0; ii < n; ii++ ) {
                                        if ( get_sensor_value( ii, i, &val )) {
                                                printf( "  value%d = %d\n", ii, val );
                                        }
                                }
                        }
                }
        }


  turn(360);
  touched(5000);
  forward(30);
  touched(5000);
  arm(90);
  touched(5000);
  arm(-90);
  touched(5000);
  stopMotors();
  return 0;
}
