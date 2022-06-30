#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
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
int stop = 0;

void signal_bump(int sig) {
	stop = 1;
}

void signal_alarm(int sig) {
	printf("Send position to server\n");
	alarm(2);
}

static bool _check_pressed( uint8_t sn )
{
        int val;

        if ( sn == SENSOR__NONE_ ) {
                return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
        }
        return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}

int main( void )
{
        int i;
        uint8_t sn;
        FLAGS_T state;
        uint8_t sn_touch;
        uint8_t sn_color;
        uint8_t sn_compass;
        uint8_t sn_sonar;
        uint8_t sn_mag;
        char s[ 256 ];
        int val;
        float valr;
        float valg;
        float valb;
        float value;
        uint32_t n, ii;
        pid_t pid_pressed, pid_send;
        
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
        printf( "Found tacho motors:\n" );
        for ( i = 0; i < DESC_LIMIT; i++ ) {
                if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
                        printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
                        printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
                        printf("  port = %d %d\n",  ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
				}
		}
        //Run motors in order from port A to D
        int port=65;
        for (port=65; port<69; port++){
        	Sleep(2000);
		    if ( ev3_search_tacho_plugged_in(port,0, &sn, 0 )) {
		            int max_speed;
		            printf( "LEGO_EV3_M_MOTOR 1 is found, run for 5 sec...\n" );
					get_tacho_max_speed( sn, &max_speed );
		            printf("  max speed = %d\n", max_speed );

		    } else {
		            printf( "LEGO_EV3_M_MOTOR 1 is NOT found\n" );
		    }
        }
//Run all sensors
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
        if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0 )){ 
			printf( "TOUCH sensor is found, press BUTTON for EXIT...\n" );
        }
        
//Init fork
		pid_pressed = fork();
		if (pid_pressed == -1) {
			printf("error fork\n");
			exit(-1);
	  	}

		if (pid_pressed == 0) { /* Child touch pressed */
			int father = getppid();
			while ( _check_pressed( sn_touch ) == 0) { }
			printf("BUMP!!!\n");
			kill(father, SIGUSR1);
			return 0;
		} else { /* Father */
			pid_send = fork();
			if (pid_send == -1) {
				printf("error fork\n");
				exit(-1);
	  		}
	  		if (pid_send == 0) { /* Child send */
				int father = getppid();
				signal(SIGALRM, signal_alarm);
				alarm(2);
				while(1); // send every 2 seconds until the end
				return 0;
			} else { /* Father */
				signal(SIGUSR1, signal_bump);
				while(stop == 0) {
					for (port=65; port<67; port++){
						if ( ev3_search_tacho_plugged_in(port, 0, &sn, 0) ) {
							int max_speed;
							get_tacho_max_speed( sn, &max_speed );
							set_tacho_stop_action_inx( sn, TACHO_COAST );
							set_tacho_speed_sp( sn, max_speed * 1/3 );
							set_tacho_time_sp( sn, 100 );
							set_tacho_ramp_up_sp( sn, 0 );
							set_tacho_ramp_down_sp( sn, 0 );
							set_tacho_command_inx( sn, TACHO_RUN_TIMED );
						}
					}
				}
				/* if bump */
				for (port=65; port<67; port++){
					if ( ev3_search_tacho_plugged_in(port,0, &sn, 0 )) {
						int max_speed;
						get_tacho_max_speed( sn, &max_speed);
					    set_tacho_stop_action_inx( sn, TACHO_COAST );
					    set_tacho_speed_sp( sn, max_speed *-1);
					    set_tacho_time_sp( sn, 300 );
					    set_tacho_ramp_up_sp( sn,50 );
					    set_tacho_ramp_down_sp( sn, 50 );
					    set_tacho_command_inx( sn, TACHO_RUN_TIMED );
					}
				}
				printf("End of program, father kill sons and exit...\n");
				kill(pid_pressed, SIGKILL);
				kill(pid_send, SIGKILL);
			}
		}
		ev3_uninit();
		printf( "*** ( EV3 ) Bye! ***\n" );

		return ( 0 );
}
