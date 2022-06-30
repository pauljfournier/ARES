#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "util.h"
#include "client.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
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
pid_t pid_pressed, pid_send, pid_motor;
volatile int stop = 0;
extern int s;
extern uint16_t msgId;
extern int * position;
extern sem_t * semPos;


void signal_bump(int sig) {
	//stop = 1;
	//kill(pid_motor, SIGKILL);
	holdMotors();
	forward(-15);
	Sleep(1000);
      	//gyroRotation(90*(((rand()%2)*2)-1));
      	//Sleep(3000);
}

void signal_alarm(int sig) {
	sendPosition();
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
	int fd;
        int i;
        uint8_t sn;
        FLAGS_T state;
        uint8_t sn_touch;
        uint8_t sn_sonar;
        char s[ 256 ];
        int val;
        float value;
        uint32_t n, ii;
	
	
	//SemPos init
	semPos = sem_open("lock_semPos", O_CREAT, 0644, 1);
	if (semPos == SEM_FAILED){
		printf("Can't init semPos");
		return -1;
	}
	
		
        
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
			printf( "TOUCH sensor is found\n" );
        }



	//For /position
	position = mmap(NULL, sizeof(int)*3, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (position == MAP_FAILED) {
		printf("/position not created , in Child");
		return -1;
	}
	writePosition(STARTX,STARTY,1);

	//Init fork
	pid_motor = fork();
	if (pid_motor == -1) {
		printf("error fork\n");
		exit(-1);
  	}
  	if (pid_motor == 0) { /* Child motor */
		int father = getppid();
		
		attac();
		
		///TEST 1
		/*
		movementTest();
		*/
		
		///TEST 2
		printf("start");
                int tour;
                for (tour=1; tour<10; tour++){
                	printf("tour %d - Start \n",tour);
                        forwardRadar(10);
                        printf("tour %d - forward 1 \n",tour);
                        Sleep(1000);
                        gyroRotation(90);
                        printf("tour %d - turn \n",tour);
                        Sleep(1000);
                        forwardRadar(10);
                        printf("tour %d - forward 1 \n",tour);
                        Sleep(1000);
                        gyroRotation(-90);
                        Sleep(1000);
                        printf("tour %d - Fin \n",tour);
                }
		///TEST 3
		/*
		bool isM = isMoveable();
		if(isM){
			dance1();
		}
		printf("is Moveable = %d",isM);
		*/
		
		
		///TEST 4
		/*
			forwardRadar(10);
			Sleep(1000);
			gyroRotation(-90);
			Sleep(1000);
			throwObs();
		*/
		
		return 0;
	} else { /* Father */
		pid_pressed = fork();
		if (pid_pressed == -1) {
			printf("error fork\n");
			exit(-1);
	  	}
		if (pid_pressed == 0) { /* Child touch pressed */
			int father = getppid();
			while(1){
				while ( _check_pressed( sn_touch ) == 0);
				printf("BUMP!!!\n");
				kill(father, SIGUSR1);
			}
		} else { /* Father */
			pid_send = fork();
			if (pid_send == -1) {
				printf("error fork\n");
				exit(-1);
	  		}
	  		if (pid_send == 0) { /* Child send */
				int father = getppid();
				//Init server
				init_server();
				signal(SIGALRM, signal_alarm);
				alarm(2);
				while(1); // send every 2 seconds until the end
				return 0;
			} else { /* Father */
				signal(SIGUSR1, signal_bump);

				
				while(stop == 0) {
					//TODO
				}
			
				printf("End of program, father kill sons and exit...\n");
				kill(pid_pressed, SIGKILL);
				kill(pid_send, SIGKILL);
			}
		}
	}
	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );
	
	return ( 0 );
}
