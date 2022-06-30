#define SERV_ADDR   "c8:21:58:af:4b:9a"     /* Whatever the address of the server is */
#define TEAM_ID     1                       /* Your team ID */

#define MSG_ACK     0
#define MSG_START    1
#define MSG_STOP   2
#define MSG_KICK    3
#define MSG_POSITION 4
#define MSG_MAPDATA     5
#define MSG_MAPDONE 6
#define Sleep( msec ) usleep(( msec ) * 1000 )

void debug (const char *, ...);

int read_from_server (int , char *, size_t );

void robot();

int init_server();
