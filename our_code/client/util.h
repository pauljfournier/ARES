#define WHEEL_ROT (6/2.9)*0.933
#define WHEEL_FOR 2.9*2*M_PI*1.17
#define ARM_ROT 1
#define SLEEP 2000
#define STARTX 60
#define STARTY 30

///Basic function for turning on place//
void turn(int);

///Basic function to forward or backward by x centimeters///
void forward(int);

///Basic function to rotate the obstacle arm///
void arm(int);
void throwObs();

////Basic function that stops all motors at once
void holdMotors();

void coastMotors();

void forwardRadar(int);

void goToL(int, int, int, int, int);


void printGyroValues();

void gyroRotation(double);

void initSensors(uint8_t);

void catipaineSonar(uint8_t);

bool canIMove(int);

void movementTest();

float distObst();

bool isObjectLess5();

void turnAround5();

bool isMoveable();

void dance1();

void writePosition(int, int, int);

int * sendPosition();

void addPosition(int);

void addDirectionPosition(int);

void weaponArm(int);

void attac();
