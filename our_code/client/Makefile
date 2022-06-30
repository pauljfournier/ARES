all: main.o util.o client.o
	gcc main.o util.o client.o -pthread -lm -lev3dev-c -lbluetooth -o tester

client.o : client.c client.h 
	gcc -std=gnu99 -c client.c -o client.o

util.o : util.c util.h client.h
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -lrt -pthread -c util.c -o util.o

main.o : main.c util.h client.h
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -lrt -pthread -c main.c -o main.o

run:
	./tester
