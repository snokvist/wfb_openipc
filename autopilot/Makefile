CC = gcc
CFLAGS = -Wall -O2
OBJS = main.o utils.o params.o rc_monitor.o alink.o mavlink_handler.o

all: autopilot

autopilot: $(OBJS)
	$(CC) $(CFLAGS) -o autopilot $(OBJS) -lm

# Cross compile for aarch64.
# This target cleans first and then rebuilds all sources with aarch64-linux-gnu-gcc.
autopilot_aarch64: clean
	aarch64-linux-gnu-gcc $(CFLAGS) -o autopilot_aarch64 main.c utils.c params.c rc_monitor.c alink.c mavlink_handler.c -lm

# Cross compile for armhf.
# This target cleans first and then rebuilds all sources with arm-linux-gnueabihf-gcc.
autopilot_armhf: clean
	arm-linux-gnueabihf-gcc $(CFLAGS) -o autopilot_armhf main.c utils.c params.c rc_monitor.c alink.c mavlink_handler.c -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o autopilot autopilot_aarch64 autopilot_armhf
