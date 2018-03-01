# IMPORTANT!!!
# You must configure the LIBSPEC variable for your own system.  The setting below reflects
# where the Cortex M0 libraries from gcc are installed on *my* system.  *Your* system
# will be different
# Tell the linker where to find the libraries -> important: use thumb versions

# This guys system:
#LIBSPEC=-L /usr/local/gcc-arm-none-eabi/lib/gcc/arm-none-eabi/4.8.3/armv6-m
#My (Luke's) system
#LIBSPEC=-L /usr/lib/gcc/arm-none-eabi/4.8/armv6-m
#My new system:
LIBSPEC=-L /usr/lib/gcc/arm-none-eabi/5.4.1/armv6-m/


# Specify the compiler to use
CC=arm-none-eabi-gcc
# Specify the assembler to use
AS=arm-none-eabi-as
# Specity the linker to use
LD=arm-none-eabi-ld
# Luke: added c99 flag
CCFLAGS=-mcpu=cortex-m0 -mthumb -g -std=c99

SIZE= arm-none-eabi-size

# List the object files involved in this project (Added: if you split off multiple files, add their objects too? I should find a tutorial on this, or an IDE that does the heavy lifting for me...)
OBJS=	init.o \
	main.o \
	uart.o \
	adxl_i2c.o

# The default 'target' (output) is main.elf and it depends on the object files being there.
# These object files are linked together to create main.elf

# added
#but it does not work. Goal is to auto-reflash the uC on succesfull recompile, maybe also start a debugger.
#all: main.elf SIZE flash
# /added

main.elf : $(OBJS)
	$(LD) $(OBJS) $(LIBSPEC) -lgcc -T linker_script.ld --cref -Map main.map -nostartfiles -o main.elf
# convert binary elf file to hex to help the stm32flash utility
	objcopy -O ihex main.elf main.hex
	$(SIZE) main.elf main.hex


# The object file main.o depends on main.c.  main.c is compiled to make main.o
main.o: main.c
	$(CC) -c $(CCFLAGS) main.c -o main.o

init.o: init.c
	$(CC) -c $(CCFLAGS) init.c -o init.o
#added: (TBD: is this correct? It works...)
uart.o: uart.c uart.h
	$(CC) -c $(CCFLAGS) uart.c -o uart.o
adxl_i2c.o: adxl_i2c.c adxl_i2c.h
	$(CC) -c $(CCFLAGS) adxl_i2c.c -o adxl_i2c.o



# if someone types in 'make clean' then remove all object files and executables
# associated wit this project
clean: 
	rm $(OBJS) 
	rm main.elf 
