#
# 'make depend' uses makedepend to automatically generate dependencies 
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#

# This makefile needs some manual changes in order to be functional under Windows or Linux
# Read through the makefile and the comments to understand which changes are necessary.
# For Shared Memory interprocess communication see the comment at the "LIBS" section of this makefile.
# Regarding the "clean" section at the bottom of this makefile - please copy one of the two lines below under the "clean:" keyword

# For LINUX:
# $(RM) *.o *~ $(MAIN) $(SRC_DIR)/*.o $(SRC_DIR_2)/*.o

# For WINDOWS:
# cd $(SRC_DIR) && del *.o && cd .. && cd $(SRC_DIR_2) && del *.o && cd.. && cd.. && del *.o && del $(MAIN_2)

# For WINDOWS run makefile using mingw-w64 OUTSIDE of eclipse! Use Windows Power Shell + make from mingw-w64
# Just install mingw-w64 with the setup and select "x86_64-8.1.0-posix-seh-rt_v6-rev0"
# Than go into "C:\Program Files\mingw-w64\x86_64-8.1.0-posix-seh-rt_v6-rev0\mingw64\bin" and make a copy of
# "mingw32-make.exe" and name this copy "make.exe"
# Now just add this "bin" path to the PATH variable of Windows.
# Make should now work within the power shell!

# DO NOT USE MAKE WITHIN ECLIPSE FOR WINDOWS!

# define the C compiler to use
CC = gcc

# define any compile-time flags
CFLAGS = -Wall -g3 -O0

# define any directories containing header files other than /usr/include
#
INCLUDES = -I./inc
INCLUDES += -I./inc/i2c
INCLUDES += -I./libs
INCLUDES += -I/usr/include/python2.7
# used for double based solic position algorithm
#INCLUDES += -I./libs/solic_position_matlabc_double
# used for float based solic position algorithm
#INCLUDES += -I./libs/solic_position_matlabc_float

# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
LFLAGS = -L./libs/
LFLAGS += -Wl,-rpath,./libs

# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname 
#   option, something like (this will link in libmylib.so and libm.so:
LIBS = -lm -lpthread -lpython2.7 #-li2c
# ATTENTION! Add "-lpthread" if you are using Shared Memory for interprocess communication! ONLY with Linux!
# -lradar_sdk is here not used as this application runs without infineon libraries

SRC_DIR = ./src
# used for double based solic position algorithm
#SRC_DIR_2 = ./libs/solic_position_matlabc_double
# used for float based solic position algorithm
#SRC_DIR_2 = ./libs/solic_position_matlabc_float

# define the C source files
SRCS = lidar_app.c
SRCS += $(wildcard $(SRC_DIR)/*.c)
SRCS += $(wildcard $(SRC_DIR_2)/*.c)

# define the C object files 
#
# This uses Suffix Replacement within a macro:
#   $(name:string1=string2)
#         For each word in 'name' replace 'string1' with 'string2'
# Below we are replacing the suffix .c of all words in the macro SRCS
# with the .o suffix
#
OBJS = $(SRCS:.c=.o)

# define the executable file 
MAIN = lidar_app
MAIN_2 = lidar_app.exe

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all:    $(MAIN)
				@echo  Simple compiler named lidar_app has been compiled

$(MAIN): $(OBJS) 
				$(CC) $(LFLAGS) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) $(LIBS)

# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
.c.o:
				$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@ $(LIBS)

clean:
				$(RM) *.o *~ $(MAIN) $(SRC_DIR)/*.o $(SRC_DIR_2)/*.o
				
