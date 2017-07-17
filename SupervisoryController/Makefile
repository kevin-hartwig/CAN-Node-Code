# Makefile for CAN Elevator Supervisory Controller
#
# Adapted from the example code from PCAN_Basic_Linux-4.1.1
#
#****************************************************************************

SRC     = Sources
INC     = -I. -I../.. -Imysql-connector-c++-1.1.9-linux-ubuntu16.04-x86-64bit/include -Imysql-connector-c++-1.1.9-linux-ubuntu16.04-x86-64bit/include/cppconn
RT      = NO_RT

DBG = DEBUG

USERAPP_LIBS = -lpcanbasic mysql-connector-c++-1.1.9-linux-ubuntu16.04-x86-64bit/lib/libmysqlcppconn.so /opt/lampp/lib/libmysqlclient.a

LDLIBS  = -L../lib -L/lib -L/usr/lib -L/usr/local/lib -Lmysql-connector-c++-1.1.9-linux-ubuntu16.04-x86-64bit/lib

DBGFLAGS   = -g

CFLAGS  = $(DBGFLAGS) $(INC) $(LDLIBS)

TARGET1 = main
FILES1  = $(SRC)/$(TARGET1).cpp

all:    $(TARGET1)

$(TARGET1): $(FILES1)
	g++ $(FILES1) $(CFLAGS) -o $(TARGET1) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

clean:
	rm -f $(SRC)/*~ $(SRC)/*.o *~ $(TARGET1)

install:
	cp $(TARGET1) /usr/local/bin
