#---------------------------------------#
#Executable program file (library)
PROGS     = librobot_sensors.so
#Object file
OBJS      = robot_sensors.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin

LDFLAGS+=-lausiliarie
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c -o $@ $< -I../bin/include -I../core -I../netus2pic -I./ -g
#-------------------------------#

#linking library
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) -O2 -shared -o $(PROGS) -I../bin/include -L../bin/librerie $(LDFLAGS)
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#
