#---------------------------------------#
#Executable program file (library)
PROGS     = librobot_comm.so libserial_comm.so
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#Object files
OBJS      = robot_comm.o serial_comm.o
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c -o $@ $<	-g
#-------------------------------#

#linking test executable
$(PROGS): $(OBJS)
	$(CC) $(OBJS) -O2 -shared -o $@ -L.
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o *.so core
#-------------------------------#
