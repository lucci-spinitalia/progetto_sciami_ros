#---------------------------------------#
#Executable program file (library)
PROGS     = librobotcore.so libpic2netus.so
#Object file
OBJS      = robot_core.o pic2netus.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c -o $@ $< -I../bin/include -L../bin/librerie -I./ -g
#-------------------------------#

#linking library
librobotcore.so: robot_core.o 
	$(CC) $< -O2 -shared -o $@ -I../bin/include -I./ -lm
#-------------------------------#
#linking library
libpic2netus.so: pic2netus.o
	$(CC) $< -O2 -shared -o $@ -I../bin/include $(LDFLAGS) -I./
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
