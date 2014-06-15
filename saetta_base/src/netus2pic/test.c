#include <stdlib.h>
#include <stdio.h>
#include "robot_comm.h"

void debug_char(unsigned char *p, int len) {
	int i;

	for( i=0; i<len; i++)
		printf("%02x ",p[i]);

	printf("\n");
}


void print_pkg(unsigned char *p, int len){

	int i;

	switch(*p) {
		case 0x7F : { 
			int vel1, vel2;
			// *(p+3)=(MAX_VEL-abs(vel1))&255;
			// *(p+4)=(MAX_VEL-abs(vel1))>>8;
			vel1 = (*(p+4)<<8) | *(p+3);
			vel1 = MAX_VEL - vel1;
			vel1 = *(p+1)?-vel1:vel1;
			// *(p+5)=(MAX_VEL-abs(vel2))&255;
			// *(p+6)=(MAX_VEL-abs(vel2))>>8;
			vel2 = (*(p+6)<<8) | *(p+5);
			vel2 = MAX_VEL - vel2;
			vel2 = *(p+2)?-vel2:vel2;
			printf("%d\t%d\n", vel1,vel2);	
			break;
			    }
		default:
		break;
	}

}

int main(int argc, char * argv[]) {

	int vel1, vel2;
	unsigned char p[12];


	vel1=atoi(argv[1]);
	vel2=atoi(argv[2]);

	set_vel_2_array(p, vel1, vel2);

	debug_char(p, 12);

	print_pkg(p,12);

}
