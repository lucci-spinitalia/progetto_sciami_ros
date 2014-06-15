/* second pipe example from Haviland */
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/wait.h>
#include <iostream>

#define MSGSIZE 64

const char *msg1 = "hello #1";
const char *msg2 = "hello #2";
const char *msg3 = "hello #3";

int main(void)
{  

   int p[2], pid;

   /* open pipe */

   if(pipe(p) == -1)
   {    perror("pipe call error");
        return(1);
   }

   switch(pid = fork()){
   case -1: perror("error: fork call");
            return(2);

   case 0:  /* if child then write down pipe */
         close(p[0]);  /* first close the read end of the pipe */
         if(dup2(p[1], 1) == -1 ) /* stdout == write end of the pipe */
         {
	    perror( "dup2 failed" );
	    return(1);
	 }
	 /* close(p[1]);*/
	 std::cout<<"Child says: "<< msg1 << std::endl;
	 std::cout<<"Child says: "<< msg2 << std::endl;
	 std::cout<<"Child says: "<< msg3 << std::endl;
         break;
   default:   /* parent reads pipe */
         close(p[1]);  /* first close the write end of the pipe */
         if(dup2(p[0], 0 ) == -1 ) /* stdin == read end of the pipe */
         {
	     perror( "dup2 failed" );
	     return(1);
	 }
	 /* close(p[0]); /* close the fd of read end of the pipe */

	std::string mystr;
	 while( std::getline(std::cin, mystr) )
         {
	     std::cout<<"Parent: " << mystr<<std::endl;
	 }
         wait(NULL);
   }

   return(0);
}


