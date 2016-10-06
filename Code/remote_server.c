
/*
Author: 
Meher Jain
Vivek Sankaranarayanan

-This C script creates a server based on TCP/IP for CAPE to log the measured data over ethernet.
-CAPE sends the complete AC Metric with 11 elements every 1.08seconds. It uses '$' as delimiter between every metric elemetns
-Server being aware of the fact copies the received data in to a csv file.

-Server is running on following Specification:

1. IP ADDRESS - 192.168.1.3
2. Port No    - 5000
*/

/********************************* Header Files *******************************************/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <arpa/inet.h>
#include <time.h>
/*******************************************************************************************/

/*********************************** Macros ************************************************/
#define BUFFERLENGTH 512
#define PORTNO		 5000
/*******************************************************************************************/


/* 
   Main File. 
   This file creates a TCP/IP based server on specified IP address and port number. The server
   creates a TCP/IP socket and waits for client to connect into. After accepting the client
   connection, it sits in an infinite loop and logs the data sent by the client. 
   args: None
   returns: None
*/
   
   
void main (int argc, char* argv[])
{
	int sock,newsocket;
	char currtime[50] = "\0";
	int i;
	const char *filelog = "ac_metric_log.csv";
	
	struct sockaddr_in serverAddr; /* Server addr */
	struct sockaddr_in clientAddr; /* Client addr */
	char receive_buffer[BUFFERLENGTH] = {'\0'};
	
	FILE *fp1;	  														// Declaring the file pointer //
	int clientlen,count =0;
	time_t t;															// Time structure //
	
	fp1 = fopen(filelog,"w");											// Opening File for logging //
	
	if(fp1 == NULL)
		printf("ERROR in Opening log file\n");
	
	/* Hardcoding the title and subtitile for every element sent by the CAPE. It sends the data in the same order as 		   	subtitles written to the file like 1. VRMS, 2. IRMS, 3. Freq, etc.*/
	fprintf(fp1,"\t\t\t\tAC METRICS\n");
	fprintf(fp1,"Time$VRMS$IRMS$Frequency$PowerFactor$Apparent Power(VA)$Active Power(W)$THD(V)$THI(A)$Phase$Vpeak$Ipeak");
	//fprintf(fp1,"\r\nHello");
	fflush(fp1);													// Flusing the local buffer in to the file //
	
	
	
	/* Opening a socket for TCP/IP Communication */
	
	if((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
	{
	   printf("ERROR: Failed to obtain Socket Descriptor");
	   exit(1);
	}
	else 
	   printf("[Server] Obtaining socket descriptor successfully.\n");

	/* Fill the client socket address struct */ 
	serverAddr.sin_family = AF_INET; // Protocol Family
	serverAddr.sin_port = htons(PORTNO); // Port number
	serverAddr.sin_addr.s_addr = inet_addr("192.168.1.2"); // Fill the local ethernet address
	bzero(&(serverAddr.sin_zero), 8); // Flush the rest of struct

	
	/* Bind a special Port */
	if( bind(sock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr)) == -1 )
	{
	   printf("ERROR: Failed to bind Port.\n");
	   exit(1);
	}
	else 
	  printf ("Binded the port 5000 ....\n");


	/* Listen remote connect/calling */ 
	if(listen(sock,10) == -1)
	{
	   printf("ERROR: Failed to listen Port.\n");
	   exit(1);
	}
	else
	   printf ("Listening on the port 5000 ....\n");
	   

	clientlen = sizeof(struct sockaddr_in);    // Size of socket structure to fillout the client address// 
	
	
	/* Accepting the connection (Blocking Function), waits till there is a a connect request from client */
	newsocket = accept(sock, (struct sockaddr *) &clientAddr, &clientlen);	
	if (newsocket == -1) printf("Error Accepting the connection");
		printf("Connection Accepted....\n");
		
		
	/* Stay in infinite loop after client is connected for logging the data */
	while(1)
	{
		
		int rcvd = recv(newsocket, receive_buffer,sizeof receive_buffer ,0);   // For receiving the data from client//
		
		/* If no data received from client, exit the server */	
		if (rcvd <= 0)
		{
			printf("No Data received from client, Exiting from the child Sever...\n");
			close(newsocket);
			exit(0);
		}
		
		/* Finding current date & time for logging purpose */
		time(&t);
		strncpy(currtime,ctime(&t),(strlen(ctime(&t))-1)); // Copying the result to a buffer //
		
		
		
		if(strcmp(receive_buffer,"$\n\r"))				// Not copying the time for the first ACK received //
		{		
			fprintf(fp1,"%s$",currtime);
		}
		
		fprintf(fp1,"%s\n",receive_buffer);				// copying received data in to the file //
		
		fflush(fp1);									// Flushing the local buffer in to the file pointer//
		printf("Data Logged...\n");	
		
		
	}
	
	close(newsocket);									// Closing the socket // (It never reaches here)
}
