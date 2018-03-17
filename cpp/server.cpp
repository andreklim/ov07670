/**
 * OpenCV video streaming over TCP/IP
 * Server: Captures video from a webcam and send it to a client
 * by Isaac Maia
 */

#include "opencv2/opencv.hpp"
#include <iostream>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> 
#include <string.h>
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls


using namespace cv;
using namespace std;

void *display(void *);
void *getvideo(void *);
int configure_port(int fd);
int open_port(void);



int capDev = 0;
Mat image;
int fd;
unsigned char read_bytes[1000000];
char pChars[1000000];

    VideoCapture cap(capDev); // open the default camera
    

   


int main(int argc, char** argv)
{   

    //--------------------------------------------------------
    //networking stuff: socket, bind, listen
    //--------------------------------------------------------
    int                 localSocket,
                        remoteSocket,
                        port = 4097;                               

    struct  sockaddr_in localAddr,
                        remoteAddr;
    pthread_t thread_id, thread_getvideo;
    
           
    int addrLen = sizeof(struct sockaddr_in);

       
    if ( (argc > 1) && (strcmp(argv[1],"-h") == 0) ) {
          std::cerr << "usage: ./cv_video_srv [port] [capture device]\n" <<
                       "port           : socket port (4097 default)\n" <<
                       "capture device : (0 default)\n" << std::endl;

          exit(1);
    }

    if (argc == 2) port = atoi(argv[1]);

    localSocket = socket(AF_INET , SOCK_STREAM , 0);
    if (localSocket == -1){
         perror("socket() call failed!!");
    }    

        fd = open_port();
        configure_port(fd);
    
    
    
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons( port );

    if( bind(localSocket,(struct sockaddr *)&localAddr , sizeof(localAddr)) < 0) {
         perror("Can't bind() socket");
         exit(1);
    }
    
    //Listening
    listen(localSocket , 3);
    
    std::cout <<  "Waiting for connections...\n"
              <<  "Server Port:" << port << std::endl;

              pthread_create(&thread_getvideo,NULL,getvideo,NULL);
   
image = Mat::zeros(240 , 320, CV_8UC1);
              
    //accept connection from an incoming client
    while(1){
    //if (remoteSocket < 0) {
    //    perror("accept failed!");
    //    exit(1);
    //}
       
     remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);  
      //std::cout << remoteSocket<< "32"<< std::endl;
    if (remoteSocket < 0) {
        perror("accept failed!");
        exit(1);
    } 
    std::cout << "Connection accepted" << std::endl;
     pthread_create(&thread_id,NULL,display,&remoteSocket);

     //pthread_join(thread_id,NULL);

    }
    //pthread_join(thread_id,NULL);
    //close(remoteSocket);

    return 0;
}

void *display(void *ptr){
    int socket = *(int *)ptr;
    //OpenCV Code
    //----------------------------------------------------------

    Mat img, imgGray, tmp;
    img = Mat::zeros(240 , 320, CV_8UC1);   
     //make it continuous
    if (!img.isContinuous()) {
        img = img.clone();
    }

    int imgSize = img.total() * img.elemSize();
    int bytes = 0;
    int key;
    

    //make img continuos
    if ( ! img.isContinuous() ) { 
          img = img.clone();
          imgGray = img.clone();
    }
        
 //   std::cout << "Image Size:" << imgSize << std::endl;

    while(1) {
                //
            /* get a frame from camera */
                //cap >> img;
                //======================
                //tmp = imread( "./ellipses.jpg", 1 );
                img = image;
                //imgSize = img.total() * img.elemSize();
            //std::cerr << "bytes = " << img << std::endl;
                //do video processing here 
                //cvtColor(img, imgGray, CV_BGR2GRAY);
//cout << "Image size" << image;


                //send processed image
                if ((bytes = send(socket, img.data, imgSize, 0)) < 0){
                     std::cerr << "bytes = " << bytes << std::endl;
                     break;
                } 
    }

}

void *getvideo(void *ptr) {
 
    int bytes_read;
int byte_count;
int k=0;
int start_frame = 0;
const char READY[] = "*RDY*";

int ready = 0;
    do {
        bytes_read = read(fd, read_bytes, 1000000); 
if(bytes_read!=0) {

        k++;
            for(int i=0; i<bytes_read; i++) {

	    if(read_bytes[i]==READY[ready]) {ready++; 

		    if(ready>4) 
			{start_frame=1; 
			printf("Start frame\n"); 
			ready=0;

		        image = cv::Mat(240,320,CV_8UC1, (unsigned char *)pChars);



/* save to file                
char filename[32];
sprintf(filename, "./test%d.bmp", k);
imwrite( filename, image );
*/

			byte_count=0;
			continue;
			}
	    }
	    else{ready=0;}
	    pChars[byte_count] = read_bytes[i]; byte_count++;

        }
}
    }

    while(1);

    
}

int open_port(void)
{
    int fd; // file description for the serial port
    
    fd = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY);
    
    if(fd == -1) // if open is unsucessful
    {
	//perror("open_port: Unable to open /dev/ttyS0 - ");
	printf("open_port: Unable to open /dev/ttyS0. \n");
    }
    else
    {
	fcntl(fd, F_SETFL, 0);
	printf("port is open.\n");
    }
    
    return(fd);
} //open_port

int configure_port(int fd)      // configure the port
{
    struct termios port_settings;      // structure to store the port settings in

    cfsetispeed(&port_settings, B1000000);    // set baud rates
    cfsetospeed(&port_settings, B1000000);

    port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;
    
    tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
    return(fd);

} //configure_port
