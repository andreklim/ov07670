#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

using namespace cv;
using namespace std;

static char * ReadAllBytes(const char * filename, int * read)
{
    ifstream ifs(filename, ios::binary|ios::ate);
    ifstream::pos_type pos = ifs.tellg();
    int length = pos;
    char *pChars = new char[length];
    ifs.seekg(0, ios::beg);
    ifs.read(pChars, length);
    ifs.close();
    *read = length;
    return pChars;
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



int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    unsigned char read_bytes[1000000];
    Mat image;
    int fd = open_port();
    configure_port(fd);

    int readb=0;
    const char * filename = "5.raw";

    char *pChars = ReadAllBytes(filename, &readb);
    image = cv::Mat(240,320,CV_8UC1, (unsigned char *)pChars);

imwrite( "raw_bmp.bmp", image );
    
//read(fd, read_bytes, 100);
int bytes_read;
int byte_count;
int k=0;
int start_frame = 0;
const char READY[] = "*RDY*";

int ready = 0;
    do {
        bytes_read = read(fd, read_bytes, 1000000); 
if(bytes_read!=0) {

    //printf("Read %d\n", bytes_read);
        k++;
            for(int i=0; i<bytes_read; i++) {
            //printf("%d-%c\n", k, read_bytes[i]);
	    if(read_bytes[i]==READY[ready]) {ready++; 
	//	    printf("Check %d %c\n", ready, read_bytes[i]);
		    if(ready>4) 
			{start_frame=1; 
			printf("Start frame\n"); 
			ready=0;
			printf("AAA\n%d %s\n", byte_count, pChars); 
		        image = cv::Mat(240,320,CV_8UC1, (unsigned char *)pChars);

//    namedWindow("Display Image", WINDOW_AUTOSIZE );

//    imshow("Display Image", image);
//    waitKey(0);

char filename[32];
sprintf(filename, "./test%d.bmp", k);
imwrite( filename, image );
//int df = open("raw","wb");
//write(df,pChars);

//std::ofstream output( "raw.raw", std::ios::binary );
//output.write(pChars, byte_count);

			byte_count=0;
			continue;
			//show img
			}
	    }
	    else{ready=0;}
	    pChars[byte_count] = read_bytes[i]; byte_count++;

        }
}
    }
//    while (bytes_read != 0);
    while(1);

//    image = imread( argv[1], 1 );




/*
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    waitKey(0);
*/
    return 0;
}
