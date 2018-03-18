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
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <time.h>   
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <algorithm>
#include <numeric>
#include <future>

using namespace cv;
using namespace std;
using namespace std::placeholders;

void *display(void *);
void *getvideo(void *);
int configure_port(int fd);
int open_port(void);
void removeThread(std::thread::id id);



Mat image;
int fd; //com port
unsigned char read_bytes[1000000];
char pChars[1000000];
std::condition_variable m_condVar;
std::mutex m_mutex;
int Lock=0;
const char tty[] = "/dev/ttyACM0";
#define THREAD_MAX  8
std::vector<std::thread> threads(THREAD_MAX);
std::map<std::thread::id, int> Locks;

    

   


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
//Open serial port
        fd = open_port();
        configure_port(fd);
    
//Start listen    
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
//Start capture serial
//              pthread_create(&thread_getvideo,NULL,getvideo,NULL);
      std::thread t_getvideo(getvideo, &remoteSocket);

 

              
    //accept connection from an incoming client
    while(1){
       
     remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);  
      std::cout << remoteSocket<< "32"<< std::endl;
      if(Locks.size()>THREAD_MAX) {
        std::cout << "max_thread error > "<< THREAD_MAX<< std::endl;  
        close(remoteSocket);
      }
     
            if (remoteSocket < 0) {
         //       perror("accept failed!");
        //       exit(1);
            }
        else{ 
            std::cout << "Connection accepted" << std::endl;
            
            threads.push_back(std::thread (display, &remoteSocket));
        }

    }
std::cout << "End" << std::endl;
    return 0;
}

void *display(void *ptr){
    //std::cerr << "Start";
    int socket = *(int *)ptr;
   Locks.insert(std::pair<std::thread::id, int>(std::this_thread::get_id(),0));

    std::unique_lock<std::mutex> mlock(m_mutex);
    //OpenCV Code
    //----------------------------------------------------------

    int imgSize = image.total() * image.elemSize();
    int bytes = 0;

    while(1) {

                //send processed image
                if ((bytes = send(socket, image.data, imgSize, 0)) < 0 || (bytes != imgSize)){
                  //   std::cerr << "bytes = " << bytes << std::endl;
                     
                     //Remove thread from vector
                         //std::lock_guard<std::mutex> lock(threadMutex);
                                threads.push_back(
                                std::thread([]() {
                                std::async(removeThread, std::this_thread::get_id());
                                })
                                );
                                Locks.erase(Locks.find(std::this_thread::get_id()));
                             //   std::cerr << "Break "<< std::this_thread::get_id() << std::endl;
                     break;
                } 

        m_condVar.wait(mlock, [](){return Locks[std::this_thread::get_id()] == 1;});
        Locks[std::this_thread::get_id()] = 0;
        
    }

}

void *getvideo(void *ptr) {
 
int bytes_read;
int byte_count;
int k=0;
int start_frame = 0;
const char READY[] = "*RDY*";
image = Mat::zeros(240 , 320, CV_8UC1);

int ready = 0;
    do {
        // std::cerr << "Loop" << std::endl;

//Используем select для порта. Иначе постоянный цикл и нагрузка 100%        
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(fd, &fds);
                struct timeval timeout = { 10, 0 }; /* 10 seconds */
        int ret = select(fd+1, &fds, NULL, NULL, &timeout);
//---------------------------------------------------------
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
                        
                        if(Locks.size()>0) {     
                            std::cerr << "Send new frame to "<< Locks.size() << " clients" << std::endl;
                                    for (std::pair<std::thread::id, int> element : Locks) {
                                        Locks[element.first] = 1;
                                    }
                            

                                m_condVar.notify_all();

                        }


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
    
    fd = open(tty, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if(fd == -1) // if open is unsucessful
    {
	//perror("open_port: Unable to open /dev/ttyS0 - ");
	printf("open_port: Unable to open %s \n", tty);
    exit(1);
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

void removeThread(std::thread::id id)
{
   // std::lock_guard<std::mutex> lock(threadMutex);
    auto iter = std::find_if(threads.begin(), threads.end(), [=](std::thread &t) { return (t.get_id() == id); });
    if (iter != threads.end())
    {
        iter->detach();
        threads.erase(iter);
    }
}
