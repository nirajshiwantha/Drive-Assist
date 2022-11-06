//code is futher optimized than v3, drawing added   
#include <pthread.h> //for threading
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>  //to use Rodrigues
#include <opencv2/imgproc/imgproc.hpp> //for text write on images

#include <string.h>  //for serial communication
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "draw.h" //this includes the functions need to draw the pose of the vehicle on the screen
 

using namespace std;
using namespace cv;



int fd;//Serial Port
int zA,zB;//angle values used to send via serial
		Mat imageA,imageB, imageCopyA,imageCopyB;
		vector< int > idsA;
        vector< vector< Point2f > > cornersA, rejectedA;
        vector< Vec3d > rvecsA, tvecsA;
		vector< int > idsB;
        vector< vector< Point2f > > cornersB, rejectedB;
        vector< Vec3d > rvecsB, tvecsB;
	int dictionaryId = 16;
    bool showRejected = 0;
    bool estimatePose = 1;
    float markerLength = 0.04;
    

void * estimatePoseA(void * temp);//functions to run in threads
void * estimatePoseB(void * temp);



//===========================FUNCTIONS RELATED TO SERIAL SEND=================================
//Main function to initialize serial communication
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    
    fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {
        perror("serialport_init: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed

    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}
////////////////////////////////////////////////////////////////////////////////


//Call this to initialize serial communication
void SerialInt(){
  
  int baudrate = 9600;  // default
  fd = serialport_init("/dev/ttyUSB0", baudrate);
  
}

///////////////////////////////////////////////////////////////////////////////
int serialport_write(int serport, const char* str)
{
    int len = strlen(str);
    int n = write(serport, str, len);
    if( n!=len ) {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0; 
}
///////////////////////////////////////////////////////////////////////////////
int serialport_writebyte( int serport, int b)
{
    int n = write(serport,&b,1);
    if( n!=1)
        return -1;
	
    return 0;
	
}

///////////////////////////////////////////////////////////////////////////////
int serialport_flush(int serport)
{
    usleep(500000); //required to make flush work, for some reason
    return tcflush(serport, TCIOFLUSH);
}

//===========================END SERIAL COMMUNICATION FUNCTIONS================================

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejectedA candidates too }";
}
 
 
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) { //this function is used to read camera parameters file
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}


//_____________________________________MAIN FUNCTION_______________________________

Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();


Mat camMatrixA, distCoeffsA;
	Mat camMatrixB, distCoeffsB;

bool serial_out = 0;//serial out on/off
 VideoCapture camA;
 VideoCapture camB;
int main(int argc, char *argv[]) {
	
		cout<<"initializing..."<<endl;
		Mat intro_im=imread("1.jpg");//splash screen
		cvNamedWindow("out", CV_WINDOW_NORMAL);
		cvSetWindowProperty("out", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		imshow("out", intro_im); 
	    waitKey(60);	
	
		
	if(serial_out==true){
	SerialInt();//initialize serial port
	}
	
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }
   
		

    
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

   
    if(!parser.check()) { 
        parser.printErrors();
        return 0;
    }

    
    
    if(estimatePose) {
        //bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
		bool readOk1 = readCameraParameters("A.yml", camMatrixA, distCoeffsA);
		bool readOk2 = readCameraParameters("B.yml", camMatrixB, distCoeffsB);
        if(!readOk1 | !readOk2) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

   // VideoCapture camA;
   //VideoCapture camB;
    int waitTime;
       
        camA.open(1); //camA.open(camId);
        camB.open(2);
        waitTime = 1;
        //camA.set(CV_CAP_PROP_FRAME_WIDTH,1280); //Use this if you want to use custom resolution
		//camA.set(CV_CAP_PROP_FRAME_HEIGHT,960);
   

    double totalTime = 0;
    int totalIterations = 0;

	
	    
   
	while(camA.grab()) {//infinite loop
				
		
        double tick = (double)getTickCount();//start timer	
		
        camA.read(imageA);
		camB.read(imageB);
		
		
			
        

        
		

     
		pthread_t thread1;
	    pthread_t thread2;
	
	 if(pthread_create(&thread1, NULL, estimatePoseA, NULL))//create thread1 and run
				
    {
        printf("Error: Failed to create thread1\n");
        return 0;
    }
	
	if(pthread_create(&thread2, NULL, estimatePoseB, NULL))//create thread2 and run
				
    {
        printf("Error: Failed to create thread2\n");
        return 0;
    }
	
		

        // draw results
        imageA.copyTo(imageCopyA);
		imageB.copyTo(imageCopyB);
		
        if(idsA.size() > 0) {
            aruco::drawDetectedMarkers(imageCopyA, cornersA, idsA);

            if(estimatePose) {
                for(unsigned int i = 0; i < idsA.size(); i++)
                    aruco::drawAxis(imageCopyA, camMatrixA, distCoeffsA, rvecsA[i], tvecsA[i],
                                    markerLength * 0.5f);
                             
                                      
            }
        }
	    if(idsB.size() > 0) {
					aruco::drawDetectedMarkers(imageCopyB, cornersB, idsB);

					if(estimatePose) {
						for(unsigned int i = 0; i < idsB.size(); i++)
							aruco::drawAxis(imageCopyB, camMatrixB, distCoeffsB, rvecsB[i], tvecsB[i],
											markerLength * 0.5f);
	

					}
				}

        if(showRejected && rejectedA.size() > 0)
            aruco::drawDetectedMarkers(imageCopyA, rejectedA, noArray(), Scalar(100, 0, 255));

		if(showRejected && rejectedB.size() > 0)
            aruco::drawDetectedMarkers(imageCopyB, rejectedB, noArray(), Scalar(100, 0, 255));
		
        
		//making the value zero when not detected
		if(idsA.size() == 0){
			zA=0;
		}
		
		if(idsB.size() == 0){ 
			zB=0;
		}
		
		//-----serial---		
		char text[255]; 
		if(serial_out==true){
		
		sprintf(text, "%d", (int)zA);
		
		serialport_write(fd, text);
		serialport_write(fd, "x");
		
		
		
		sprintf(text, "%d", (int)zB);
		
		serialport_write(fd, text);
		
		serialport_write(fd, "\n"); 
		
		cout << zA << "    "<<zB << endl;
		}
		
		//---------------end serial
		
		
		
		
		int fontFace=2;
		double fontScale=1;
		Scalar color=CV_RGB(0, 255, 0);
		int thickness=2;
		int lineType=8;
	

		
		sprintf(text, "RAz= %d", (int)zA);
		putText(imageCopyA, text, Point(60,40), fontFace, fontScale, color, thickness, lineType, false);
				
		sprintf(text, "RBz= %d", (int)zB);
		putText(imageCopyB, text, Point(60,40), fontFace, fontScale, color, thickness, lineType, false);
		
		//imshow("outA", imageCopyA);
		//imshow("outB", imageCopyB);
		
		//combining imageCopyA and imageCopyB to single image
		//Mat dst(320,480,CV_8UC3,Scalar(0,0,0));
		Mat dst
		//cv::hconcat(imageCopyA, imageCopyB, dst);
		
		float B1,B2;
		B1=zA/57.2958;
		B2=zB/57.2958;
		draw_truck(dst);
		draw_connection(dst,B1,B2);
		draw_guide(dst,B1,B2);  
		
		imshow("out", dst); 
		
		pthread_detach(thread1);
		pthread_detach(thread2);
		
		
		double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
       // if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;

			
		//}
		
		
        char key = (char)waitKey(waitTime);
        if(key == 27) break;//esc
    }


    return 0;
}


void * estimatePoseA(void * temp)
{
   // detect markers and estimate pose
        aruco::detectMarkers(imageA, dictionary, cornersA, idsA, detectorParams, rejectedA);
        
		
		if(estimatePose && idsA.size() > 0){
          	
			aruco::estimatePoseSingleMarkers(cornersA, markerLength, camMatrixA, distCoeffsA, rvecsA, tvecsA);
        
			
		Mat R ;
		Rodrigues(rvecsA[0], R);//make rotation matrix from the rotation vector 
					
		/*
			//show rotation vector and it's elements:for debugging purposes
		cout<< R <<endl;
		cout<< "r11=" << R.at<double>(0,0) <<endl;//11
		cout<< "r12=" << R.at<double>(0,1) <<endl;//12
		cout<< "r13=" << R.at<double>(0,2) <<endl;//13
			cout<< "r21=" << R.at<double>(1,0) <<endl;//21
			cout<< "r22=" << R.at<double>(1,1) <<endl;//22
			cout<< "r23=" << R.at<double>(1,2) <<endl;//23
				cout<< "r31=" << R.at<double>(2,0) <<endl;//31
				cout<< "r32=" << R.at<double>(2,1) <<endl;//32
				cout<< "r33=" << R.at<double>(2,2) <<endl;//33
*/
			
			//calculating angle of rotation around x-y-z axis of the marker
			double Rxa,Rya,Rza;
			//Rxa=57.2958*(atan2(R.at<double>(2,1),R.at<double>(2,2)));
			//Rya=57.2958*(atan2(-R.at<double>(2,0),sqrt(pow(R.at<double>(2,1),2)+pow(R.at<double>(2,2),2))) );
			Rza=57.2958*(atan2(R.at<double>(1,0),R.at<double>(0,0)));
			
			if(Rza<0){//angle calibaration
			zA=Rza+148;
			}else{
			zA=Rza-211;
			}
			
			/*cout<< "Rxa=" << Rxa <<endl;
			cout<< "Rya=" << Rya <<endl;
			cout<< "Rza=" << Rza <<endl;*/
		
			
		
		
		}
		
    pthread_exit(NULL); 
}		









void * estimatePoseB(void * temp)
{
  
	aruco::detectMarkers(imageB, dictionary, cornersB, idsB, detectorParams, rejectedB);
if(estimatePose && idsB.size() > 0){
            
			
			aruco::estimatePoseSingleMarkers(cornersB, markerLength, camMatrixB, distCoeffsB, rvecsB,tvecsB);
        		
		Mat R ;
		Rodrigues(rvecsB[0], R);
		
			
			double Rxb,Ryb,Rzb;
			//Rxb=57.2958*(atan2(R.at<double>(2,1),R.at<double>(2,2)));
			//Ryb=57.2958*(atan2(-R.at<double>(2,0),sqrt(pow(R.at<double>(2,1),2)+pow(R.at<double>(2,2),2))) );
			Rzb=57.2958*(atan2(R.at<double>(1,0),R.at<double>(0,0)));
			zB=Rzb+20;
			
			/*cout<< "Rxb=" << Rxb <<endl;
			cout<< "Ryb=" << Ryb <<endl;
			cout<< "Rzb=" << Rzb <<endl;*/
			
		}
		
    pthread_exit(NULL);
}		




