#include <cstdio>
#include <iostream>
#include <string>

#include <rosbag/bag.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <utiasdata_to_rosbags/MCLAMMeasurementData.h>
#include <utiasdata_to_rosbags/MCLAM_landmark_GTData.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace ros;



Time
readtime (FILE* fp)
{
  long long unsigned int t;
  fscanf (fp, " %Lu", &t);
  return Time (t / 1000000, (t % 1000000) * 1000);
}

Time
makeTimeFromMCLAMtime (long long unsigned int timeSec, int timeMillisecond)
{
  return Time (timeSec, timeMillisecond * 1000000/*1000000 this was before when the millisecond was in 3-digit... now it is artificially 4*/);
}

Time
makeTimeFromMCLAMtime_measurements (long long unsigned int timeSec, int timeMillisecond)
{
  return Time (timeSec, timeMillisecond * 100000/*1000000 this was before when the millisecond was in 3-digit... now it is artificially 4*/);
}


int
main (int argc,
      char** argv)
{
  if (argc != 3) {
    cerr << "Usage: " << argv[0] << " <UTIAS dataset folder location, e.g., > out.bag" << endl;
    return 1;
  }
  
  char odomFileName[500];
  char measurementFileName[500];
  char gtFileName[500];
  char barcodesFileName[500];
  char landmarkGTFilename[500];
  
  char odomMSGName[500];
  char measurementMSGName[500];
  char gtMSGName[500];  
  
  
  rosbag::Bag bag (string (argv[2]), rosbag::bagmode::Write);
  
  ////////////////////////////////
  ///@recording barcodes first
  ////////////////////////////////
  strcpy(barcodesFileName,argv[1]);
  strcat(barcodesFileName,"Barcodes.dat");
  FILE* data = fopen (barcodesFileName, "r");
  cout<<"The path to the barcode file is "<<barcodesFileName<<endl;
  char type; char filler[500]; int subject,barcode, subject_barcode[20];
  fscanf (data, " %c", &type);
  int hashcounter=0;
  while (! feof (data))
  {
    if (type=='#') {
      hashcounter++;
      fgets(filler, 400, data);
      printf("String = %s\n", filler);   
      if(hashcounter<4)
	 fscanf (data, " %c", &type);
      else
      {
	type='d';
	hashcounter=0;
      }
    }
    else
    {
      fscanf (data, "%d\t%d\n",&subject,&barcode);
      printf("subject = %d, Barcode = %d \n", subject, barcode);
      subject_barcode[subject-1]=barcode;
    }       
  }
  fclose (data);
  
  ////////////////////////////////
  ///@recording landmark gt next
  ////////////////////////////////
  strcpy(landmarkGTFilename,argv[1]);
  strcat(landmarkGTFilename,"Landmark_Groundtruth.dat");
  data = fopen (landmarkGTFilename, "r");
  cout<<"The path to the landmark GroundTruth file is "<<landmarkGTFilename<<endl;
  float x, y, stdX, stdY;
  fscanf (data, " %c", &type);
  hashcounter=0;
  utiasdata_to_rosbags::MCLAM_landmark_GTData landmarkGTdata;
  static unsigned landmarkgt_seq = 0;
  landmarkGTdata.header.seq = landmarkgt_seq++;  
  //time field of the header of landmarkGTdata is set later when extracting first odometry.. check there.
  landmarkGTdata.barcode[0] = subject_barcode[0];// this is actually robot 1
  landmarkGTdata.barcode[1] = subject_barcode[1];// this is actually robot 1
  landmarkGTdata.barcode[2] = subject_barcode[2];// this is actually robot 1
  landmarkGTdata.barcode[3] = subject_barcode[3];// this is actually robot 1
  landmarkGTdata.barcode[4] = subject_barcode[4];// this is actually robot 1
  while (! feof (data))
  {
    if (type=='#') {
      hashcounter++;
      fgets(filler, 400, data);
      printf("String = %s\n", filler);   
      if(hashcounter<4)
	 fscanf (data, " %c", &type);
      else
      {
	type='d';
	hashcounter=0;
      }
    }
    else
    {
      fscanf (data, "%d\t%f\t%f\t%f\t%f\n",&subject,&x,&y,&stdX,&stdY);
      landmarkGTdata.landmarkGTpose[subject-1].pose.position.x= x;
      landmarkGTdata.landmarkGTpose[subject-1].pose.position.y= y;
      landmarkGTdata.landmarkGTpose[subject-1].covariance[0] = stdX*stdX;
      landmarkGTdata.landmarkGTpose[subject-1].covariance[7] = stdY*stdY;//the covariance is a 6by6 row major matrix so varY will be at the 7th cell starting from 0.
      landmarkGTdata.barcode[subject-1] = subject_barcode[subject-1];
      printf("subject = %d, x = %f, y = %f, stdX = %f, stdY = %f\n", subject, x,y,stdX,stdY);
    }       
  }  
  
  

  
  char buf[12]; FILE *dataOdom, *dataMeas, *dataGT; 
  for (int robNumber=1; robNumber<=5; robNumber++)
  {    
    sprintf(buf, "%d", robNumber);
    
    strcpy(odomFileName,argv[1]);
    strcat(odomFileName,"Robot");    
    strcat(odomFileName,buf); 
    strcat(odomFileName,"_Odometry.dat");    
    cout<<"The path to the Odometry file is "<<odomFileName<<endl;
    dataOdom = fopen(odomFileName,"r");
    strcpy(odomMSGName,"Robot_");    
    strcat(odomMSGName,buf); 
    strcat(odomMSGName,"/odometry"); 
    
    strcpy(measurementFileName,argv[1]);
    strcat(measurementFileName,"Robot");    
    strcat(measurementFileName,buf); 
    strcat(measurementFileName,"_Measurement.dat");
    cout<<"The path to the Measurement file is "<<measurementFileName<<endl;    
    dataMeas = fopen(measurementFileName,"r");
    strcpy(measurementMSGName,"Robot_");    
    strcat(measurementMSGName,buf);
    strcat(measurementMSGName,"/measurements");
    
    strcpy(gtFileName,argv[1]);
    strcat(gtFileName,"Robot");    
    strcat(gtFileName,buf); 
    strcat(gtFileName,"_Groundtruth.dat");
    cout<<"The path to the Robot Groundtruth file is "<<gtFileName<<endl;        
    dataGT = fopen(gtFileName,"r");
    strcpy(gtMSGName,"Robot_");    
    strcat(gtMSGName,buf);
    strcat(gtMSGName,"/GTpose");    
    
    
    ////////////////////////////////
    ///@recording ODOMETRY NOW
    ////////////////////////////////
    
    char type; char filler[500]; int subject,barcode;
    fscanf (dataOdom, " %c", &type);
    int hashcounter=0;
    long long unsigned int timeOdomSec;
    int timeOdomMilliSec;
    float v_fwd,w_rot;
    nav_msgs::Odometry odometry;
    
    Time curTime;
    Time prevTimeO;
    double deltaX=0,deltaY=0,deltaTheta=0;
    
    while(!feof(dataOdom))
    {
      if (type=='#') {
      hashcounter++;
      fgets(filler, 400, dataOdom);
      printf("String = %s\n", filler);   
	if(hashcounter<4)
	  fscanf (dataOdom, " %c", &type);
	else
	{
	  type='d'; //filling a non hash character
	  hashcounter=0;
	}
      }
      else
      {
	fscanf (dataOdom, "%llu.%d\t%f\t%f\n",&timeOdomSec,&timeOdomMilliSec,&v_fwd,&w_rot);
	//printf("timeOdomSec = %llu, timeOdomMilliSec = %d, v_fwd = %f, w_rot = %f\n", timeOdomSec, timeOdomMilliSec,v_fwd,w_rot);
	prevTimeO = curTime;
	Time time = makeTimeFromMCLAMtime(timeOdomSec,timeOdomMilliSec);
	curTime = time;
	
	double linearVel = v_fwd;
	double angularVel = w_rot;
	
	
	Duration delta = curTime - prevTimeO;
	double deltaSecs = delta.toSec();
	if(deltaSecs<0)
	{ cout<<"Something wiered or first timestamp, so deltaT NEGATIVE"<<endl; 
	  cout<<"deltaSecs = "<<deltaSecs<<endl;
	}
	deltaX = linearVel*deltaSecs*cos(angularVel*deltaSecs);
	deltaY = linearVel*deltaSecs*sin(angularVel*deltaSecs);
	deltaTheta = angularVel*deltaSecs;  
  
	
	odometry.header.stamp = time;
        static unsigned o_seq[5] = {0,0,0,0,0};
        odometry.header.seq = o_seq[robNumber-1]++;	
	odometry.twist.twist.linear.x=v_fwd;
	odometry.twist.twist.angular.x=w_rot;
	
	odometry.pose.pose.position.x = deltaX;
	odometry.pose.pose.position.y = deltaY;
	odometry.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(deltaTheta);
        odometry.pose.pose.orientation = odom_quat;	
	
	bag.write (odomMSGName, time, odometry);	
      }       
    }
    
    ////////////////////////////////
    ///@recording MEASUREMENTS NOW
    ////////////////////////////////
    
    
    fscanf (dataMeas, " %c", &type);
    hashcounter=0;
    long long unsigned int timeMeasSec;
    int timeMeasMilliSec;
    int prevTime;
    float range,bearing;
    utiasdata_to_rosbags::MCLAMMeasurementData MCLAMmeasurements;
    
    int randCount = 0;
    
    while(!feof(dataMeas))
    {
      if (type=='#') {
      hashcounter++;
      fgets(filler, 400, dataMeas);
      printf("String = %s\n", filler);   
	if(hashcounter<4)
	  fscanf (dataMeas, " %c", &type);
	else
	{
	  type='d';//filling anon hash character
	  hashcounter=0;
	}
      }
      else
      {
	fscanf (dataMeas, "%llu.%d\t%d\t%f\t%f\n",&timeMeasSec,&timeMeasMilliSec,&subject,&range,&bearing);
	//printf("timeMeasSec = %llu, timeMeasMilliSec = %d, subject = %d, range = %f, bearing = %f\n", timeMeasSec, timeMeasMilliSec,subject,range,bearing);
	
	
	timeMeasMilliSec = 10*timeMeasMilliSec+randCount;
	randCount++;
	
	if(randCount==9)
	  randCount=0;
	
	Time time = makeTimeFromMCLAMtime_measurements(timeMeasSec,timeMeasMilliSec);
	MCLAMmeasurements.header.stamp = time;
        static unsigned m_seq[5] = {0,0,0,0,0};
        MCLAMmeasurements.header.seq = m_seq[robNumber-1]++;
	MCLAMmeasurements.subjectBarcode = subject;
	MCLAMmeasurements.range=range;
	MCLAMmeasurements.bearing=bearing;
	bag.write (measurementMSGName, time, MCLAMmeasurements);
      }       
    }   
    
    ////////////////////////////////
    ///@recording ROBOT's Ground Truth NOW
    ////////////////////////////////

  
    fscanf (dataGT, " %c", &type);
    hashcounter=0;
    long long unsigned int timeGTSec;
    int timeGTMilliSec;
    float x_pos,y_pos,orientation;
    geometry_msgs::PoseStamped GTPose;
    
    while(!feof(dataGT))
    {
      if (type=='#') {
      hashcounter++;
      fgets(filler, 400, dataGT);
      printf("String = %s\n", filler);   
	if(hashcounter<4)
	  fscanf (dataGT, " %c", &type);
	else
	{
	  type='d';//filling anon hash character
	  hashcounter=0;
	}
      }
      else
      {
	fscanf (dataGT, "%llu.%d\t%f\t%f\t%f\n",&timeGTSec,&timeGTMilliSec,&x_pos,&y_pos,&orientation);
	//printf("timeGTSec = %llu, timeGTMilliSec = %d, x_pos = %f, y_pos = %f, orientation = %f\n", timeGTSec, timeGTMilliSec,x_pos,y_pos,orientation);
	Time time = makeTimeFromMCLAMtime(timeGTSec,timeGTMilliSec);
	GTPose.header.stamp = time;
        static unsigned gt_seq[5] = {0,0,0,0,0};
        GTPose.header.seq = gt_seq[robNumber-1]++;	
	GTPose.pose.position.x=x_pos;
	GTPose.pose.position.y=y_pos;
	GTPose.pose.position.z=0.0;
	geometry_msgs::Quaternion orientation_quat = tf::createQuaternionMsgFromYaw(orientation);
        GTPose.pose.orientation = orientation_quat;
	bag.write (gtMSGName, time, GTPose);
	
	//Adding landmark GT as well
	{
	  landmarkGTdata.header.stamp = time;
	  bag.write ("all_landmark_gtdata",time,landmarkGTdata); //keep adding the landmark GT!
	}		
	
      }       
    }       
    
  }

  fclose (dataOdom);
  fclose (dataMeas);
  fclose (dataGT);
  bag.close();
  return 0;
}
