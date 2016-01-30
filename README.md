# utiasdata_to_rosbags
Tool to convert UTIAS multirobot dataset to ROSbags format

UTIAS multirobot dataset can be obtained from http://asrl.utias.utoronto.ca/datasets/mrclam/#Download
Please note that we are not the authors of this dataset. For citing this dataset authored by Keith YK Leung et.al. please see: http://ijr.sagepub.com/content/30/8/969

utiasdata_to_rosbags is aros-based tool to convert the UTIAS datasets (which are text file-based) into rosbags. 

INSTRUCTIONS OF USE:

1. catkin_make this packge.
2. Download the UTIAS multrobot dataset (There are 9 datasets)
3. Let us say the the path to any one of the dataset's root folder where you have files such as Robot1_Odometry.dat and so on is called /path-to-mrclamdataset/MRCLAM_Dataset1/
4. Issue the follwoing command
-- rosrun utiasdata_to_rosbags utiasdata_to_bag /store/UTIAS_MRCLAM/MRCLAM_Dataset1/ UTIAS_Dataset_1.bag
5. Your bag will be created and saved as UTIAS_Dataset_1.bag in the location where you issued the command.

If you have any doubts, please send me an email at aamir.itkgp[at]gmail.com
