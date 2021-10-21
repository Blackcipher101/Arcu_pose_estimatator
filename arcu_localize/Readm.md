# ARcu_detection
We use Arcumarkers to localize the bins this is done using the ```stero_localize.py``` which detects arcu markers and then localizez them and does the neded tranforms
------
The detection is done using
```
corners, ids, rejected_img_points = cv.aruco.detectMarkers(gray, arucoDict,parameters=arucoParams,
        cameraMatrix=callib_mat,
        distCoeff=d)
```
it takes the gray image then is applies the parameters selected and the distortion matrix and callib_matrix

the Pose estimatation is done by 
```
rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i],makerSize , callib_mat,d)
```
this returns the trasltation and the rotational vector by taking the marker size callib_matrix and distortion matrix

and we the define the tf we want it to get pose from

## To run
Clone the repo into the src folder and 
run
```
catkin_make
```
then source it
```
source devel/setup.bash
```
then to run it
```
rosrun arcu_localize stero_localize.py
```

