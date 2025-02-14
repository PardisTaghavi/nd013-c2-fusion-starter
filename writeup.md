# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
Step one of the project needs implementing EKF to track a single target and the input was limited to lidar data at first. As a usual approach for writing EKF algorithms it was needed to complete predict() and update() functions. The challenge I faced at the beginning was the fact I did not notice we have lidar data that is 3d so dimension of F and Q should be 6*6. printing some of the inputs in the program quickly I noticed the problem and fixed my metrics. Formulas In Update and Predict functions are standard formulas used in the Nanodegree course.
RMSE plot of this first step :
![alt text](https://github.com/PardisTaghavi/nd013-c2-fusion-starter/blob/main/img/rmseMean.png)


Step Two of the project is related to implementation of the track management. tracks have been divided to three different states "initialized", "tentative" and "confirmed", and track score is added for tentative detections with score over the defined threshold and for confirmed detections with score over confirmed threshold. This part was more straight forward the point that I should have noticed was the fact that detection should be deleted with considering different thresholds (e.g. only confirmed tracks are deleted based on the delete_threshold=0.6 but this number is high for deleting initial or tentative detections and if we use this large threshold we would lose some of our detections).
RMSE plot of the second step :
![alt text](https://github.com/PardisTaghavi/nd013-c2-fusion-starter/blob/main/img/smseMeanTrackManagement.png)


Step Three of the project has been the most challenging part for me because of the small mistakes I made at the first. This part mainly takes care of defining the association matrix, finding the distance of trackings and measurements using MHD gating and finding closest tracks and measurements with "get_closest_track_and_meas". First I implemented MHD and Gating functions and taking advantage of them wrote association_matrix. Running the program at this point I did not get any error and I continued working on updating the list of unassigned measurements and unassigned tracks and finding the closest distance between tracks and measurements, but at the end of this work my algorithm was detecting nothing. Printing out the association matrix I noticed my track list is empty. The first mistake was related to updating unassigned tracks. After changing this part, the program could detect the first measurement and tracking list but later on it could not get updated correctly, finding some other mistakes one of them was related to the calculation of MHD distance. The first way I defined this distance was wrong.
RMSE plot of the third part:
![alt text](https://github.com/PardisTaghavi/nd013-c2-fusion-starter/blob/main/img/result3.png)


Step four primarily takes care of the field of view based on the camera. Later measurement function of the camera should be added (hx) which is a nonlinear function. and lastly in the Measurement class attributes of the camera should be considered.
RMSE plot of the fourth step :
![alt text](https://github.com/PardisTaghavi/nd013-c2-fusion-starter/blob/main/img/result4.png)





### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
Answer of both questions are yes. in Theory adding more sensors should provide us with better and more accurate detections which is what we can see in the code. Comparing RMSE plot of step 3(using only lidar) and step 4(using camera + Lidar) we can see average RMSE of three different tracks decreased.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
1. One of the challenges happening in both real-world and this code is that tracking systems receive many measurements and tracks and deciding which measurement is related to which track should be considered carefully. Track management should be tuned as well so we are sure we can initialise, delete and continue detection objects correctly.
2. another potential problem in both cases is false detections and miss-detections
3. defining a model which is comprehensive enough to detect objects in different scenarios is another challenge facing in the real world.




### 4. Can you think of ways to improve your tracking results in the future?
1. Fine-tuning parameters is one of the approaches that can improve results. We can check our changes with RMSE plots.
2. another helpful scenario could be using more complex models rather than assuming linear and constant vehicle speed.  

