# Traffic-Flow-Estimation
#Details################ 

Simulation of our proposed protocol on a road network (Highway) with 5 RSUS. Distance between two-RSUs is 5km.  Each vehicle speed varies 50-60 km/h. Bloom filters are created every five minutes for each RSU and sent for estimation at different time intervals.   

Bloom filteres at an RSU with different time-span  are aggregated for Estimation.  
 

#For Compiling###############
 
c++ main.cpp -lcrypto -lgmp -lpthread


###For Running####

./a.out

## Simulation Result file formate

Start-Receiver, End-Receiver, time0-flow-estimation, time1-flow-estimation, time2-flow-estimation, time3-flow-estimation, time4-flow-estimation,....
