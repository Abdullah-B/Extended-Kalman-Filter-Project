# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project we  utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.  


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


## Changes and Results

as per what was requested for this project the following files were modified to fullfil the requirments of the project

   ### 1 : FusionEKF.cpp

Finished all the initializations indicated in the comments

   ### 2 : FusionEKF.h

added   float noise_ax;
        float noise_ay;

   ### 3 : kalman_filter.cpp

Coded the Predict(),Update(const VectorXd &z), and UpdateEKF(const VectorXd &z)

   ### 4 : Kalman_filter.h

added   Tools tools;
and
        Eigen::MatrixXd Hj_; 

   ### 5 : tools.cpp

added the code to calculate the RMSE inside 
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
and the code to calculate the Jacobian inside 
    MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
    
   ### The resutls were
    
   ![Screenshot](/OutPut&Results/Result.PNG)
   
   and the Matrix outputs are documented inside OutPut & Results/ Consol_OUT_PUT.txt
   

    














