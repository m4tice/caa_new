# Carla Autonomous Application

Repository created for Master Thesis: Autonomous Driving

**SOFTWARES and FRAMEWORKS:**  
* [CARLA](https://github.com/carla-simulator/carla) > 0.9.10  
* [Conda](https://docs.conda.io/en/latest/)  
* Python

**IMPORTANT REQUIREMENTS:**  
* Python > 3.7.x  
* Tensorflow > 2.1.x  

**INSTRUCTIONS:**  
Install the Conda environment, which contains the necessary libraries by running the following commands:  

```
conda env create -f environment.yml
conda activate tf_gpu
```

After finishing CARLA installation, clone this repo and place it as follows:  

    .
    ├── ...
    ├── PythonAPI
    │   ├── caa_new <<===          
    │   ├── carla             
    │   ├── examples                      
    │   └── util                
    └── ...


# End-to-end Deep Learning for Autonomous Driving  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/e2e_01.gif)
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/e2e_02.gif)

**OVERVIEW**  
The problem of this part is a supervised regression problem, which relates to the car steering angles and the road images in front of a car. The complete pipeline includes three mainphases:  
* Data collection  
* Training  
* Controlling  

**DATA COLLECTION**  
The first step is to  set up a camera at the front of the vehicle to capture the road images and record the steering angles at the same time. The name of the image and its corresponding steering angle are viewed as feature and label and are put in a csv file.  
The network used for this project is the [NVIDIA](https://developer.nvidia.com/blog/deep-learning-self-driving-cars/) model, which has been proven to work.  
This approach requires a huge amount of data, which is why data augmentation is needed to generate fake data with meaningful label.  

Camera view of the road  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/recorded_01.png)
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/recorded_02.png)
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/recorded_03.png)

Camera input of the network  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/e2e_input.gif)  

**TRAINING**  
The re-designed model is based on the work of Mister [naokishibuya](https://github.com/naokishibuya/car-behavioral-cloning). The architecture of the model is as follows:  
* Image normalization  
* Convolution: 5x5, filter: 24, strides: 2x2, activation: ELU  
* Convolution: 5x5, filter: 36, strides: 2x2, activation: ELU  
* Convolution: 5x5, filter: 48, strides: 2x2, activation: ELU  
* Convolution: 3x3, filter: 64, strides: 1x1, activation: ELU  
* Convolution: 3x3, filter: 64, strides: 1x1, activation: ELU  
* Drop out (0.5)  
* Fully connected: neurons: 100, activation: ELU  
* Fully connected: neurons: 50, activation: ELU  
* Fully connected: neurons: 10, activation: ELU  
* Fully connected: neurons: 1 (output)  

**DATA AUGMENTATION**  
During the training, the process of augmentation applied on to the images are performed randomly. The augmentation methods include:  
* `random_translation` Translated the image randomly and compute the new steering angle corresponding to the movement of the image on the x-axis.
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/translated_sample.PNG)  

* `random_flip` Randomly flip the image and change the sign of the steering value as positive of negative responsively.  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/flipped_sample.png)  

* `random_shadow` Randomly create a random region of darkness, which imitates the shadow in real life. This helps the model to be more generalised.  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/shadow_sample.png)  
* `random_brightness` Randomly adjust the brightness of the image, which imitates the brightness of the sun, lamps, etc.  

**TRAINING RESULT**  
The training start with the following parameters:  
* Number of samples: 12000  
* EPOCHS: 50  
* Step per epoch: 10000  
* Batch size: 40  
* Learning rate: 1.0e-4  
<img src="https://github.com/m4tice/caa_new/blob/main/assets/training_result.png" width="60%">  
<img src="https://github.com/m4tice/caa_new/blob/main/assets/e2e_test_01.png" width="60%">
<img src="https://github.com/m4tice/caa_new/blob/main/assets/e2e_test_02.png" width="60%">
<img src="https://github.com/m4tice/caa_new/blob/main/assets/e2e_test_03.png" width="60%">  

**FLIES INCLUDED - `E2E`**   
* `module_e2e.py` The file includes the functions used for the demonstration  
* `demonstration_e2e.py` Run this file to see the demonstration of the E2E approach  

**CREDITS**  
* End-to-end Deep Learning for Self-Driving Cars in Udacity: [naokishibuya](https://github.com/naokishibuya)  
* End-to-End Deep Learning for Self-Driving Cars: [NVIDIA](https://developer.nvidia.com/blog/deep-learning-self-driving-cars/)  

# Model Predictive Control  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/mpc_01.gif)
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/mpc_02.gif)

**OVERVIEW**  
The MPC controller control the throttle and steering of the vehicle based on a linearised model of the vehicle. The basic idea is that a reference path is provided and the goal of the controller is find a path the has the smallest cost difference comparing with the reference path. This is done by generating all the possible paths using the throttle and steering applied on the linearised model to predict the next positions of the vehicle with a certain amount of time step. The part is based on the work of Mister [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics).  

State vector of the vehicle model includes xy position, velocity and yaw angle.    
<img src="https://render.githubusercontent.com/render/math?math=z=[x, y, v, \phi]">  

Input vector of the vehicle model includes acceleration and steering.    
<img src="https://render.githubusercontent.com/render/math?math=z=[a, \delta]">  

State matrix:  
<img src="https://github.com/m4tice/caa_new/blob/main/assets/state_matrix.png" width="40%">  

Input matrix:  
<img src="https://github.com/m4tice/caa_new/blob/main/assets/input_matrix.PNG" width="40%">  

**RESULTS**  
<img src="https://github.com/m4tice/caa_new/blob/main/assets/mpc_loc_01.png" width="60%">  
<img src="https://github.com/m4tice/caa_new/blob/main/assets/mpc_loc_02.png" width="60%">  

**CREDITS**  
* Model Predictive Control: [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics).  

# GNSS for controller switching  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/gnss_01.gif)

# LiDAR for obstacle detection and stop    
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/lidar_01.gif)
