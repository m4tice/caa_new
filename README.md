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

**Overview**  
The problem of this part is a supervised regression problem, which relates to the car steering angles and the road images in front of a car. The complete pipeline includes three mainphases:  
* Data collection  
* Training  
* Controlling  

**Data collection**  
The first step is to  set up a camera at the front of the vehicle to capture the road images and record the steering angles at the same time. The name of the image and its corresponding steering angle are viewed as feature and label and are put in a csv file.  
The network used for this project is the [NVIDIA](https://developer.nvidia.com/blog/deep-learning-self-driving-cars/) model, which has been proven to work.  
This approach requires a huge amount of data, which is why data augmentation is needed to generate fake data with meaningful label.  

Camera input of the network  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/e2e_input.gif)  

**Training**
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


**Files included in `E2E`**   
* `module_e2e.py` The file includes the functions used for the demonstration  
* `demonstration_e2e.py` Run this file to see the demonstration of the E2E approach 

**Credits**  
* End-to-end Deep Learning for Self-Driving Cars in Udacity: [naokishibuya](https://github.com/naokishibuya)  
* End-to-End Deep Learning for Self-Driving Cars: [NVIDIA](https://developer.nvidia.com/blog/deep-learning-self-driving-cars/)  

# Model Predictive Control  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/mpc_01.gif)
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/mpc_02.gif)

**Credits**  
* Model Predictive Control: [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics).  

# GNSS for controller switching  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/gnss_01.gif)

# LiDAR for obstacle detection and stop    
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/lidar_01.gif)
