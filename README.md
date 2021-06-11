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

Camera input of the network  
![Alt text](https://github.com/m4tice/caa_new/blob/main/assets/e2e_input.gif)  

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
