# Advanced Robotics Course
1. Calculate Sensor Model and Motion Model of Vector Robot in ROS and Gazebo with Python API
   - Sensor Model
   - Motion Model
       - Translation
       - Rotation
2. Implementation of the Particle Filter Algorithm for Vector Robot Localization in Simulation Environment and Real World
   - Monte Carlo + random
   - Augmented Monte Carlo (AMCL) + Particle Random Sampling
   - Augmented Monte Carlo (AMCL) + Particle Random Sampling + N_eff
        - (N_eff = 1 / sum(weight^2) s.t. N_eff < (number of particles / 2 ))
   - Grid Fast SLAM
