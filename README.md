# ROS2diablo
Machine Learning-Based Collision Detection for a Mobile Robot Using Proprioceptive Sensors

1. Problem
Wheeled mobile robots are widely utilized in various applications such as service, entertainment, and transport due to their high maneuverability. However, even when robots are equipped with active sensors like cameras, ultrasonic sensors, or Light Detection and Ranging (LiDAR), collisions with obstacles or impacts from fast-moving objects remain inevitable. These sensors inherently have limitations, including blind spots, response delays, limited resolution, and restricted sensing ranges, which may prevent the timely detection of rapidly approaching objects (He et al., 2007). In office environments, where robots navigate in confined spaces, interact with humans, and encounter unpredictable obstacles like moving chairs or sudden human movements, these challenges become even more critical. Furthermore, if a robot becomes immobilized, high stall currents can occur in its motors, potentially leading to damage. Therefore, it is crucial to implement robust collision detection and post-collision escape behaviors for mobile robots to enhance safety and reliability in office environments.

2 Task Statement
The primary objective of this work is to develop a classification model using a neural network
to detect collisions based on proprioceptive data. This data includes sensor readings from the
robot’s IMUs, motor-related measurements, and control inputs. To extend the collision detection to a more informative multi-class classification task. Once a collision is detected, a secondary neural network model classifies the direction of the impact, identifying whether the
collision occurred on the robot's front, left, or right side.
The second objective is to deploy the trained models on the robot’s onboard computer, enabling real-time monitoring and triggering response behaviors based on the predicted collision.
This mechanism minimizes potential damage and enhances the robot’s ability to safely navigate in cluttered or dynamic indoor environments. The proposed system will be integrated
into the DIABLO platform, a commercially available, two-wheeled self-balancing robot (Direct Drive Technology Limited, 2024). The approach is designed to operate efficiently on a
resource-constrained, low-cost computing platform such as the Raspberry Pi 4, ensuring feasibility for real-world applications.
<img width="945" height="823" alt="image" src="https://github.com/user-attachments/assets/12792c48-b529-4241-8878-8f4c84444655" />

3. Expected Benefits and Functions of the Solution
The goal of this thesis is to develop a collision detection system for an AMR that achieves
high classification accuracy (>99%) for collisions encountered in indoor office environments,
within a command speed range of 0.2–1 m/s. To ensure real-time applicability and minimize
potential damage, the system is designed to respond within 200 milliseconds by triggering an
appropriate response behavior. By relying solely on proprioceptive sensors and control commands, the proposed solution eliminates the need for external tactile sensors, thereby reducing
hardware complexity and preserving the robot’s aesthetic design. To validate the system's
effectiveness and robustness under realistic operating conditions, all experiments will be conducted with a real robot in real-world indoor environments

Watch video
https://youtu.be/GtcHjTE9KhQ
