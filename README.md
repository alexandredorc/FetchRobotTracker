# FetchRobotTracker
Fetch robot following a guider: Assesment 3 Mecatronic

Using  the  learnt  and  state-of-art  control  algorithm  to  control  the  Fetch  robot  to  follow  the  path  of  
the  guider  in  front.  Both  depth  images  and  RGB  images  can  be  used  to  track  the  guider.  Special 
designed artificial markers or patterns can be put on the back of the guider. 
A  map  of  the  environment  needs  to  be  built  and  a  localisation  algorithm  is  also  required.  Open  
source code are available for these two parts but the group needs to be able to make them working 
on in the simulator (and the real robot).  
Target: The Fetch robot follows the guider in front (maintaining a certain distance) in the office and 
corridor  environment.  The  project  will  be  first  done  using  a  Fetch  simulator;  once  the  simulator  
works well, the real robot can be used if it is allowed to access the lab. 


## How to install

Simulateur en ligne de ros
https://app.theconstructsim.com/Desktop 


autoriser les telechargement apt :
      echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list 
update apt
      sudo apt update
      
installer les packet necessaires
sudo apt install ros-melodic-fetch-calibration ros-melodic-fetch-open-auto-dock \
ros-melodic-fetch-navigation ros-melodic-fetch-tools -y
sudo apt install ros-melodic-fetch-gazebo

changer le nom du package en tracker_robot

mettre le model dans la le dossier .gazebo/models
