<div id="top"></div>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://www.fumrobotics.ir">
    <img src="https://www.fumrobotics.ir/_nuxt/img/fum-logo.bd8ac1a.png" alt="Logo" >
  </a>

  <h3 align="center">Ping Pong Robatic System (PPRS)</h3>
</div>
  
## Install

Install instructions for Ubuntu Bionic.

1. Install at least one simulator,
   [Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Alternativeinstallation:step-by-step) or
   [Ignition Gazebo](https://ignitionrobotics.org/docs/citadel/install)

1. Install the appropriate ROS 2 version as instructed
   [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

  
        source /opt/ros/foxy/setup.bash
        cd ~
        git clone https://github.com/6enyamin/ieee_ws.git 


## Run

        cd ~/ieee_ws/src/applyTorque/worlds/
        gazebo -u world1.world

open new terminal:

        cd ~/ieee_ws/src/applyTorque/scripts/
        python3 publisher1.py  
