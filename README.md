# AERA_Webots

## Installation:
### Webots
Go to https://cyberbotics.com/#download and download the Webots R2023a version and install as described by Webots.

### AERA_webots repo
Clone this repository to your workspace.  
Then open the Visual Studio solution file of your choice of controller (e.g., AERA_Webots/controllers/aera_hand_controller_protobuf/aera_hand_controller_protobuf.sln) in Visual Studio.  
Build the project (don't run, only build).  

## Run Webots demo:
Change the settings.xml file to use the right seed code (e.g., hand-grab-sphere-learn.external.replicode) and set the I/O-Device to the tcp_io_device.  
Run AERA.  
Open Webots.  
Select File->Open World... and select the correct Webots world which includes the robot controlled by the controller that was built earlier (e.g., AERA_Webots/worlds/aera_hand_robot.wbt).  
Start the simulation with the small "Play" button on top of the screen.  
