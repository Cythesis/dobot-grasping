# Chef's Third Arm
Developed by:

Patrick Dowe

David Dao

Carl Hemsworth

Adelaide Yuill

Brendan Edwards

### Scenario
The Dobot and conveyor are to be installed as part of an automated pantry. When an operator selects an item, the robot locates it within the pantry, retrieves it and places it on the conveyor belt to be taken out to the user. When the user is done with the item, they can return it to the conveyor. Returning items are identified by the robot and placed back in their previous position.
This scenario will utilise the Dobot and conveyor setup in the Mechatronics Lab. Our group has purchased an Xbox 360 Kinect to act as a visual sensor in the system and will be installed looking down at the setup. Items will be labelled with AR tags to allow for pose estimation by the vision system.

## Outline
The main code is a GUI program storred in the src file. Code functionality exists within classes storred in the lib folder. 
### Storring Food
1. Place container on conveyor.
2. In the GUI, select the container type and input the name of what is being storred.
3. Once selection is convirmed in the GUI, conveyor will feed the container into the enclosure. Using the vision system, the Dobot will collect the container and placed it one one of the shelves depending on its type.

### Retrieveing Food
1. Select from containers that have been storred in the workspace.
2. Vision system allows the Dobot to find requested container and place it on the conveyor.
3. Conveyor will carry the container out of the enclosure.
