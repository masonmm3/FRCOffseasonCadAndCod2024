# FRCOffseasonCadAndCode2024

This is a project I did to learn some new things.

### Starting with the [CAD](https://cad.onshape.com/documents/7b13049c2cde382fdd62978c/w/b0b2d0bd70eaff5d44976ab3/e/618cb2263d489624640a6db2?renderMode=0&uiState=676de51d1038fe27e777eb76). 
* This was my first time doing a mastersketch and using the derrived feature.
Overall happy with that and loved that. I also attempted to approach and elevator and learned quite a bit from that. The elevator in the model needs redone along with most of the packaging. The packaging ended up worse than expected and the climber was scrapped because of this and time as this project took a week. The elevator breaks frame perimeter and physics so it was ignored going forward. the robot also wound up 0.5" over frame perimeter limit. A lot of learning from this.
![image](https://github.com/user-attachments/assets/f3a50dda-3c09-4e25-8a51-0d27c28d4258)


The folder Robot_FrankenstiensMonster is the advantage scope model. The elevator component has not been setup. The name is a callback to the inseason Robot I competed with on 6366 being Frankenstien.

### Moving to Code
* this is when I get into my comfort zone. The team I am mentoring in 2025 used a different swerve library meaning I needed to learn some new libraries.
* The libraries in use in this code are YAGSL, MAPLESIM, AKIT, PHOTONVISION, REVLIB, PATHPLANNER. I am familiar with all but one of these but had some more to learn.
* YAGSL was the new one so I spent a lot of time on that.
* I had previously used it in the 23 offseason when learning swerve for the first time and had a miserable experience in an offseason event as a result of just wierd things YAGSL would do. I swaped to the 23 akit advanced swerve template for 24 changing and improving it for use with the ctre ecosystem once we made the switch about week 3 of build.
 * YAGSL was overal a better system being less of a pain than last time and the only thing that didnt work as intended in sim was their point at speaker script having some horrible math error. I ended up redoing it anyways to make room for some features I wanted.
 * YAGSL made implementing MAPLESIM and the sim for PHOTONVISION simple. the Vision code provided in their example needs some work particularly in regard to ease of access to the camera settings but it behaved well and seemed to at a minimum not a step back from my previous code which was adapted from FRC 281 midseason after limelight failed hilariously on 6366s robot multiple times.
 * I can only really complain about being unable to get the target states from their api which was only a result of me doing a custom set of code that didnt use the default logger as I have had it crash network tables on many occasions.
* now for features. The code is based around a teleop mode called smart aim. Smart aim uses distance to features on the field to determine what the robot should be doing. rotating to angles to make game pieces more likley to be spotted by rear cams and pointing at the speaker and pass points. It however has one other trick to make it even less obtrusive. Unless the driver is asking to shoot the driver joystick can simply steer overtop of the steer pid and normalize the aim to point out of existance until they release, it also can be set to use SWM (shoot while moving) aim to adjust for sidestep in aim.
* the code is designed to use a decision matrix of some kind to make it adaptable to multiple uses instead of requiring a rewrite like the base yagsl system.
* The shooter which is based around the MAPLESIM ecosystem has a few tricks. The game pieces are handled by the Indexer subsystem and uses a handoff and checkpoint system to improve simulation accuracy by requiring the game piece to move between points with a delay and storing a location for visualization of the gamepiece in the robot. The shooter is also setup for SWM however the field velocity has to be made negative on blue for somereason so there are still math errors. The code uses 4 interpolating double tree maps for aim point generation. speed and angle for both passing and shooting.
* The code is missing some things I wish it wasnt such as and IO file for controllers and some redundant features on a second controller such as manual shooting setpoints, shoot overide for the autoshoot, and a way to disable the smart aim.
* From an auto perspective the code is fully featured allowing for swm and stationary shooting using two scripts.
* All code is abstracted using the Akit system and because of time real hardware was never coded so everything is currently sim only.

Had a lot of fun working through this and hope those who come across this can learn from it. 

[Video Showcase](https://youtu.be/myx9rAiUxsQ)



