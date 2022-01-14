# 2019  (Cecilia Payne, Destination Deep Space)
Noteworthy software components:

## Vision coprocessor

The vision coprocessor was a Raspberry Pi model 3 mounted to the robot, connected to a network switch along with the RoboRIO and radio.  We wrote a separate application, which was stored in a VS code project in [this folder](https://github.com/first95/FRC2019/tree/master/VisionCoprocessor).  The raspberry pi was configured with the directions [here](https://docs.wpilib.org/en/latest/docs/software/vision-processing/raspberry-pi/using-the-raspberry-pi-for-frc.html)

Development of the vision coprocessor application was made much easier by writing two `main()` functions: one designed to run on a desktop PC, and one designed to run on the Pi.  The one on the desktop was configured to loop through a number of test input images and display the artifacts of its analysis.  This was great for testing and development, especially prior to having the real-world source of images ready.

## Significant use of the command pattern

This was the first year we really took advantage of the scheduler capabilities of the command pattern.  See this command in particular: [Command to pick up a hatch cover off the ground and transfer it to the gripper](https://github.com/first95/FRC2019/blob/master/FRC2019/src/main/java/frc/robot/commands/compound/PickupAndHandoffGroundHatch.java) Remember to click down in through the dependencies to understand it fully.

## First use of the controller's rumble packs

However, note that driver's station support for this was buggy until 2020 (only supported one controller).  So check out the [implementation in the 2020 codebase](https://github.com/first95/FRC2020/blob/master/FRC2020/src/main/java/frc/robot/commands/RumbleCommand.java).  Note that much of the meat of the rumble support is actually in the [OI class](https://github.com/first95/FRC2020/blob/master/FRC2020/src/main/java/frc/robot/OI.java).

# 2018 (Kepler and Doppler, FIRST Power Up)

## Conditional auto moves

This game was the first one to incorporate "game data" delivered to the software by the field management system during the match.  We set up a programmable system of auto moves, such that the drive team could specify in advance which auto move should occur in each condition that could arise.  This technique is likely to be useful in other games.

As part of this we made a version of the `SendableChooser` class that could be changed after construction, shown  [here](https://github.com/first95/FRC2018/blob/master/FRC2018/src/org/usfirst/frc/team95/robot/oi/MutableSendableChooser.java).  I think this was necessary to populate the listings upon robot startup rather than hardcoding everything, but I don't entirely remember the justification.  The interesting thing was that it was possible to extend the UI classes in the smartdashboard with some level of success.

## Auto-acquire

Kepler had a series of optical proximity sensors in its maw.  We had an effective system for detecting when a sufficient count of those photosensors was triggered, and when enough of them indicated the presence of a solid object, and grab the item.  [This class](https://github.com/first95/FRC2018/blob/master/FRC2018/src/org/usfirst/frc/team95/robot/commands/collector/AutoCloseMawOnCube.java)  This may have been the first use of the command pattern for sequencing.

# 2017 (Orville and Wilbur Wright, Steamworks)

# 2016 (Kovaka, FIRST Stronghold)

This year had a noteworthy near miss, safety-wise.  The "cannon" was controlled in elevation as a closed-loop position control system.  It was very heavy, so the motors driving it were very strong (I think it was several CIMs in parallel).  We had a working encoder reporting the current position, but no indexing sensor.  This builds an implicit assumption into the software: whatever position the arm is at when the robot is powered on is zero degrees (or whatever angle).  The robot looked vaguely front-back symmetric.  I don't remember the precise setup, but somehow, a student tried to command the cannon to point straight upward, and the cannon instead tried to seek a position straight downward, with quite some force.  The student hit the disable button shortly after the cannon slammed into the floor, hard enough to be felt through the whole classroom.  Nobody was hurt, because everyone was doing a good job of staying clear from the robot while it was being debugged.  The robot escaped damage too.  I call it a near miss because of the sheer force imparted by the robot, and how badly things could have been if someone was too close.

Things that have been done to reduce this class of problem since:
- Indexing sensors (such as the one on Kepler and Ceclia's elevators)
- Absolute encoders where applicable
- Making sure students look around the robot before enabling, and yelling "enabling" (I think we were doing this already, but this incident underscores its importance to me)

# 2015 (Ada Lovelace, Recycle Rush)

