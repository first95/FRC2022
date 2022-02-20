# FRC Team 95's version of rpi-colorsensor.py
This year, with two-color cargo, it became clear that using a color sensor would be important.

The Kit of Parts included Rev Color sensors (v3), which are I2C sensors at a fixed bus ID

According to the [Known Issues](https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html) document for 
WPILib 2022, the current release of the RoboRIO's OS has a bug that can lock up the RoboRIO if the on-board I2C port is used.

That document claims the I2C pins in the MXP are unaffected, however we've heard through the grapevine that it may cause lockups as well.

Out of an abundance of caution, we decided to tack on a raspberry pi and have it read the color sensor and publish the data to NetworkTables.

It would appear that the folks at WPI have anticipated this need, and have released [rpi-colorsensor](https://github.com/PeterJohnson/rpi-colorsensor)

That code uses getRawColor, however the Java code we had already written using the on-board I2C port was using getColor, so this is our adaptation of wpi's code (only the main code at the very bottom has been modified) to minimize the changes needed to read the data from NetworkTables instead of directly from a connected sensor.

We did start with a raspberry pi 4 running [WPILibPi](https://github.com/wpilibsuite/WPILibPi/releases) since the script's setup instructions were written with that release in mind.

To make it even easier, I've compiled the setup steps outlined in the rpi-colorsensor's README.md file into a script, called pri-colorsensor-setup.sh for those with little to no Linux command-line experience.  It's written to be a script that could be transferred to the pi and run, but the individual lines in the script can be copied and pasted into the ssh session window as well