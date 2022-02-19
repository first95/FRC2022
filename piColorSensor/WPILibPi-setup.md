# Setting up the WPILibPi for rpi-colorsensor.py

1. [Installing the image to your MicroSD card](https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/installing-the-image-to-your-microsd-card.html)
2. Browse to [http://wpilibpi.local](http://wpilibpi.local)
3. ![wpilibpi network setup](https://user-images.githubusercontent.com/55641973/154821076-306a39e8-4455-493e-a3f4-54a327ddafbd.png)
4. ![wpilibpi Network Tables setup](https://user-images.githubusercontent.com/55641973/154821087-9b00eba4-2939-47be-8bd2-337e5d69af49.png)
5. ![wpilibpi upload script](https://user-images.githubusercontent.com/55641973/154821092-4e599492-433a-4009-8079-03358bd4ce08.png)
6. Open an SSH session to the raspberry pi, and log in with the username of "pi" and the password of "raspberry".  PuTTY is a popular program for ssh, but Windows 10 also ships with the OpenSSH client, so just Win+R and typing "ssh pi@wpilibpi.local" and clicking OK should work too.
7. Run the un-commented lines from [rpi-colorsensor-setup.sh](https://github.com/first95/FRC2022/blob/main/piColorSensor/rpi-colorsensor-setup.sh)
8. Wire up a color sensor, reboot and see if it's working.  

You can view Network Tables data using the OutlineViewer program included in the Wpilib software.  Two non-obvious things that help with that software:
1. set the IP address of the computer running OutlineViewer to the 10.TE.AM.2 (the IP address the RoboRio will use, so change it before re-connecting to the robot.
2. Make sure Windows Firewall rules allow incoming connections to outlineviewer.exe on TCP port 1735
![image](https://user-images.githubusercontent.com/55641973/154821800-2a3d96a6-7e7a-47c6-9f6b-8766b13b852f.png)
