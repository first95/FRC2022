#!/bin/sh
# This is just the setup steps from WPI's rpi-colorsensor.py README.md file converted to copy/pastable steps.

# before runing these steps, you can upload the rpi-colorsensor.py file to the raspberry pi using the web console.
# At the top of the web console, click Writable to re-mount the filesystem with read/write capabilities
# In the menu on the left, choose Application.
# in the panel on the right, use Choose File in the File Upload section to select rpi-colorsensor.py on your computer and click Upload

# now make sure you have network connectivity to the pi ("ping wpilibpi.local" is a good way to test)
# and ssh to the raspberry pi - Windows 10 now ships with OpenSSH client, so you can just use Start button and type:
#  ssh pi@wpilibpi.local
# and use the password "raspberry" when prompted

# If the prompt includes "(ro)" (it shouldn't after clikcing Writable above) put the raspberry pi into Read-Write mode
rw

# Remove any carriage returns in the file - \x0d is the method to tell sed to search for ascii 18 (0d in hex) 
sed -e 's/\x0d//' -i /home/pi/rpi-colorsensor.py 


# set the pi's GPIO server to run automaticall at boot
sudo systemctl enable pigpiod

# add "i2c-dev" to the end of /etc/modules to load the kernel module at next boot
sudo /bin/sh -c 'echo i2c-dev >> /etc/modules'

# uncomment the i2c line from the /boot/config.txt file
# the sed (stream editor) command is searching for a line that contains "i2c" (first text between //'s)
#        the s (substitute) command then replaces "#" with nothing /#//
#        -i tells gnu sed to do an in-place edit of the file (actualy creates a temp file, then renames it to the original name)
sudo sed -e '/i2c/s/#//' -i  /boot/config.txt

# Make a directory for the files needed to create an auto-start service out of the rpi-colorsensor.py script
sudo mkdir -p /service/colorsensor

# create the startup script, called "run" in that folder, using a "here" document (text between << and the end-of-text marker EOF)
sudo /bin/sh -c 'cat > /service/colorsensor/run << EOT
#!/bin/sh
exec /home/pi/rpi-colorsensor.py
EOT
'
# change the permissions on the run file to be executable (x) for all (a) users 
sudo chmod a+x /service/colorsensor/run

# create a symbolic link in /service/colorsensor called supervise, pointing to /tmp/colorsensor-supervise
sudo ln -s /tmp/colorsensor-supervise /service/colorsensor/supervise

# create a symbolic link in /setc/service called colorsensor pointing to /service/colorsensor 
# this symlink enables auto-starting the service at boot time.  Skip this step if you want to modify your version of rpi-colorsensor.py
# on the raspberry pi and run it manually (via an ssh session) until it's ready to be set up to start automatically
sudo ln -s /service/colorsensor /etc/service/colorsensor

# all done making changes, so set it to read-only again
ro