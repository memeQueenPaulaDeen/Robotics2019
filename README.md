# Robotics2019

Need to install a few packages before hand to make it all work

sudo apt-get install libatlas-base-dev
sudo apt-get install numpy

might be a few others

To get the most recent version of the code type the following into the console on the pi and then your laptop:

git clone https://github.com/memeQueenPaulaDeen/Robotics2019.git

On the Pi we will run the driver.py code on our laptop (the server) we will run matlab.


There is one line of code that we will need to change in LocalizationClient.py it is the one that tells the pi your laptops. Internet address. 

Open a CMD window and type ipconfig
look for the feild labled IPv4: and copy the number it should look like x.x.x.x

and replace the adress in this line of the LocalizationClient.py

Also be sure that the fire wall on your laptop is turned off, Windows defender setting

class LocalizationClient():

	#TCP_IP = '127.0.0.1'
	TCP_IP = '192.168.1.5' <--- paste in the new IP adress here
  .
  .
  .
 
  
  
save the file and run the driver code
