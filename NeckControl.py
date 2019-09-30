#import libraries
import os
import time

yaw=5 # Vertical.
pitch=2 # Horizontal. 
yawValue = 50 # Yaw percentage value. Never use over 50%.
pitchValue= 50 # Pitch percentage value.

# Start the servo motors
os.system('sudo /home/pi/PiBits/ServoBlaster/user/servod')
os.system('cd PiBits/ServoBlaster/user/')
os.system('sudo servod --min=650us --max=2350us')
# Reset position
os.system("echo %d=%d%% > /dev/servoblaster"%(int(yaw),int(yawValue)))
os.system("echo %d=%d%% > /dev/servoblaster"%(int(pitch),int(pitchValue)))
time.sleep(1.0)

"""
#test
#Up
os.system("echo %d=%d%% > /dev/servoblaster"%(int(yaw),int(0)))
time.sleep(0.5)
#Down
os.system("echo %d=%d%% > /dev/servoblaster"%(int(yaw),int(50)))
time.sleep(0.5)
#Left
os.system("echo %d=%d%% > /dev/servoblaster"%(int(pitch),int(100)))
time.sleep(0.5)
#Center
os.system("echo %d=%d%% > /dev/servoblaster"%(int(pitch),int(50)))
time.sleep(0.5)
#Right
os.system("echo %d=%d%% > /dev/servoblaster"%(int(pitch),int(0)))
time.sleep(0.5)
#Center
os.system("echo %d=%d%% > /dev/servoblaster"%(int(pitch),int(50)))
time.sleep(0.5)
"""

"""
try:
    print('Press [ESC] to quit')
    # Loop indefinitely
    while True:
        
except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    MOVE_STOP()
    
    """

# Stop the servo motors 
os.system('sudo killall servod')