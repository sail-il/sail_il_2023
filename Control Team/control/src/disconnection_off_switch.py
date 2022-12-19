from multiprocessing import connection
import serial
import subprocess
import time

arduino = serial.Serial( '/dev/ttyACM0' , 9600)
#IMPORTANT: /dev/ttyACM0 is the CURRENT connected port.
#in order to check the connected port write: "ls /dev/tty*" once with the arduino connected and once without and see what port was added.
#the added port is the new port.     9600 is the baud rate configured in the ARDUINO CODE. 
#print("kosomoooo")
while True:
    p = subprocess.Popen(['ping','192.168.1.100','-c','1',"-W","2"]) #insert the right IP
    # The -c means that the ping will stop afer 1 package is replied 
    time.sleep(4)
    p.wait()
    hello = p.poll()
    print (hello)
    if hello == 1:
        arduino.write(b'3')
        break # whatever chen wants



