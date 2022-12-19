import serial


while True:

    arduino = serial.Serial('/dev/ttyACM0' , 9600) #initialize arduino.
    switch_state= input("write on/off/regular/nitro: ")

    battery_string =(arduino.readline().decode('utf-8').lstrip().rstrip())
    battery_list = battery_string.split(",")
    battery1 = "14.8v power1:" + battery_list[0]+ "%"
    battery2 = "14.8v power2:" + battery_list[1]+ "%"
    battery3 = "18.5v power3: " + battery_list[2]+ "%"
    battery4 = "PC: " + battery_list[3]+ "%"
    battery5 = "canon:" + battery_list[4] + "%"
    battery_meter = [battery1, battery2,battery3,battery4,battery5]
    print(battery_meter)

    if switch_state == "off": #off switch
        arduino.write(b'1')
    elif switch_state == "on": #on switch
        arduino.write(b'3')
    elif switch_state == "regular": #14.8v battery switch
        arduino.write(b'0')
    elif switch_state == "intro": #18.5v battery switch
        arduino.write(b'2')


