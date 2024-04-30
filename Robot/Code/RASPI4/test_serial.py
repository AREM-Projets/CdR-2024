import serial



print("Connexion à l'embase...")
port = serial.Serial('/dev/ttyACM0', 115200)
port.write(b'e')