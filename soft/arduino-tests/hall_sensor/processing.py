import serial
import time
import csv

with open('SensorData.csv', mode='a') as sensor_file:
    sensor_writer = csv.writer(sensor_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    sensor_writer.writerow(["Sensor0","Sensor1","Sensor2","Sensor3",])


com = "/dev/ttyUSB1"
baud = 9600

x = serial.Serial(com, baud, timeout = 0.1)

while x.isOpen() == True:
    data = str(x.readline().decode('utf-8')).rstrip().split(',')
    if len(data) == 4:
         print(data)
         with open('SensorData.csv', mode='a') as sensor_file:
             sensor_writer = csv.writer(sensor_file, delimiter=',', quoting=csv.QUOTE_MINIMAL)
             sensor_writer.writerow(data)