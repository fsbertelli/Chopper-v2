import pyrealsense2 as rs
import numpy as np
import depthai as dai
import cv2
import RPi.GPIO as GPIO
import time
import imutils
import serial
from controller.imageController import ImageController

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
gpgga_info = "$GPRMC,"
GPGGA_buffer = 0
NMEA_buff = 0

def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
    
print("Load all libraries")
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
ledPin = 12
buttonPin = 7
GPIO.setup(ledPin, GPIO.OUT)
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#camera quadrada
print("Create camera config")
pipeline = dai.Pipeline()
camRgb = pipeline.createColorCamera()
xoutRgb = pipeline.createXLinkOut()
xoutRgb.setStreamName("rgb")
camRgb.setPreviewSize(1280, 720)
camRgb.preview.link(xoutRgb.input)
camRgb.setFps(60) 
device_info = dai.DeviceInfo("172.16.2.100")

#realsense
# Configure depth and color streams
pipeline_rs = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline_rs)
pipeline_profile = config.resolve(pipeline_wrapper)
device_rs = pipeline_profile.get_device()


config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Start streaming

pipeline_rs.start(config)

print("Starting camera ...")
with dai.Device(pipeline, device_info) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    i = 0
    while True:
        received_data = (str)(ser.readline()) #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                
        buttonState = GPIO.input(buttonPin)       
        # get frame oak-d
        frame = qRgb.get().getCvFrame()
        show_frame = imutils.resize(frame, width=640)
        # get frame realsense
        frames = pipeline_rs.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
           
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPRMC,",1)[1]  #store data coming after “$GPGGA,” string
            NMEA_buff = (GPGGA_buffer.split(","))
            nmea_time = []
            nmea_latitude = []
            nmea_longitude = []
            nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
            nmea_latitude = NMEA_buff[2]                #extract latitude from GPGGA string
            nmea_longitude = NMEA_buff[4]               #extract longitude from GPGGA string
            nmea_date = NMEA_buff[8]
            lat = (float)(nmea_latitude)
            lat = convert_to_degrees(lat)
            longi = (float)(nmea_longitude)
            longi = convert_to_degrees(longi)
            hora = str(nmea_time[0:2]+":"+nmea_time[2:4]+":"+nmea_time[4:6])
            data = str(nmea_date[0:2]+"-"+nmea_date[2:4]+"-"+nmea_date[4:6])
           
            if (buttonState == False):
                GPIO.output(ledPin, GPIO.HIGH)
                name_image_oak = "./OAK1/{} {} {},{}.jpg".format(data,hora,lat,longi)
                name_image_rs = "./realsenseprint/{} {} {},{}.jpg".format(data,hora,lat,longi)		
                cv2.imwrite(name_image_oak, frame) 
                cv2.imwrite(name_image_rs, color_image)
                time.sleep(0.8)
                GPIO.output(ledPin, GPIO.LOW)            
	
                print(f"OAK1: *{name_image_oak}* saved.")
                print(f"REALSENSE: *{name_image_rs}* saved.")
                 
            else:
                GPIO.output(ledPin, GPIO.LOW)
                cv2.imshow("OAK1", show_frame)
                cv2.imshow("realsense", color_image)
            key = cv2.waitKey(1) & 0xFF
			
            if key == ord('q'):
                pipeline.stop()
                break
       
pipeline.stop()
        
