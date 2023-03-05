import simpleio
import board
import time
import neopixel
from analogio import AnalogIn
import adafruit_ble
from adafruit_ble.advertising import Advertisement
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.standard.hid import HIDService
from adafruit_ble.services.standard.device_info import DeviceInfoService

from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.keycode import Keycode

# Create HID Service
hid = HIDService()

#Blueooth Advertising Information
device_info = DeviceInfoService(software_revision=adafruit_ble.__version__, manufacturer="Adafruit Industries")
advertisement = ProvideServicesAdvertisement(hid)
advertisement.appearance = 961
scan_response = Advertisement()

# name our device NEOBATv2 for our bluetooth profile
scan_response.complete_name = "NEOBatv2"

ble = adafruit_ble.BLERadio()
if not ble.connected:
    print("advertising")
    ble.start_advertising(advertisement, scan_response)
else:
    print("already connected")
    print(ble.connections)

# set our piezo Pin to A1 Analog
pinA1 = AnalogIn(board.A1)

# Convert into Floating Point
def get_voltage(pin):
    return (pin.value * 3.3) / 65536

# make variable voltage the value of getVoltage(A1) (THIS WILL ENABLE FLOATING POINT VOLTAGE)
voltage = getVoltage(pinA1);

# set our voltage ranges {}
baserange = 0.7
range1 = 1
range2 = 1.3
range3 = 1.6
range4 = 1.9
range5 = 2.2
range6 = 2.5
range7 = 2.8
MaxRange = 3.29

# make the keyboard OBJECT 
k = Keyboard(hid.devices)
kl = KeyboardLayoutUS(k) 

while True:
    while ble.connected:

# HID Programming Array - DO WHILE LOOP (figure out the best way to do the ranges - i dont know figure it out)
# Add neopixels color changer 
		if voltage <= baserange: 
			kl.write("") #non bat hit such as just moving it around

		if voltage >= baserange and <= range1:  
			 kl.write("1") #smallest flick of the bat hit
			 time.sleep(5)

		if voltage >= range2 and <= range2: 
			kl.write("2")
		
		if voltage >= range3 and <= range3:
			kl.write("3")

		if voltage >= range4 and <= range4:
			kl.write("4")

		if voltage >= range5 and <= range5:
			kl.write("5")

		if voltage >= range6 and <= range6:
			kl.write("6")

		if voltage >= range7 and <= range7:
			kl.write("7")

		if voltage >= MaxRange:
		kl.write("8") #sweet spot maximum bat hit

#Start Bluetooth advertising
ble.start_advertising(advertisement)
