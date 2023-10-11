from gortmcp import GortMCP23017
import smbus
import time
import rospy

mcp = GortMCP23017.MCP23017(1,0x20)

INPUT = 0xFF
OUTPUT = 0x00
HIGH = 0xFF
LOW = 0x00

mcp.pin_mode(8,OUTPUT)
mcp.pin_mode(1,INPUT)
mcp.digital_write(8,HIGH)

while True:
	try:
		mcp.digital_write(8,HIGH)
	except:
		print("error WH")
	
	time.sleep(0.05)
	try:
		mcp.digital_write(8,LOW)
	except:
		print("error WL")
	time.sleep(0.05)	
	try:
		number = mcp.digital_read(1)
	except:
		print("error R")	
		
	print(number)
	time.sleep(0.05)
		
		
