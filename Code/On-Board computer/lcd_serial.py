
#Command Name Description 0xC6
#0x00 null (ignored) Ignored as a no operation
def lcd_ignore(ser):
	inpt="NEX\xc6\x00" 
	ser.write(inpt)
	return True
	
#0x01 Cursor Home Sets the cursor to the home position (top left)
def lcd_cursor_home(ser):
	inpt="NEX\xc6\x01" 
	ser.write(inpt)
	return True
	
#0x02 Set cursor (1-80 or 32) 
#Cursor to a position specified by the next byte, where 1
#is the top left and 80/32 is the bottom right
def set_cursor(ser,value):
	temp="c602%s" %value
	inpt="NEX" + binascii.unhexlify(temp)
	#inpt="NEX\xc6\x02" 
	ser.write(inpt)
	return True

#0x03 set cursor (line,column)
#Sets cursor using two bytes, where first byte is the line and the second byte is the column
def set_cursor(ser,line, column):
	temp="c60%s0%s" %(line,column)
	inpt="NEX" + binascii.unhexlify(temp)
	#inpt="NEX\xc6\x02" 
	ser.write(inpt)
	return True

#0x04 Hide cursor stops the position cursor from appearing on the display
def hide_cursor(ser):
	inpt="NEX\xc6\x04" 
	ser.write(inpt)
	return True
# 0x05 Show underline cursor Changes the cursor to the underline type
def show_underline(ser):
	inpt="NEX\xc6\x05" 
	ser.write(inpt)
	return True
#0x06 Show blinking cursor
#Changes the cursor to the blinking type
def show_blinking(ser):
	inpt="NEX\xc6\x06" 
	ser.write(inpt)
	return True
	
#0x08 Backspace deletes the preceding character from the current position
#on the display
def backspace_del(ser):
	inpt="NEX\xc6\x08" 
	ser.write(inpt)
	return True
	
#0x09 Horizontal tab (by tab set)
#Moves the current position across by the tab space set
#by command 18 (default tab space 4)
def h_tab(ser):
	inpt="NEX\xc6\x09" 
	ser.write(inpt)
	return True

#0x0A Smart line feed Moves the cursor down one line to the position beneath
#in the same column
def smart_next_line(ser):
	inpt="NEX\xc6\x0A" 
	ser.write(inpt)
	return True
	
#0x0B Vertical tab Moves the cursor up one line to the position above in
#the same column
def vertical_tab(ser):
	inpt="NEX\xc6\x0B" 
	ser.write(inpt)
	return True
	
#0x0C Clear screen Clears the screen and sets cursor to the home position
def clear_screen(ser):
	inpt="NEX\xc6\x0C" 
	ser.write(inpt)
	return True
	
#0x0D Carriage Return Moves the cursor to the start of the next line
def next_line(ser):
	inpt="NEX\xc6\x0D" 
	ser.write(inpt)
	return True
	
#0x0F Software version Module returns a single byte software version
def software_version(ser):
	inpt="NEX\xc6\x0F" 
	output=ser.write(inpt)
	return output
	
#0x11 Clear Column Clears the contents of the current column and moves
#cursor right by one column
def clear_column(ser):
	inpt="NEX\xc6\x11" 
	ser.write(inpt)
	return True
	
#0x12 Tab set Sets the required tab size, the following byte can be a
#size of between 1 and 10
def tab_size(ser,size):
	temp="c612%s" %size
	inpt="NEX" + binascii.unhexlify(temp)
	#inpt="NEX\xc6\x00" 
	ser.write(inpt)
	return True
	
#0x13 Backlight on Turns the backlight of the LCD03 on
def backlight_on(ser):
	inpt="NEX\xc6\x13" 
	ser.write(inpt)
	return True
	
#0x14 Backlight off
#(default)
#Turns the backlight of the LCD03 off
def backlight_on(ser):
	inpt="NEX\xc6\x14" 
	ser.write(inpt)
	return True
	
#0x15 Disable startup
#message
#Disables the display of setup information at power up
def disable_startup_msg(ser):
	inpt="NEX\xc6\x15" 
	ser.write(inpt)
	return True
	
#0x16 Enable startup
#message
#Enables the display of setup information at power up
def enable_startup_msg(ser):
	inpt="NEX\xc6\x16" 
	ser.write(inpt)
	return True
	
#x19 Change Address Change the I2C bus address of the LCD

	
'''#0x20-0xFF
#ASCII characters Writes ASCII characters straight to the display
def write_value(ser,ascii_value):
	temp="c602%s" %ascii_value
	inpt="NEX" + binascii.unhexlify(temp)
	inpt="NEX\xc6\x00" 
	ser.write(inpt)
	return True
'''
