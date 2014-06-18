#for Testing purpose
import connection
ser = connection.connect()
print "-- Acceptes only two argument commend --"
try:
	while True:
		output =[]
		print "NEX "
		input_var = raw_input("Enter Command : ")
		input_var = input_var.split()
		inpt= "NEX" + ('\\x' + input_var[0]).decode('string_escape') +('\\x' + input_var[1]) .decode('string_escape')
		print inpt
		output = ser.write(inpt) 
		print output
except:
	connection.disconnect()
