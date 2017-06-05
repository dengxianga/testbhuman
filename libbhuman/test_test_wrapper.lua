-- equivalent to keyframe.lua

test = require('test_wrapper')

while 1 do
	local sensorVal;
	sensorVal = bhlowcmd.get_sensor_position(1);
	print('1:', sensorVal);
end
