-- equivalent to keyframe.lua

require('init')
Body = require('Lib/NaoBody')

while 1 do
	local sensorVal;
	sensorVal = Body.get_sensor_position(1);
	print('1:', sensorVal);
	unix.usleep(0.0025 * 1E8);
end
