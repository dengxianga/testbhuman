bhlowcmd = require 'bhlowcmd'
 
-- local tmp = bhlowcmd.getdummy()
-- print('tmp',tmp,'\n')

while 1 do
	local result = bhlowcmd.get_sensor_position(1);
	print('result', result, '\n');
end
