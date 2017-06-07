cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;

bhlowcmd = require 'bhlowcmd'
require('init')

function set_actuator_hardnesses(values, indices)
	bhlowcmd.set_actuator_hardnesses(values, indicies); 
end

function set_actuator_positions(values, indices)
	bhlowcmd.set_actuator_positions(values, indicies); 
end

function set_actuator_position(value, index)
	bhlowcmd.set_actuator_position(value, index); 
end

function set_actuator_hardness(values, index)
	if type(values) == "number" then
		bhlowcmd.set_actuator_hardness(values, index);
	else
		index = index or 0;
		bhlowcmd.set_actuator_hardnesses(values, index); 
	end
end

function get_actuator_position()
	local result = bhlowcmd.get_actuator_position(); 
	return result;
end

function get_actuator_hardness()
	local result = bhlowcmd.get_actuator_hardness(); 
	return result;
end

function set_actuator_command(values, starting_index)
	if type(values) == "number" then
		bhlowcmd.set_actuator_position(values, starting_index);
	else
		starting_index = starting_index or 0;
		bhlowcmd.set_actuator_command(values, starting_index); 
	end
end

function get_actuator_command(starting_index)
	bhlowcmd.get_actuator_command(starting_index); 
end

function get_sensor_position(index)
	local result;
	if index then
		result = bhlowcmd.get_sensor_position(index);
	else
		result = bhlowcmd.get_sensor_positions(); 
	end
	return result;
end

function get_sensor_positions()
	local result = bhlowcmd.get_sensor_positions(); 
	return result;
end

function get_actuator_hardnesses()
	local result = bhlowcmd.get_actuator_hardnesses(); 
	return result;
end

function get_imu_angle()
	local result = bhlowcmd.get_imu_angle(); 
	return result;
end

function get_imu_acc()
	local result = bhlowcmd.get_imu_acc(); 
	return result;
end

function get_imy_gyr()
	local result = bhlowcmd.get_imu_gyr(); 
	return result;
end

function set_actuator_velocity(value, index) 
	bhlowcmd.set_actuator_velocity(value, index);
end

function get_actuator_velocity(index)
	local result = bhlowcmd.get_actuator_velocity(index);
	return result;
end


