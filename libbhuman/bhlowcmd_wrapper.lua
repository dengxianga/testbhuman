cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;

bhlowcmd = require 'bhlowcmd'
require('init')

-- function set_actuator_hardnesses(values, indices)
-- 	bhlowcmd.set_actuator_hardnesses(values, indices); 
-- end

---
-- @param values - vector of actuator position values
-- @param indicies - vector detailing which indices to set 
-- 					 correspondng to values vector
---
function set_actuator_positions(values, indices)
	bhlowcmd.set_actuator_positions(values, indicies); 
end

---
-- @param value - single position 
-- @param indicies - single index indicating which actuator to set
---
function set_actuator_position(value, index)
	bhlowcmd.set_actuator_position(value, index); 
end

---
-- Case 1: set single joint actuator
-- @param values - single hardness 
-- @param index - single index indicating which actuator to set
--				  the hardness of
-- 
-- Case 2: set a group of limb actuators
-- @param values - table of hardnesses
-- @param index - single index indicating initial actuator 
--				  to start setting hardnesses
-- 
-- Case 3: set all joint actuators
-- @param values - table of 22 hardnesses
-- @param index - nil
---
function set_actuator_hardness(values, index)
	if type(values) == "number" then
		bhlowcmd.set_actuator_hardness(values, index);
	else
		index = index or 1;
		local indices = vector.zeros(#values);
		for i = 1, #values do
			indices[i] = index + i - 1;
		end
		bhlowcmd.set_actuator_hardnesses(values, indices);
	end
end
 
--
-- @return - expected actuator position of all joints
---
function get_actuator_position()
	local result = bhlowcmd.get_actuator_position(); 
	return result;
end

---
-- @return - expected actuator hardness of all joints
---
function get_actuator_hardness()
	local result = bhlowcmd.get_actuator_hardness(); 
	return result;
end

---
-- Case 1: set single joint actuator
-- @param values - single position 
-- @param index - single index indicating which actuator to set
-- 
-- Case 2: set a group of limb actuators
-- @param values - table of positions
-- @param index - single index indicating initial actuator 
--				  to start setting positions
-- 
-- Case 3: set all joint actuators
-- @param values - table of 22 positions
-- @param index - nil
---
function set_actuator_command(values, starting_index)
	if type(values) == "number" then
		bhlowcmd.set_actuator_position(values, starting_index);
	else
		starting_index = starting_index or 1;
		bhlowcmd.set_actuator_command(values, starting_index); 
	end
end

---
-- @param starting_index - initial index of head or a limb
-- @return - all expected position values of the limb (variable joints)
---
function get_actuator_command(starting_index)
	bhlowcmd.get_actuator_command(starting_index); 
end

---
-- Case 1: get single joint actuator position
-- @param index - joint index 
-- @return - actual position value of requested joint
-- 
-- Case 2: 
-- @param index - nil
-- @return - all actual joint actuator positions  (22 joints)
---
function get_sensor_position(index)
	local result;
	if index then
		result = bhlowcmd.get_sensor_position(index);
	else
		result = bhlowcmd.get_sensor_positions(); 
	end
	return result;
end

---
-- @return - all actual actuator positions (22 joints)
---
function get_sensor_positions()
	local result = bhlowcmd.get_sensor_positions(); 
	return result;
end

---
-- @return - all expected actuator hardnesses (22 joints)
---
function get_actuator_hardnesses()
	local result = bhlowcmd.get_actuator_hardnesses(); 
	return result;
end

---
-- @return - X, Y, Z angle IMU measurements
---
function get_sensor_imuAngle(index)
	local result = bhlowcmd.get_imu_angle(); 
	if (index) then
		return result[index];
	end
	return result;
end

---
-- @return - X, Y, Z accelerometer IMU measurements
---
function get_sensor_imuAcc(index)
	local result = bhlowcmd.get_imu_acc(); 
	if (index) then
		return result[index];
	end
	return result;
end

---
-- @return - X, Y, Z gyroscope IMU measurements
---
function get_sensor_imuGyr(index)
	local result = bhlowcmd.get_imu_gyr(); 
	if (index) then
		return result[index];
	end
	return result;
end

---
-- DATA STRUCTURE DOES NOT SUPPORT VELOCITY
---
function set_actuator_velocity(value, index) 
	bhlowcmd.set_actuator_velocity(value, index);
end

---
-- DATA STRUCTURE DOES NOT SUPPORT VELOCITY
---
function get_actuator_velocity(index)
	local result = bhlowcmd.get_actuator_velocity(index);
	return result;
end

---
-- @return - sensor reading of battery charge
---
function get_sensor_batteryCharge()
	local result = bhlowcmd.get_sensor_batteryCharge();
	return result;
end

---
-- @return - sensor reading of whether chest button is pressed
---
function get_sensor_button()
	local result = bhlowcmd.get_sensor_button();
	return result;
end

---
-- @return - binary indication of whether each left, right side
--			 of left foot bumper are pressed
---
function get_sensor_bumperLeft()
	local result = bhlowcmd.get_sensor_bumperLeft();
	return result;
end

---
-- @return - binary indication of whether each left, right side
--			 of right foot bumper are pressed
---
function get_sensor_bumperRight()
	local result = bhlowcmd.get_sensor_bumperRight();
	return result;
end

---
-- @return - distance in meters of the first ten US echoes 
-- 			 to the left US sensor
---
function get_sensor_usLeft()
	local result = bhlowcmd.get_sensor_sonarLeft();
	return result;
end

---
-- @return - distance in meters of the first ten US echoes 
-- 			 to the right US sensor
---
function get_sensor_usRight()
	local result = bhlowcmd.get_sensor_sonarRight();
	return result;
end

---
-- @return - dcm time
---
function get_sensor_time(index)
	local result = bhlowcmd.get_time();
	return result;
end

---
-- @return - current readings of all 22 joints
---
function get_sensor_current()
	local result = bhlowcmd.get_sensor_current();
	return result;
end


---
-- @return - temperature readings of all 22 joints
---
function get_sensor_temperature()
	local result = bhlowcmd.get_sensor_temperature();
	return result;
end

---
-- @param - US functionality setting, generally use 68
---
function set_actuator_us(command)
	bhlowcmd.set_actuator_ultraSonic(command);
end

---
-- @return - left foot sensor readings in order: left front, 
--			 right front, left rear, right rear
---
function get_sensor_fsrLeft() 
	local result = bhlowcmd.get_sensor_fsrLeft();
	return result;
end

---
-- @return - right foot sensor readings in order: left front, 
--			 right front, left rear, right rear
---
function get_sensor_fsrRight()
	local result = bhlowcmd.get_sensor_fsrRight();
	return result;
end

-- ---
-- -- @return - US reading if using functionality setting 0-3
-- -- (doesn't currently work)
-- ---
-- function get_sensor_ultraSonic()
-- 	local result = bhlowcmd.get_sensor_ultraSonic();
-- 	return result;
-- end

function set_actuator_ledFootLeft(values)
	bhlowcmd.set_actuator_ledFootLeft(values);
end

function set_actuator_ledFootRight(values)
	bhlowcmd.set_actuator_ledFootRight(values);
end

function set_actuator_ledEarsLeft(values)
	bhlowcmd.set_actuator_ledEarsLeft(values);
end

function set_actuator_ledEarsRight(values)
	bhlowcmd.set_actuator_ledEarsRight(values);
end

function set_actuator_ledFaceLeft(values, index)
	if (index) then
		bhlowcmd.set_actuator_ledFaceLeft(values, index);
	else
		bhlowcmd.set_actuator_ledFaceLeft(values, 1);
	end
end

function set_actuator_ledFaceRight(values, index)
	if (index) then
		bhlowcmd.set_actuator_ledFaceRight(values, index);
	else
		bhlowcmd.set_actuator_ledFaceRight(values, 1);
	end
end

function set_actuator_ledChest(values)
	bhlowcmd.set_actuator_ledChest(values);
end

function set_actuator_ledHead(values)
	bhlowcmd.set_actuator_ledHead(values);
end
