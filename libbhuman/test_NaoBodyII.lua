-- For testing bhlowcmd and NaobodyII.lua
cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require("init")
require("Config")
require("Body")

local indexHead = 1;
local nJointHead = 2;
local indexLArm = 3;
local nJointLArm = 4;
local indexLLeg = 7;
local nJointLLeg = 6;
local indexRLeg = 13;
local nJointRLeg = 6;
local indexRArm = 19;
local nJointRArm = 4;

local clock = os.clock

function sleep(n)  -- seconds
  local t0 = clock()
  while clock() - t0 <= n do end
end

all_joints = {}    -- new array
for i=1, 22 do
  all_joints[i] = i;
end

-- Tests function where all joints are set, no index is given
function set_actuator_hardnesses_test_1()
	print(unpack(all_joints));
	-- set_actuator_hardness(vector.ones(22)*0.75); 
	set_actuator_hardness({0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0});
end

local function rotate_joint()
	local hardnesses = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0};
	hardnesses[1] = 0.5;
	-- hardnesses[3] = 0.5;
	-- hardnesses[19] = 0.5;
	local positions = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0};
	Body.set_actuator_hardness(hardnesses);
	local speed = 0.005*0.1;
	local count = 0;
	while (1) do
		if (count % 20 == 0)  then 
			new_pos = math.cos(speed * count);
			Body.set_actuator_position(new_pos, 1);
			sleep(0.005);
		end
		count = count + 1;
	end
end

-- -- Tests function where all joints are set, no index is given
-- function set_actuator_hardnesses_test_2()
-- 	print(unpack(all_joints));
-- 	set_actuator_hardness(vector.ones(22)*0); 
-- end

function set_head_command_movement_test()
	head_target = Body.get_head_position();
	head_command = Body.get_head_position();
	Body.set_head_hardness(0.7);
	
	Body.update_head_movement();
end

function set_head_hardness_test()
	Body.set_actuator_hardness(vector.zeros(22));
	Body.set_head_hardness(0.5);
end

function set_larm_hardness_test()
	Body.set_actuator_hardness(vector.zeros(22));
	Body.set_larm_hardness(0.5);
end

function set_rarm_hardness_test()
	Body.set_actuator_hardness(vector.zeros(22));
	Body.set_rarm_hardness(0.5);
end

function set_lleg_hardness_test()
	Body.set_actuator_hardness(vector.zeros(22));
	Body.set_lleg_hardness(0.5);
end

function set_rleg_hardness_test()
	Body.set_actuator_hardness(vector.zeros(22));
	Body.set_rleg_hardness(0.5);
end

function set_larm_command_test()
	set_larm_hardness_test();
	Body.set_larm_command({0.5, 0.5, -1, -0.5});
end

function set_rarm_command_test()
	set_rarm_hardness_test();
	Body.set_rarm_command({0.5, -0.5, 1, 0.5});
end

function set_lleg_command_test()
	set_lleg_hardness_test();
	Body.set_rleg_hardness(0.5);
	Body.set_lleg_command({0.3, 0.5, -1, 1, 0.5, 0.3, 0.3, -0.5, -1, 1, 0.5, -0.3});
end

function set_rleg_command_test()
	set_rleg_hardness_test();
	Body.set_rleg_command({0.3, -0.5, -1, 1, -0.5, -0.3});
end

function calibrate_test()
	for i = 1, 102 do
		print(Body.ccount)
		Body.calibrate(-1);
	end
end

function get_sensor_imuGyrRPY_test()
	print(unpack(Body.get_sensor_imuGyrRPY()));
end

function get_battery_level_test()
	print(Body.get_battery_level());
end

function get_sensor_bumperLeft_test()
	print(unpack(Body.get_sensor_bumperLeft()));
end

function get_sensor_bumperRight_test()
	print(unpack(Body.get_sensor_bumperRight()))
end

function get_sensor_button_test()
	print(Body.get_sensor_button())
end

function get_sensor_usLeft_test()
	print(unpack(Body.get_sensor_usLeft()));
end

function get_sensor_usRight_test()
	print(unpack(Body.get_sensor_usRight()));
end

function get_sensor_temperature_test()
	local sensorInfo = Body.get_sensor_temperature();
	for i = 1, 22 do
		print(i, sensorInfo[i]);
	end
end

function stand_test()
	local count = 0;
  set_actuator_hardnesses_test_1();
  Body.set_rarm_command({1.6, 0, 0, 0});
  Body.set_larm_command({1.6, 0, 0, 0});
end

function get_sensor_fsrLeft_test() 
	local sensorInfo = Body.get_sensor_fsrLeft();
	print(unpack(sensorInfo));
end 

function get_sensor_fsrRight_test() 
	local sensorInfo = Body.get_sensor_fsrRight();
	print(unpack(sensorInfo));
end 

function get_time_test()
  local time = Body.get_time();
  print(time);
end

function set_actuator_ledFoot_test()
	local blue = {0, 0, 1};
	local green = {0, 1, 0};
	local red = {1, 0, 0};
	local white = {1, 1, 1};
	local off = {0, 0, 0};
	local yellow = {1, 1, 0};
	local cyan = {0, 1, 1};
	local purple = {1, 0, 1};
	Body.set_actuator_ledFootLeft(off);
	-- set_actuator_ledFootRight(purple);
end

function set_actuator_ledEars_test(charge)
	local command = vector.ones(10);
	local i = 0;
	while (i < 10-charge) do
		command[i] = 0;
		i = i + 1;
	end
	Body.set_actuator_ledEarsRight(command);
end

function set_actuator_ledEars_forever_test()
	local count = 0;
	while 1 do
		charge = count % 10;
		Body.set_actuator_ledEars_test(charge);
		count = count + 1;
		sleep(1);
	end
end

function set_actuator_ledFace_test() 
	local white = vector.ones(24);
	local one_color = vector.ones(8);
	Body.set_actuator_ledFaceLeft(one_color, 7);
	Body.set_actuator_ledFaceRight(one_color, 17);
end

----------- Script -----------

--uncomment below to relax joints in case it's hitting itself
-- Body.set_actuator_hardness(vector.zeros(22));
--set_actuator_hardnesses_test_1()
-- local sensorinfo = Body.get_sensor_temperature();
-- print(unpack(Body.get_sensor_temperature()))
while 1 do	
  
	--print(Body.get_sensor_ultraSonic());
	print(unpack(Body.get_sensor_position()));
	sleep(1);
end

print("done!");
