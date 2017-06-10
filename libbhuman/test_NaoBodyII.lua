-- For testing bhlowcmd and NaobodyII.lua

Body = require('Lib/NaoBodyII');

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
	set_actuator_hardness(vector.ones(22)); 
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
	-- local sensorInfo = Body.get_sensor_usLeft();
	-- print(sensorInfo[1]);
end

function get_sensor_temperature_test()
	local sensorInfo = Body.get_sensor_temperature();
	for i = 1, 22 do
		print(i, sensorInfo[i]);
	end
end

----------- Script -----------

--uncomment below to relax joints in case it's hitting itself
Body.set_actuator_hardness(vector.zeros(22));

print('starting important stuff')
local count = 0;
set_actuator_hardnesses_test_1();
Body.set_rarm_command({1.6, 0, 0, 0});
Body.set_larm_command({1.6, 0, 0, 0});

while 1 do
	-- if (count % 2 == 0) then
	-- 	Body.set_rleg_command({0.3, -0.5, -1, 1, -0.5, -0.3});
	-- else
	-- 	Body.set_rleg_command({0, 0, 0, 0, 0, 0});
	-- end

	get_sensor_temperature_test();
	sleep(1);
end

print("done!");
