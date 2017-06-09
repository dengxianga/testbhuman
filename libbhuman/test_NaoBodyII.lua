-- For testing bhlowcmd and NaobodyII.lua

Body = require('Lib/NaoBodyII');

local clock = os.clock

function sleep(n)  -- seconds
  local t0 = clock()
  while clock() - t0 <= n do end
end

all_joints = {}    -- new array
for i=1, 22 do
  all_joints[i] = i;
end

-- Tests function where rarm joints are set, depending on start index and length of values
function set_actuator_hardnesses_test_1()
	print(unpack(all_joints));
	set_actuator_hardness(vector.ones(22)*0); 
end
-- Tests function where all joints are set, no index is given
function set_actuator_hardnesses_test_2()
	print(unpack(all_joints));
	set_actuator_hardness(vector.ones(22)*0); 
end

----------- Script -----------
-- while 1 do	
	-- local sensor_result = Body.get_sensor_button();
	-- print(sensor_result);
	-- sleep(1);
-- end

set_actuator_hardnesses_test();
print("done!");