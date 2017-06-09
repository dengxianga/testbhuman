-- For testing original luadcm and Naobody.lua
cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require('init')
require('Config');
require('shm')
require('Body')
require('vector')
require('getch')
require('Motion');
require('walk');
require('dive');
require('Speak')
require('util')


local clock = os.clock

function sleep(n)  -- seconds
  local t0 = clock()
  while clock() - t0 <= n do end
end

-- while 1 do	
-- 	local sensor_gyro = Body.get_sensor_imuAngle();
-- 	print(unpack(sensor_gyro));
-- 	sleep(1);
-- end

local sensor_result = Body.get_sensor_button();
print(unpack(sensor_result));