cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;

bhlowcmd = require 'bhlowcmd'
require('init')
-- require('Config');
-- require('Body')
-- require('vector')
-- require('getch')
-- require('walk');
-- require('util')

-- local tmp = bhlowcmd.getdummy()
-- print('tmp',tmp,'\n')

local qs={0.1, 0};
local stfs={0.5, 0.5}
local ids={1, 2};

local cnt=1;

local temp=0;
local speed = 0.01;

bhlowcmd.set_actuator_stiffneses(stfs,ids)
-- bhlowcmd.set_speed(speed);

-- while 1 do
-- 	-- while (bhlowcmd.get_flag()) do

-- 	-- end
-- 	local result = bhlowcmd.get_actuator_position(ids[1]);
-- 	qs[1]= math.cos(speed*cnt);
--   bhlowcmd.set_actuator_positions(qs,ids)

-- 	print('actual ', result, 'expected ', qs[1]);
-- 	cnt=cnt+1;

-- 	print('time: ', unix.time());
-- 	print('current: ', bhlowcmd.get_sensor_current(2));
-- 	-- for i = 1, 1000000 do
-- 	-- 	temp = temp + 1;
-- 	-- end



-- 	local tDelay = 0.0025 * 1E6; -- Loop every 2.5ms
-- 	unix.usleep(tDelay);
-- end

-- print('tmp',tmp,'\n')

bhlowcmd.set_actuator_position_forever(ids, ids);
