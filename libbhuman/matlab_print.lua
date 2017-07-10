cwd = os.getenv('PWD');
require('init');
require('unix');
require('Body');

local clock = os.clock

function sleep(n)  -- seconds
  local t0 = clock()
  while clock() - t0 <= n do end
end

while 1 do 
	print(Body.get_time(), unpack(Body.get_sensor_temperature()));
	sleep(1);

end