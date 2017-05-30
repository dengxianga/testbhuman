bhlowcmd = require 'bhlowcmd'

-- local tmp = bhlowcmd.getdummy()
-- print('tmp',tmp,'\n')

local qs={0.1};
local stfs={0.5}
local ids={1};

local cnt=1;



bhlowcmd.set_actuator_stiffneses(qs,ids)

while 1 do
  local result = bhlowcmd.get_sensor_position(ids[1]);
  qs[1]= math.cos(0.0001*cnt);
  bhlowcmd.set_actuator_positions(qs,ids)

  print('here',result )
  cnt=cnt+1;
end

print('tmp',tmp,'\n')
