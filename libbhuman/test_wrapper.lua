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
local write = io.write;
local qs={0.1, 0};
local stfs={0.5, 0.5}
local ids={1, 2};

local cnt=1;

local speed = 0.00333333;

function printAllSensors() 
  local sensor_positions = bhlowcmd.get_sensor_positions();
  print('Actuator sensors: ');
  for i = 1, #sensor_positions do
    print(i, ": ", sensor_positions[i]);
  end
end

function printImuAngle()
  local imu_angle = bhlowcmd.get_imu_angle();
  print('Angle: IMU-X: ', imu_angle[1], ' IMU-y: ', imu_angle[2]);
end

function printImuAcc() 
  local imu_acc = bhlowcmd.get_imu_acc();
  print('Acceleration: IMU-X: ', imu_acc[1], ' IMU-y: ', imu_acc[2], ' IMU-z: ', imu_acc[3]);
end

function printImuGyro()
  local imu_gyro = bhlowcmd.get_imu_gyr();
  print('Gyro: IMU-X: ', imu_gyro[1], ' IMU-y: ', imu_gyro[2], ' IMU-z: ', imu_gyro[3]);
end

function printSensorPosition()
  local RShoulderPitchSensor = bhlowcmd.get_sensor_position(9);
  print(RShoulderPitchSensor);
end

function printActuatorCommand()
  local actuator_command = bhlowcmd.get_actuator_command(1);
  print(actuator_command);
end

function setActuatorCommand()
  local actuator_pos_to_set = {1, -0.5};
  bhlowcmd.set_actuator_command(actuator_pos_to_set, 1);
end

function printSensorCurrent()
  print('current: ', bhlowcmd.get_sensor_current(1));
end

function printAllStiffnesses()
  print('Stiffnesses: ');
  local stiffnesses = bhlowcmd.get_actuator_hardnesses();
  for i = 1, #stiffnesses do
    print(stiffnesses[i]);
  end
end

function printTime()
  print('time: ', unix.time());
end

function printActuatorPosition()
  local result = bhlowcmd.get_actuator_position(ids[1]);
  print('actual ', result, 'expected ', qs[1]);
end

------- Script ---------
bhlowcmd.set_actuator_hardnesses(stfs,ids)

while 1 do
  ------ Movement ------
	qs[1]= math.cos(speed*cnt);
  bhlowcmd.set_actuator_positions(qs,ids);
  ----------------------

  ----- Print Inside ----
  if (cnt % 100 == 0) then
    printAllSensors();
  end  
  -----------------------

	local tDelay = 0.0025 * 1E6; -- Loop every 2.5ms
	unix.usleep(tDelay);
  cnt=cnt+1;
end




-- bhlowcmd.set_actuator_position_forever(ids, ids);
