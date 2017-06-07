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

Body.set_body_hardness(0);
while 1 do	
	local rarm_pos = Body.get_rarm_position();
	print(unpack(rarm_pos));
end