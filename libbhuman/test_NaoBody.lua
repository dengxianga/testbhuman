cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;

Body = require('NaoBody')
require('init')

Body.set_body_hardness(0.5);
