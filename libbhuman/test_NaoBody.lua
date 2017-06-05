cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;

require('Body')
