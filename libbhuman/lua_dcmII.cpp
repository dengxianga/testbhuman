#include <iostream>
#include <lua.hpp>

#include "RoboCupGameControlData.h"
#include "bhuman.h"
using namespace std;

static int luaBH_getdummy (lua_State *L) {
  lua_pushnumber(L, 0);
  std::cout<<"here \n";
  // TODO try to manipulate the shared memory "data"
  return 1;
}

static const struct luaL_Reg bhlowcmd_lib [] = {
  {"getdummy", luaBH_getdummy},
  {NULL, NULL}
};


extern "C" int luaopen_bhlowcmd(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, bhlowcmd_lib);
#else
  luaL_register(L, "bhlowcmd", bhlowcmd_lib);
#endif
  return 1;
}
