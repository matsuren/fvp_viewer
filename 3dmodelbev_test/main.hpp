#pragma once

#include <atomic>


// ----- global --------
// thread exit
extern std::atomic<bool> exit_flag;
extern bool checkExit();
extern void threadExit();

// render mode 1:floor 2:dome 3:LRF
extern int RENDER_MODE;
extern char **g_argv; 
extern int g_argc;