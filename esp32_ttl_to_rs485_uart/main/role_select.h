#pragma once

#define ROLE_MASTER
//#define ROLE_RECEIVER

#ifdef ROLE_MASTER
#define IS_MASTER 1
#else
#define IS_MASTER 0
#endif
