
#ifndef JLUROBOVISION_PLATFORM_H
#define JLUROBOVISION_PLATFORM_H

// 这种PACK宏里面的结构体就不能用逗号了
#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

#endif //JLUROBOVISION_PLATFORM_H
