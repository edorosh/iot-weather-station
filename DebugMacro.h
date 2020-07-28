/*
  DebugMacro.h - Defines macros for debugging via Serial puproses. Macros depends on another macro DEBUG.
*/

#ifndef DebugMacro_h
#define DebugMacro_h

#ifdef DEBUG
#define DPRINT(...)    Serial.print(__VA_ARGS__)
#define DPRINTFF(...)  Serial.printf(F(__VA_ARGS__))
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#define DPRINTF(...)   Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...) Serial.println(F(__VA_ARGS__))

#else
#define DPRINT(...)     //blank line
#define DPRINTFF(...)   //blank line
#define DPRINTLN(...)   //blank line
#define DPRINTF(...)    //blank line
#define DPRINTLNF(...)  //blank line

#endif

#endif // end of DebugMacro_h