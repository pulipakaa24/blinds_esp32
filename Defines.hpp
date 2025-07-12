#define ccwSpeed 6500
#define cwSpeed 3300
#define offSpeed 4900

#define ccwMax 10
#define cwMax 0

#define getMovingCW(port) ((movingCW & (1 << port)) >> port)
#define setMovingCW(port) (movingCW |= (1 << port))
#define clearMovingCW(port) (movingCW &= ~(1 << port))
#define getMovingCCW(port) ((movingCCW & (1 << port)) >> port)
#define setMovingCCW(port) (movingCCW |= (1 << port))
#define clearMovingCCW(port) (movingCCW &= ~(1 << port))
#define getCalibCW(port) ((calibCW & (1 << port)) >> port)
#define setCalibCW(port) (calibCW |= (1 << port))
#define clearCalibCW(port) (calibCW &= ~(1 << port))
#define getCalibCCW(port) ((calibCCW & (1 << port)) >> port)
#define setCalibCCW(port) (calibCCW |= (1 << port))
#define clearCalibCCW(port) (calibCCW &= ~(1 << port))
#define getCalibDone(port) ((calibDone & (1 << port)) >> port)
#define setCalibDone(port) (calibDone |= (1 << port))
#define clearCalibDone(port) (calibDone &= ~(1 << port))
#define getBlocked(port) ((blocked & (1 << port)) >> port)
#define setBlocked(port) (blocked |= (1 << port))
#define clearBlocked(port) (blocked &= ~(1 << port))
#define getPos10(port) ((movingCCW & (1 << (port + 4))) >> (port + 4))
#define setPos10(port) (movingCCW |= (1 << (port + 4)))
#define clearPos10(port) (movingCCW &= ~(1 << (port + 4)))
#define getPos0(port) ((movingCW & (1 << (port + 4))) >> (port + 4))
#define setPos0(port) (movingCW |= (1 << (port + 4)))
#define clearPos0(port) (movingCW &= ~(1 << (port + 4)))

#define prefNameCalibs "periph_info_"
