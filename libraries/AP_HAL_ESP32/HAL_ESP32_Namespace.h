#pragma once

namespace ESP32
{
class UARTDriver;
#ifdef USE_USB
class USBDriver;
#endif
class WiFiDriver;
class WiFiUdpDriver;
class Scheduler;
class EEPROMStorage;
class AnalogIn;
class RCInput;
class RCOutput;
class ADCSource;
class RCInput;
class Util;
class Semaphore;
class Semaphore_Recursive;
class GPIO;
class DigitalSource;
class Storage;
class RmtSigReader;
}  // namespace ESP32
