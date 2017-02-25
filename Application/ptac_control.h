#include <stdint.h>

extern void SetTemperature(uint8_t setTemperature);
extern void ForceFan(uint8_t forceFan);
extern void ForceCool(uint8_t forceCool);
extern void ForceHeat(uint8_t forceHeat);
extern void UpdatePTAC(uint8_t actualTemperature);
