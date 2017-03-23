
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <../Application/ptac_control_app.h>
#include "board.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

extern void SetTemperature(uint8_t setTemperature);
extern void ForceFan(uint8_t forceFan);
extern void ForceCool(uint8_t forceCool);
extern void ForceHeat(uint8_t forceHeat);
extern void UpdatePTAC(uint8_t actualTemperature);
extern PIN_State hstate_pass(PIN_State passhandle);
