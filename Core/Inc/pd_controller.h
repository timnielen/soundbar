#ifndef __PD_CONTROLLER_H
#define __PD_CONTROLLER_H

#define PD_CONTROLLER_ADDR (0x22 << 1) // CH224A
#define PD_CONTROLLER_STATUS_ADDR 0x09
#define PD_CONTROLLER_VOLTAGE_ADDR 0x0A


uint8_t pdControllerStatus = 0;
static volatile uint32_t swLeftLastEdge = 0;
static volatile uint8_t  swLeftPending  = 0;


#endif