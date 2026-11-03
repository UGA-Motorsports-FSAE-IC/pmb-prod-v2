
//struct that bundles fdcan header and 8 bytes of data together

#ifndef __FDCANMESSAGE_H
#define __FDCANMESSAGE_H

#include "stm32c0xx_hal_fdcan.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
  FDCAN_RxHeaderTypeDef rxheader;
  uint8_t canrxdata[8];
} rxmessage;

typedef struct {
  uint32_t id;
  uint8_t cantxdata[8];    
} txmessage;

#ifdef __cplusplus
}
#endif
#endif /* __FDCANMESSAGE_H */