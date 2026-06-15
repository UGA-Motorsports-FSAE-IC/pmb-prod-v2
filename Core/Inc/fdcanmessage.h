
//struct that bundles fdcan header and 8 bytes of data together

#ifndef __FDCANMESSAGE_H
#define __FDCANMESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
  FDCAN_RxHeaderTypeDef rxheader;
  uint8_t canrxdata[8];
} rxmessage;


#ifdef __cplusplus
}
#endif
#endif /* __FDCANMESSAGE_H */