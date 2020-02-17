#ifndef __KEY_H__
#define __KEY_H__

#define QuadKey1  GPIOPinRead(GPIO_PORTE_BASE , GPIO_PIN_2)
#define QuadKey2  GPIOPinRead(GPIO_PORTF_BASE , GPIO_PIN_4)

void Key_Init(void);
bool Key_Scan(uint8_t release);


#endif


