#ifndef _USB_SERIAL_STRUCTS_H_
#define _USB_SERIAL_STRUCTS_H_

// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size of
// a maximum-sized USB packet.

#define UART_BUFFER_SIZE 256

extern uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t TxHandler(void *pvi32CBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);

extern tUSBBuffer g_sTxBuffer;
extern tUSBBuffer g_sRxBuffer;
extern tUSBDCDCDevice g_sCDCDevice;
extern uint8_t g_pui8USBTxBuffer[];
extern uint8_t g_pui8USBRxBuffer[];

#endif
