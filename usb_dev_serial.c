#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "utils/ustdlib.h"
#include "usb_serial_structs.h"
#include "utils/uartstdio.h"

#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)
volatile uint32_t g_ui32SysTickCount = 0;

// Flags used to pass commands from interrupt context to the main loop.
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;

static volatile bool g_bUSBConfigured = false;

static bool SetLineCoding(tLineCoding *psLineCoding);
static void GetLineCoding(tLineCoding *psLineCoding);

static tLineCoding g_sLineCoding = { 115200, USB_CDC_STOP_BITS_1, USB_CDC_PARITY_NONE, 8 };

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
    }
}
#endif

void SysTickIntHandler(void) {
    g_ui32SysTickCount++;
}

static bool SetLineCoding(tLineCoding *psLineCoding) {
    memcpy(&g_sLineCoding, psLineCoding, sizeof(tLineCoding));
    return(true);
}

static void GetLineCoding(tLineCoding *psLineCoding) {
    memcpy(psLineCoding, &g_sLineCoding, sizeof(tLineCoding));
}

uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {

    uint32_t ui32IntsOff;

    switch(ui32Event) {

        // We are connected to a host and communication is now possible.
        case USB_EVENT_CONNECTED:

            g_bUSBConfigured = true;

            // Flush our buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            ui32IntsOff = ROM_IntMasterDisable();
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff) { ROM_IntMasterEnable(); }

            break;

        // The host has disconnected.
        case USB_EVENT_DISCONNECTED:

            g_bUSBConfigured = false;

            ui32IntsOff = ROM_IntMasterDisable();
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            if(!ui32IntsOff) { ROM_IntMasterEnable(); }
            break;

        // Return the current serial communication parameters.
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_LINE_CODING:
            SetLineCoding(pvMsgData);
            break;

        // Ignore
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        case USBD_CDC_EVENT_SEND_BREAK:
        case USBD_CDC_EVENT_CLEAR_BREAK:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}

uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {

    switch(ui32Event) {
        case USB_EVENT_TX_COMPLETE:
            // Since we are using the USBBuffer, we don't need to do anything here.
            break;
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {

    uint32_t ui32Count;

    uint32_t ui32Read;
    uint8_t ui8Char;

    switch(ui32Event) {

        case USB_EVENT_RX_AVAILABLE: {

            // Get a character from the receive buffer.
            ui32Read = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ui8Char, 1);
            if(ui32Read) {
                // Write character to the transmit buffer
                USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&ui8Char, 1);
            }

            break;
        }

        case USB_EVENT_DATA_REMAINING: {
            // TODO: AUDIT: does this really work?
            ui32Count = USBBufferDataAvailable(&g_sRxBuffer);
            return(ui32Count);
        }
        case USB_EVENT_REQUEST_BUFFER: {
            return(0);
        }
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}

int main(void) {

    // init tiva-c @ 80mhz
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // systick
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    // usb
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
    g_bUSBConfigured = false;

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    // Set the USB stack mode to Device mode with VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device on the bus.
    USBDCDCInit(0, &g_sCDCDevice);

    // Enable interrupts now that the application is ready to start.
    ROM_IntMasterEnable();

    while(1) {

        // Have we been asked to update the status display?
        if(g_ui32Flags & COMMAND_STATUS_UPDATE) {
            // Clear the command flag
            ROM_IntMasterDisable();
            g_ui32Flags &= ~COMMAND_STATUS_UPDATE;
            ROM_IntMasterEnable();
        }

    }
}
