#include "myusrt.h"
#include "Drv_Uart.h"

void myusrt1send(void)
{
	 u8 Buf[] = {0x0A,0x0B,0X0C,0X0D,0x0A};
	 DrvUart1SendBuf(Buf,4);
}
