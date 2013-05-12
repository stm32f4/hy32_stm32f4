/******************************************************************************
 * @title   main.c
 * @author  Goofie31
 * @date    14 April 2013
 * @brief
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "main.h"
#include "psram.h"
#include "norflash.h"
#include "string.h"
#include "5110.h"
#include "serial.h"
#include "ascii.h"

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE        0x100
#define WRITE_READ_ADDR    0x0
#define MAX_RAM			   0x400000

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t TimingDelay;
static uint16_t blink;

static uint16_t TxBuffer[BUFFER_SIZE];
static uint16_t RxBuffer[BUFFER_SIZE];
static volatile uint32_t WriteReadStatus = 0;
static uint32_t Index = 0;

/* Private function prototypes -----------------------------------------------*/
static void Fill_Buffer(uint16_t *pBuffer, uint16_t BufferLenght,
		uint32_t Offset);
static uint32_t testNOR(uint32_t startAddress, uint32_t size,
		uint16_t bufferSize);
static uint32_t testSRAM(uint32_t startAddress, uint32_t size,
		uint16_t bufferSize);
static void ProcessSerialInput(char* buffer, uint16_t size);

/**
 * @brief  HY-STM32 board test
 * @param  None
 * @retval None
 */
int main(void) {

	/* Set Systick to 1 ms */
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1)
			;
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* GPIOF Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	/* Configure PF6 as output (state LED) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	LCD5110_init();
	LCD5110_Led(0);
	LCD5110_Set_XY(0, 0);
	LCD5110_Write_String("HY32 Test");

	initUART4(115200);
	SerialSetCallBack((callBack) ProcessSerialInput);
	SerialSendBytes("Hello\r\n", 7);

	WriteReadStatus = testNOR(0x10000, 0x100, 0x100);
	WriteReadStatus = testSRAM(0, 0x400000, 0x1000);

	if (WriteReadStatus == 0) {
		blink = 500; /* Ok */
	} else {
		blink = 100; /* Ko */
	}

	while (1) {
		Delay_ms(blink);
		GPIO_ToggleBits(GPIOF, GPIO_Pin_6);
		if (GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_6) == 0) {
			LCD5110_Set_XY(0, 5);
			LCD5110_Write_String("- - - - - - - ");
		} else {
			LCD5110_Set_XY(0, 5);
			LCD5110_Write_String(" - - - - - - -");
		}
	}
}

uint32_t testNOR(uint32_t startAddress, uint32_t size, uint16_t bufferSize) {
	uint8_t index;
	uint16_t numWord;
	uint16_t check;
	uint8_t startBlock = startAddress / NORFLASH_SECTOR_SIZE;
	uint8_t endBlock = (startAddress + size) / NORFLASH_SECTOR_SIZE;
	NORFLASH_Status status;
	NORFLASH_Id chipId;

	NORFLASH_Init();
//	for (index = startBlock; index <= endBlock; index++) {
//		status = NORFLASH_EraseBlockNum(index);
//	}
//	SerialSendBytes("Start writing.\r\n", 16);
//	for (numWord = 0; numWord < size; numWord++) {
//		NORFLASH_WriteWord(startAddress + numWord, numWord);
//	}
//	SerialSendBytes("Memory written.\r\n", 17);

	char testLine[24] = "Addr:xxxxxxxx :  yyyy \r\n";
	for (numWord = 0; numWord < size; numWord++) {
		check = NORFLASH_ReadWord(startAddress + numWord);
		getHexFromLong(testLine + 5, startAddress + numWord, 8);
		getHexFromLong(testLine + 17, check, 4);
		Delay_ms(1);
		SerialSendBytes(testLine, 24);
	}

	return 0;
}

uint32_t testSRAM(uint32_t startAddress, uint32_t size, uint16_t bufferSize) {
	uint16_t check;
	uint32_t loop;
	char AddressLine[24] = "Addr:xxxxxxxx : yyyyy \r\n";

	PSRAM_Init();

	LCD5110_Set_XY(0, 2);
	LCD5110_Write_String("Addr : ");
	for (loop = startAddress; loop < startAddress + size; loop++) {
		PSRAM_WriteWord((uint16_t) loop, loop);
		check = PSRAM_ReadWord(loop);

		if ((loop & 0xFFF) == 0x0) {
			LCD5110_Set_XY(7, 2);
			LCD5110_Write_Dec32(loop, 7);

			getHexFromLong(AddressLine + 5, loop, 8);
			getDecimalFromShort(AddressLine + 16, check);
			SerialSendBytes(AddressLine, 24);
		}
		if ((uint16_t) loop != check) {
			WriteReadStatus++;
			break;
		}
	}
	if (WriteReadStatus == 0) {
		LCD5110_Set_XY(0, 2);
		LCD5110_Write_String("RAM Ok.       ");
		LCD5110_Set_XY(0, 3);
		LCD5110_Write_String("Size :");
		LCD5110_Set_XY(7, 3);
		LCD5110_Write_Dec(size >> 9, 4);
		LCD5110_Set_XY(11, 3);
		LCD5110_Write_String(" Ko");
	} else {
		LCD5110_Set_XY(0, 2);
		LCD5110_Write_String("Ram failed. ");
		LCD5110_Set_XY(0, 3);
		LCD5110_Write_String("Addr : ");
		LCD5110_Set_XY(7, 3);
		LCD5110_Write_Dec32(loop, 7);
	}

	return WriteReadStatus;
}

/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds.
 * @retval None
 */
void Delay_ms(volatile uint32_t ntime) {
	TimingDelay = ntime;
	while (TimingDelay != 0)
		;
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void) {
	if (TimingDelay > 0x00) {
		TimingDelay--;
	}
}

void ProcessSerialInput(char* buffer, uint16_t size) {
	if (buffer[size - 1] == 0xA) {
		buffer[size - 2] = 0;
		LCD5110_Clear();
		LCD5110_Set_XY(0, 0);
		LCD5110_Write_String(buffer);
		SerialClearInputBuffer();
	}
}

/**
 * Route IRQ to the implemented handler
 *
 * @param irq IRQ type
 */
void DispatchIRQ(IRQ_PPP_Type irq) {
	switch (irq) {
	case UART4_IRQ:
		UART4_IRQHandler_impl();
		break;
	default:
		break;
	}
}
