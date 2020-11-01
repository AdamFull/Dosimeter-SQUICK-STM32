
#include "libs/w25qxx.h"
#include "libs/w25qxxConf.h"
#include "util.h"

#if (_W25QXX_DEBUG==1)
#include <stdio.h>
#endif

#define W25QXX_DUMMY_BYTE         0xA5

w25qxx_t	w25qxx;

void spi_receive(uint8_t *buffer, uint32_t size){
	uint8_t ticker = 0;
	for(uint32_t i = 0; i < size; i++){
		//while(LL_SPI_IsActiveFlag_RXNE(_W25QXX_SPI)){ if(ticker > 1000) break; ticker++; }
		buffer[i] = LL_SPI_ReceiveData8(_W25QXX_SPI);
	}

}

void spi_transmit(uint8_t* buffer, uint32_t size){
	uint8_t ticker = 0;
	for(uint32_t i = 0; i < size; i++){
		while(!LL_SPI_IsActiveFlag_TXE(_W25QXX_SPI)){ if(ticker > 1000) break; ticker++; }
		LL_SPI_TransmitData8(_W25QXX_SPI, buffer[i]);
	}

}

#if (_W25QXX_USE_FREERTOS==1)
#define	W25qxx_Delay(delay)		osDelay(delay)
#include "cmsis_os.h"
#else
#define	W25qxx_Delay(delay)		LL_mDelay(delay);//for(uint32_t i = 0; i < delay; i++) asm("nop");
#endif
//###################################################################################################################
uint8_t	W25qxx_Spi(uint8_t	Data){
	uint8_t	ret;
	spi_transmit(&Data, 1);
	spi_receive(&ret, 1);
	return ret;
}
//###################################################################################################################
uint32_t W25qxx_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0x9F); //0x9F
  Temp0 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
  Temp1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
  Temp2 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
  LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return Temp;
}
//###################################################################################################################
void W25qxx_ReadUniqID(void)
{
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Spi(0x4B);
	for(uint8_t	i=0;i<4;i++) W25qxx_Spi(W25QXX_DUMMY_BYTE);
	for(uint8_t	i=0;i<8;i++) w25qxx.UniqID[i] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
}
//###################################################################################################################
void W25qxx_WriteEnable(void)
{
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Spi(0x06);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Delay(1);
}
//###################################################################################################################
void W25qxx_WriteDisable(void)
{
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0x04);
  LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Delay(1);
}
//###################################################################################################################
uint8_t W25qxx_ReadStatusRegister(uint8_t	SelectStatusRegister_1_2_3)
{
	uint8_t	status=0;
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	if(SelectStatusRegister_1_2_3==1)
	{
		W25qxx_Spi(0x05);
		status=W25qxx_Spi(W25QXX_DUMMY_BYTE);	
		w25qxx.StatusRegister1 = status;
	}
	else if(SelectStatusRegister_1_2_3==2)
	{
		W25qxx_Spi(0x35);
		status=W25qxx_Spi(W25QXX_DUMMY_BYTE);	
		w25qxx.StatusRegister2 = status;
	}
	else
	{
		W25qxx_Spi(0x15);
		status=W25qxx_Spi(W25QXX_DUMMY_BYTE);	
		w25qxx.StatusRegister3 = status;
	}	
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	return status;
}
//###################################################################################################################
void W25qxx_WriteStatusRegister(uint8_t	SelectStatusRegister_1_2_3,uint8_t Data)
{
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	if(SelectStatusRegister_1_2_3==1)
	{
		W25qxx_Spi(0x01);
		w25qxx.StatusRegister1 = Data;
	}
	else if(SelectStatusRegister_1_2_3==2)
	{
		W25qxx_Spi(0x31);
		w25qxx.StatusRegister2 = Data;
	}
	else
	{
		W25qxx_Spi(0x11);
		w25qxx.StatusRegister3 = Data;
	}
	W25qxx_Spi(Data);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
}
//###################################################################################################################
void W25qxx_WaitForWriteEnd(void)
{
	W25qxx_Delay(1);
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Spi(0x05);
  do{
    w25qxx.StatusRegister1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		W25qxx_Delay(1);
  }
  while ((w25qxx.StatusRegister1 & 0x01) == 0x01);
  LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
}
//###################################################################################################################
bool	W25qxx_Init(void)
{
	w25qxx.Lock=1;	
	while(GetTick()<100) W25qxx_Delay(1);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Delay(100);
	uint32_t	id;
	id=W25qxx_ReadID();

	switch(id&0x0000FFFF)
	{
		case 0x401A:	// 	w25q512
			w25qxx.ID=W25Q512;
			w25qxx.BlockCount=1024;
		break;
		case 0x4019:	// 	w25q256
			w25qxx.ID=W25Q256;
			w25qxx.BlockCount=512;
		break;
		case 0x4018:	// 	w25q128
			w25qxx.ID=W25Q128;
			w25qxx.BlockCount=256;
		break;
		case 0x4017:	//	w25q64
			w25qxx.ID=W25Q64;
			w25qxx.BlockCount=128;
		break;
		case 0x4016:	//	w25q32
			w25qxx.ID=W25Q32;
			w25qxx.BlockCount=64;
		break;
		case 0x4015:	//	w25q16
			w25qxx.ID=W25Q16;
			w25qxx.BlockCount=32;
		break;
		case 0x4014:	//	w25q80
			w25qxx.ID=W25Q80;
			w25qxx.BlockCount=16;
		break;
		case 0x4013:	//	w25q40
			w25qxx.ID=W25Q40;
			w25qxx.BlockCount=8;
		break;
		case 0x4012:	//	w25q20
			w25qxx.ID=W25Q20;
			w25qxx.BlockCount=4;
		break;
		case 0x4011:	//	w25q10
			w25qxx.ID=W25Q10;
			w25qxx.BlockCount=2;
		break;
		default:
			w25qxx.Lock=0;	
			return false;
				
	}		
	w25qxx.PageSize=256;
	w25qxx.SectorSize=0x1000;
	w25qxx.SectorCount=w25qxx.BlockCount*16;
	w25qxx.PageCount=(w25qxx.SectorCount*w25qxx.SectorSize)/w25qxx.PageSize;
	w25qxx.BlockSize=w25qxx.SectorSize*16;
	w25qxx.CapacityInKiloByte=(w25qxx.SectorCount*w25qxx.SectorSize)/1024;
	W25qxx_ReadUniqID();
	W25qxx_ReadStatusRegister(1);
	W25qxx_ReadStatusRegister(2);
	W25qxx_ReadStatusRegister(3);
	w25qxx.Lock=0;	
	return true;
}	
//###################################################################################################################
void	W25qxx_EraseChip(void)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;	
	W25qxx_WriteEnable();
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0xC7);
  LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_WaitForWriteEnd();
	W25qxx_Delay(10);
	w25qxx.Lock=0;	
}
//###################################################################################################################
void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while(w25qxx.Lock==1) W25qxx_Delay(1);
	w25qxx.Lock=1;	
	W25qxx_WaitForWriteEnd();
	SectorAddr = SectorAddr * w25qxx.SectorSize;
	W25qxx_WriteEnable();
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Spi(0x20);
	if(w25qxx.ID>=W25Q256) W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);
	W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
	W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
	W25qxx_Spi(SectorAddr & 0xFF);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_WaitForWriteEnd();
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;	
	W25qxx_WaitForWriteEnd();
	BlockAddr = BlockAddr * w25qxx.SectorSize*16;
  W25qxx_WriteEnable();
  LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0xD8);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((BlockAddr & 0xFF000000) >> 24);
  W25qxx_Spi((BlockAddr & 0xFF0000) >> 16);
  W25qxx_Spi((BlockAddr & 0xFF00) >> 8);
  W25qxx_Spi(BlockAddr & 0xFF);
  LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_WaitForWriteEnd();
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
uint32_t	W25qxx_PageToSector(uint32_t	PageAddress)
{
	return ((PageAddress*w25qxx.PageSize)/w25qxx.SectorSize);
}
//###################################################################################################################
uint32_t	W25qxx_PageToBlock(uint32_t	PageAddress)
{
	return ((PageAddress*w25qxx.PageSize)/w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t	W25qxx_SectorToBlock(uint32_t	SectorAddress)
{
	return ((SectorAddress*w25qxx.SectorSize)/w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t	W25qxx_SectorToPage(uint32_t	SectorAddress)
{
	return (SectorAddress*w25qxx.SectorSize)/w25qxx.PageSize;
}
//###################################################################################################################
uint32_t	W25qxx_BlockToPage(uint32_t	BlockAddress)
{
	return (BlockAddress*w25qxx.BlockSize)/w25qxx.PageSize;
}
//###################################################################################################################
bool 	W25qxx_IsEmptyPage(uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_PageSize)
{
	while(w25qxx.Lock==1)
	W25qxx_Delay(1);
	w25qxx.Lock=1;	
	if(((NumByteToCheck_up_to_PageSize+OffsetInByte)>w25qxx.PageSize)||(NumByteToCheck_up_to_PageSize==0))
		NumByteToCheck_up_to_PageSize=w25qxx.PageSize-OffsetInByte;
	uint8_t	pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<w25qxx.PageSize; i+=sizeof(pBuffer))
	{
		LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
		WorkAddress=(i+Page_Address*w25qxx.PageSize);
		W25qxx_Spi(0x0B);
		if(w25qxx.ID>=W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		spi_receive(pBuffer, sizeof(pBuffer));
		LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((w25qxx.PageSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<w25qxx.PageSize; i++)
		{
			LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
			WorkAddress=(i+Page_Address*w25qxx.PageSize);
			W25qxx_Spi(0x0B);
			if(w25qxx.ID>=W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			spi_receive(pBuffer, sizeof(pBuffer));
			LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	w25qxx.Lock=0;
	return true;	
	NOT_EMPTY:
	w25qxx.Lock=0;
	return false;
}
//###################################################################################################################
bool 	W25qxx_IsEmptySector(uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_SectorSize)
{
	while(w25qxx.Lock==1)
	W25qxx_Delay(1);
	w25qxx.Lock=1;	
	if((NumByteToCheck_up_to_SectorSize>w25qxx.SectorSize)||(NumByteToCheck_up_to_SectorSize==0))
		NumByteToCheck_up_to_SectorSize=w25qxx.SectorSize;
	uint8_t	pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<w25qxx.SectorSize; i+=sizeof(pBuffer))
	{
		LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
		WorkAddress=(i+Sector_Address*w25qxx.SectorSize);
		W25qxx_Spi(0x0B);
		if(w25qxx.ID>=W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		spi_receive(pBuffer, sizeof(pBuffer));
		LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((w25qxx.SectorSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<w25qxx.SectorSize; i++)
		{
			LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
			WorkAddress=(i+Sector_Address*w25qxx.SectorSize);
			W25qxx_Spi(0x0B);
			if(w25qxx.ID>=W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			spi_receive(pBuffer, sizeof(pBuffer));
			LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	w25qxx.Lock=0;
	return true;	
	NOT_EMPTY:
	w25qxx.Lock=0;
	return false;
}
//###################################################################################################################
bool 	W25qxx_IsEmptyBlock(uint32_t Block_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_BlockSize)
{
	while(w25qxx.Lock==1)
	W25qxx_Delay(1);
	w25qxx.Lock=1;	
	if((NumByteToCheck_up_to_BlockSize>w25qxx.BlockSize)||(NumByteToCheck_up_to_BlockSize==0))
		NumByteToCheck_up_to_BlockSize=w25qxx.BlockSize;
	uint8_t	pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<w25qxx.BlockSize; i+=sizeof(pBuffer))
	{
		LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
		WorkAddress=(i+Block_Address*w25qxx.BlockSize);
		W25qxx_Spi(0x0B);
		if(w25qxx.ID>=W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		spi_receive(pBuffer, sizeof(pBuffer));
		LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((w25qxx.BlockSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<w25qxx.BlockSize; i++)
		{
			LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
			WorkAddress=(i+Block_Address*w25qxx.BlockSize);
			W25qxx_Spi(0x0B);
			if(w25qxx.ID>=W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			spi_receive(pBuffer, sizeof(pBuffer));
			LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	w25qxx.Lock=0;
	return true;	
	NOT_EMPTY:
	w25qxx.Lock=0;
	return false;
}
//###################################################################################################################
void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	W25qxx_WaitForWriteEnd();
  W25qxx_WriteEnable();
  LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0x02);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
  W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
  W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
  W25qxx_Spi(WriteAddr_inBytes & 0xFF);
  W25qxx_Spi(pBuffer);
  LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_WaitForWriteEnd();
	w25qxx.Lock=0;
}
//###################################################################################################################
void 	W25qxx_WritePage(uint8_t *pBuffer	,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToWrite_up_to_PageSize)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	if(((NumByteToWrite_up_to_PageSize+OffsetInByte)>w25qxx.PageSize)||(NumByteToWrite_up_to_PageSize==0))
		NumByteToWrite_up_to_PageSize=w25qxx.PageSize-OffsetInByte;
	if((OffsetInByte+NumByteToWrite_up_to_PageSize) > w25qxx.PageSize)
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize-OffsetInByte;
	W25qxx_WaitForWriteEnd();
  W25qxx_WriteEnable();
  LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0x02);
	Page_Address = (Page_Address*w25qxx.PageSize)+OffsetInByte;	
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
  W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
  W25qxx_Spi((Page_Address & 0xFF00) >> 8);
  W25qxx_Spi(Page_Address&0xFF);
  spi_transmit(pBuffer, NumByteToWrite_up_to_PageSize);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_WaitForWriteEnd();
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
void 	W25qxx_WriteSector(uint8_t *pBuffer	,uint32_t Sector_Address,uint32_t OffsetInByte	,uint32_t NumByteToWrite_up_to_SectorSize)
{
	if((NumByteToWrite_up_to_SectorSize>w25qxx.SectorSize)||(NumByteToWrite_up_to_SectorSize==0))
		NumByteToWrite_up_to_SectorSize=w25qxx.SectorSize;
	if(OffsetInByte>=w25qxx.SectorSize)
	{
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToWrite;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToWrite = w25qxx.SectorSize-OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;	
	StartPage = W25qxx_SectorToPage(Sector_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_WritePage(pBuffer,StartPage,LocalOffset,BytesToWrite);
		StartPage++;
		BytesToWrite-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToWrite>0);		
}
//###################################################################################################################
void 	W25qxx_WriteBlock	(uint8_t* pBuffer ,uint32_t Block_Address	,uint32_t OffsetInByte	,uint32_t	NumByteToWrite_up_to_BlockSize)
{
	if((NumByteToWrite_up_to_BlockSize>w25qxx.BlockSize)||(NumByteToWrite_up_to_BlockSize==0))
		NumByteToWrite_up_to_BlockSize=w25qxx.BlockSize;
	if(OffsetInByte>=w25qxx.BlockSize)
	{
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToWrite;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToWrite = w25qxx.BlockSize-OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;	
	StartPage = W25qxx_BlockToPage(Block_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_WritePage(pBuffer,StartPage,LocalOffset,BytesToWrite);
		StartPage++;
		BytesToWrite-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToWrite>0);		
}
//###################################################################################################################
void 	W25qxx_ReadByte(uint8_t *pBuffer,uint32_t Bytes_Address)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
  W25qxx_Spi(0x0B);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((Bytes_Address & 0xFF000000) >> 24);
  W25qxx_Spi((Bytes_Address & 0xFF0000) >> 16);
  W25qxx_Spi((Bytes_Address& 0xFF00) >> 8);
  W25qxx_Spi(Bytes_Address & 0xFF);
	W25qxx_Spi(0);
	*pBuffer = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	w25qxx.Lock=0;
}
//###################################################################################################################
void W25qxx_ReadBytes(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Spi(0x0B);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((ReadAddr & 0xFF000000) >> 24);
  W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
  W25qxx_Spi((ReadAddr& 0xFF00) >> 8);
  W25qxx_Spi(ReadAddr & 0xFF);
	W25qxx_Spi(0);
	spi_receive(pBuffer, NumByteToRead);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
void 	W25qxx_ReadPage(uint8_t *pBuffer,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_PageSize)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	if((NumByteToRead_up_to_PageSize>w25qxx.PageSize)||(NumByteToRead_up_to_PageSize==0))
		NumByteToRead_up_to_PageSize=w25qxx.PageSize;
	if((OffsetInByte+NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
		NumByteToRead_up_to_PageSize = w25qxx.PageSize-OffsetInByte;
	Page_Address = Page_Address*w25qxx.PageSize+OffsetInByte;
	LL_GPIO_ResetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Spi(0x0B);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
  W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
  W25qxx_Spi((Page_Address& 0xFF00) >> 8);
  W25qxx_Spi(Page_Address & 0xFF);
	W25qxx_Spi(0);
	spi_receive(pBuffer, NumByteToRead_up_to_PageSize);
	LL_GPIO_SetOutputPin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN);
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
void 	W25qxx_ReadSector(uint8_t *pBuffer,uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_SectorSize)
{	
	if((NumByteToRead_up_to_SectorSize>w25qxx.SectorSize)||(NumByteToRead_up_to_SectorSize==0))
		NumByteToRead_up_to_SectorSize=w25qxx.SectorSize;
	if(OffsetInByte>=w25qxx.SectorSize)
	{
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToRead;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToRead = w25qxx.SectorSize-OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;	
	StartPage = W25qxx_SectorToPage(Sector_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_ReadPage(pBuffer,StartPage,LocalOffset,BytesToRead);
		StartPage++;
		BytesToRead-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToRead>0);		
}
//###################################################################################################################
void 	W25qxx_ReadBlock(uint8_t* pBuffer,uint32_t Block_Address,uint32_t OffsetInByte,uint32_t	NumByteToRead_up_to_BlockSize)
{
	if((NumByteToRead_up_to_BlockSize>w25qxx.BlockSize)||(NumByteToRead_up_to_BlockSize==0))
		NumByteToRead_up_to_BlockSize=w25qxx.BlockSize;
	if(OffsetInByte>=w25qxx.BlockSize)
	{
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToRead;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToRead = w25qxx.BlockSize-OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;	
	StartPage = W25qxx_BlockToPage(Block_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_ReadPage(pBuffer,StartPage,LocalOffset,BytesToRead);
		StartPage++;
		BytesToRead-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToRead>0);		
}
//###################################################################################################################

