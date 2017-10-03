#include "common.h"
#include "LPLD_OV7725_Eagle.h"

//内部函数声明
static uint8 ov7725_eagle_reg_init(void);
static void ov7725_eagle_port_init(void);
static void ov7725_eagle_vsync(void);
static void ov7725_eagle_dma();
static void img_extract(void *, void *, uint32 );
void H_Process(uint8);
void V_Process(void);

static GPIO_InitTypeDef gpio_init_struct0;
static GPIO_InitTypeDef gpio_init_struct1;
static GPIO_InitTypeDef gpio_init_struct2;
static DMA_InitTypeDef dma_init_struct;

/*OV7725初始化配置表*/
reg_s ov7725_eagle_reg[] =
{
  //寄存器，寄存器值次
  {OV7725_COM4         , 0xC1},
  {OV7725_CLKRC        , 0x00},
  {OV7725_COM2         , 0x03},
  {OV7725_COM3         , 0xD0},
  {OV7725_COM7         , 0x40},
  {OV7725_HSTART       , 0x3F},
  {OV7725_HSIZE        , 0x50},
  {OV7725_VSTRT        , 0x03},
  {OV7725_VSIZE        , 0x78},
  {OV7725_HREF         , 0x00},
  {OV7725_SCAL0        , 0x0A},
  {OV7725_AWB_Ctrl0    , 0xE0},
  {OV7725_DSPAuto      , 0xff},
  {OV7725_DSP_Ctrl2    , 0x0C},
  {OV7725_DSP_Ctrl3    , 0x00},
  {OV7725_DSP_Ctrl4    , 0x00},

#if (CAMERA_W == 80)
  {OV7725_HOutSize     , 0x14},
#elif (CAMERA_W == 160)
  {OV7725_HOutSize     , 0x28},
#elif (CAMERA_W == 240)
  {OV7725_HOutSize     , 0x3c},
#elif (CAMERA_W == 320)
  {OV7725_HOutSize     , 0x50},
#else

#endif

#if (CAMERA_H == 60 )
  {OV7725_VOutSize     , 0x1E},
#elif (CAMERA_H == 120 )
  {OV7725_VOutSize     , 0x3c},
#elif (CAMERA_H == 180 )
  {OV7725_VOutSize     , 0x5a},
#elif (CAMERA_H == 240 )
  {OV7725_VOutSize     , 0x78},
#else

#endif

  {OV7725_EXHCH        , 0x00},
  {OV7725_GAM1         , 0x0c},
  {OV7725_GAM2         , 0x16},
  {OV7725_GAM3         , 0x2a},
  {OV7725_GAM4         , 0x4e},
  {OV7725_GAM5         , 0x61},
  {OV7725_GAM6         , 0x6f},
  {OV7725_GAM7         , 0x7b},
  {OV7725_GAM8         , 0x86},
  {OV7725_GAM9         , 0x8e},
  {OV7725_GAM10        , 0x97},
  {OV7725_GAM11        , 0xa4},
  {OV7725_GAM12        , 0xaf},
  {OV7725_GAM13        , 0xc5},
  {OV7725_GAM14        , 0xd7},
  {OV7725_GAM15        , 0xe8},
  {OV7725_SLOP         , 0x20},
  {OV7725_LC_RADI      , 0x00},
  {OV7725_LC_COEF      , 0x13},
  {OV7725_LC_XC        , 0x08},
  {OV7725_LC_COEFB     , 0x14},
  {OV7725_LC_COEFR     , 0x17},
  {OV7725_LC_CTR       , 0x05},
  {OV7725_BDBase       , 0x99},
  {OV7725_BDMStep      , 0x03},
  {OV7725_SDE          , 0x04},
  {OV7725_BRIGHT       , 0x00},
  {OV7725_CNST         , 0x3C},
  {OV7725_SIGN         , 0x06},
  {OV7725_UVADJ0       , 0x11},
  {OV7725_UVADJ1       , 0x02},

};

uint8 ov7725_eagle_cfgnum = sizeof( ov7725_eagle_reg )/2 ; /*结构体数组成员数目*/
uint8 img_buffer[CAMERA_SIZE],img[CAMERA_H][CAMERA_W];
uint8 Row_Counter;

/*!
 *  @brief      鹰眼ov7725初始化
 *  @since      v5.0
 */
uint8 ov7725_eagle_init(void)
{
  while(ov7725_eagle_reg_init() == 0);
  ov7725_eagle_port_init();
  return 0;
}

/*!
 *  @brief      鹰眼ov7725寄存器 初始化
 *  @return     初始化结果（0表示失败，1表示成功）
 *  @since      v5.0
 */
uint8 ov7725_eagle_reg_init(void)
{
  uint16 i = 0;
  uint8 Sensor_IDCode = 0;
  SCCB_GPIO_init();

  //OV7725_Delay_ms(50);
  if( 0 == SCCB_WriteByte ( OV7725_COM7, 0x80 ) ) /*复位sensor */
  {
    printf("\n警告:SCCB写数据错误");
    return 0 ;
  }

  //LPLD_LPTMR_DelayMs(50);

  if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, OV7725_VER ) )    /* 读取sensor ID号*/
  {
    printf("\n警告:读取ID失败");
    return 0;
  }
  printf("\nGet ID success，SENSOR ID is 0x%x", Sensor_IDCode);
  printf("\nConfig Register Number is %d ", ov7725_eagle_cfgnum);
  if(Sensor_IDCode == OV7725_ID)
  {
    for( i = 0 ; i < ov7725_eagle_cfgnum ; i++ )
    {
      if( 0 == SCCB_WriteByte(ov7725_eagle_reg[i].addr, ov7725_eagle_reg[i].val) )
      {
        printf("\n警告:写寄存器0x%x失败", ov7725_eagle_reg[i].addr);
        return 0;
      }
    }
  }
  else
  {
    return 0;
  }
  printf("\nOV7725 Register Config Success!");
  return 1;
}

/*!
 *  @brief      鹰眼ov7725管脚初始化（内部调用）
 *  @since      v5.0
 */
void ov7725_eagle_port_init()
{
  gpio_init_struct0.GPIO_PTx = PTA;      //场中断来时，初始化DMA，并启动DMA传输
  gpio_init_struct0.GPIO_Pins = GPIO_Pin29;
  gpio_init_struct0.GPIO_Dir = DIR_INPUT;
  gpio_init_struct0.GPIO_PinControl = IRQC_RI|INPUT_PF_EN;
  gpio_init_struct0.GPIO_Isr = ov7725_eagle_vsync;
  LPLD_GPIO_Init(gpio_init_struct0);
    
  gpio_init_struct1.GPIO_PTx = PTA;      //PCLK上升沿时采集图像
  gpio_init_struct1.GPIO_Pins = GPIO_Pin27;
  gpio_init_struct1.GPIO_Dir = DIR_INPUT;
  gpio_init_struct1.GPIO_PinControl = IRQC_DMAFA|INPUT_PF_EN;
  LPLD_GPIO_Init(gpio_init_struct1);
    
  gpio_init_struct2.GPIO_PTx = PTB;
  gpio_init_struct2.GPIO_Pins =GPIO_Pin0_7; //GPIO_Pin0|GPIO_Pin1|GPIO_Pin2|GPIO_Pin3|GPIO_Pin4|GPIO_Pin5|GPIO_Pin6|GPIO_Pin7;
  gpio_init_struct2.GPIO_Dir = DIR_INPUT;
  gpio_init_struct2.GPIO_PinControl = IRQC_DIS;
  LPLD_GPIO_Init(gpio_init_struct2);
    
  //初始化DMA工作参数
  dma_init_struct.DMA_CHx = DMA_CH0;           //使用Ch0通道
  dma_init_struct.DMA_Req = PORTA_DMAREQ;      //A口是DMA请求源
  dma_init_struct.DMA_MajorLoopCnt = CAMERA_W/8;        //主循环计数
  dma_init_struct.DMA_MinorByteCnt = 1;       //次循环传输字节计数
  dma_init_struct.DMA_SourceAddr = (uint32)&(PTB->PDIR);       //源地址
  dma_init_struct.DMA_SourceDataSize = DMA_SRC_8BIT;     //源地址传输数据宽度8位
  dma_init_struct.DMA_SourceAddrOffset = 0;        //源地址偏移0
  dma_init_struct.DMA_DestAddr = (uint32)img_buffer;     //目的地址
  dma_init_struct.DMA_DestDataSize = DMA_DST_8BIT;      //目的地址传输数据宽度8位
  dma_init_struct.DMA_DestAddrOffset = 1;       //目的地址偏移1
  dma_init_struct.DMA_AutoDisableReq = TRUE;   //使用自动禁用请求
  dma_init_struct.DMA_MajorCompleteIntEnable = TRUE;    //主循环结束中断允许
  dma_init_struct.DMA_Isr = ov7725_eagle_dma;
  //初始化DMA
  LPLD_DMA_Init(dma_init_struct);
  //开DMA中断
  LPLD_DMA_EnableIrq(dma_init_struct);
  
  LPLD_GPIO_ClearIntFlag(PORTA);
  LPLD_GPIO_EnableIrq(gpio_init_struct0);
}

/*!
 *  @brief      鹰眼ov7725场中断服务函数,在一场的开始执行
 *  @since      v5.0
 */
void ov7725_eagle_vsync(void)
{
  LPLD_GPIO_ClearIntFlag(PORTA);
  LPLD_DMA_EnableReq(DMA_CH0);
  LPLD_GPIO_ClearIntFlag(PORTA);
  LPLD_DMA_LoadDstAddr(DMA_CH0,(uint32)img_buffer);
  Row_Counter = 0;
}

/*!
 *  @brief      鹰眼ov7725 DMA中断服务函数
 *  @since      v5.0
 */
void ov7725_eagle_dma()
{
  if(Row_Counter < CAMERA_H-1)           
  {
    LPLD_DMA_EnableReq(DMA_CH0);
  }
  
  img_extract(&img[Row_Counter][CAMERA_W-1],img_buffer+Row_Counter*CAMERA_W/8,CAMERA_W/8);
  H_Process(Row_Counter);
  
  if(Row_Counter < CAMERA_H-1)
  {
    Row_Counter++; 
  }else
  {
    V_Process();
  }    
}

//压缩二值化图像解压（空间 换 时间 解压）
//srclen 是二值化图像的占用空间大小
//【鹰眼解压】鹰眼图像解压，转为 二维数组 - 智能车资料区 - 山外论坛 http://vcan123.com/forum.php?mod=viewthread&tid=17&ctid=6
//解压的时候，里面有个数组，配置黑、白对应的值是多少。
void img_extract(void *dst, void *src, uint32 srclen)
{
  uint8_t colour[2] = {1, 0}; //0 和 1 分别对应的颜色
  uint8_t * mdst = dst;
  uint8_t * msrc = src;
  //注：山外的摄像头 0 表示 白色，1表示 黑色
  uint8_t tmpsrc;
  while(srclen --)
  {
    tmpsrc = *msrc++;
    *mdst-- = colour[ (tmpsrc >> 7 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 6 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 5 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 4 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 3 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 2 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 1 ) & 0x01 ];
    *mdst-- = colour[ (tmpsrc >> 0 ) & 0x01 ];
  }
}