/****************************************************************************
* Title                 :   Perimeter wire module
* Filename              :   perimeter.c
* Author                :   Nekraus
* Origin Date           :   24/09/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file perimeter.c
*  \brief 
*
*/
/******************************************************************************
* Includes
*******************************************************************************/
#include "main.h"
#include "board.h"
#include "perimeter.h" 
#include <math.h>
#include <stdlib.h>

#ifdef OPTION_PERIMETER

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define PERIMETER_NBPTS 1284 /* 12 ms / 9.333 µs */
#define PERIMETER_OVERSAMPLING 3
#define PERIMETER_AVERAGE_N 3

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
ADC_HandleTypeDef ADC_Handle;
extern DMA_HandleTypeDef hdma_adc;


/* Expected perimeter signal shape. The absoulte values do not matter but the sum of all elements must be zero! */
static const int32_t sigcode1[]={ -2, -2, -2, 2, 2, 2, -2, -2, 2, 2, 2, 2, 2, 2, 2, 0, -2, -2, -2, 1, 2, 2, 2, 2, 2, 1, 0, 0, -2, -2, -2, -2, -2, -2, -2, -1, -1 };
#define SIGCODE1_LENGTH (sizeof(sigcode1)/sizeof(int32_t))
static const int32_t sigcode2[]={ -2, -3, 0, 3, 3, -1, -2, -1, 3, 3, 3, 3, 3, 2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, 3, 3, 3, 3, 3, 3, 1, 0, -1, -2, -2, -2, -2, -2, -2, -1, -1 };
#define SIGCODE2_LENGTH (sizeof(sigcode2)/sizeof(int32_t))
uint16_t pu16_PerimeterADC_buffer[PERIMETER_NBPTS]; /* Input from perimeter coil */

bool perimeter_bFlagIT = false;
static const int32_t *sigcode=NULL;
static int sigcode_length;
static int print_pos=-1;

float coilSigSum[COIL_OFF] = {0.0,0.0,0.0};
int coilSigN[COIL_OFF]={0,0,0};

perimeter_CoilNumber_e idxCoil = COIL_LEFT;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static double corrFilter(void);
void perimeter_SetCoil(perimeter_CoilNumber_e idx);

/******************************************************************************
*  Public Functions
*******************************************************************************/
// TODO: We can't use ADC1 exclusively for perimeter for the Yardforce 500B, since it is used for charging already!
// 		 This could be multiplexed, but this is a task for later as perimeter sensing is not needed for now.
void Perimeter_vInit(void){
#if BOARD_YARDFORCE500_VARIANT_ORIG
 ADC_ChannelConfTypeDef sConfig = {0};
    
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**ADC1 GPIO Configuration
    PA6     ------> Perometer Sense
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    ADC_Handle.Instance = ADC1;
    ADC_Handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
    ADC_Handle.Init.ContinuousConvMode = ENABLE;
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    ADC_Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIG_EDGE_NONE;
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_Handle.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&ADC_Handle) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_6; // PA6 Perimeter sense
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5; /* 9.333 µs with the adc clock to 9mhz */
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &sConfig) != HAL_OK)
    {
       Error_Handler();
    }

    /* ADC DMA Init */
    /* ADC Init */

    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode = DMA_NORMAL;
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      Error_Handler();
    }

   __HAL_LINKDMA(&ADC_Handle,DMA_Handle,hdma_adc);

  // calibrate  - important for accuracy !
  HAL_ADCEx_Calibration_Start(&ADC_Handle); 

  perimeter_bFlagIT = false;
  idxCoil = COIL_OFF;
  perimeter_SetCoil(idxCoil);
  HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t*)&pu16_PerimeterADC_buffer[0],PERIMETER_NBPTS);
  __HAL_DMA_DISABLE_IT(&hdma_adc,DMA_IT_HT);
#endif
}

void Perimeter_vApp(void){

  if(sigcode && perimeter_bFlagIT == true){
    if (print_pos>=0) {
      if (print_pos<PERIMETER_NBPTS) {
        for (int c=0; c<4 && print_pos<PERIMETER_NBPTS; c++) {
          DB_TRACE("%c%d",print_pos ? ',' : '\n',(int32_t) pu16_PerimeterADC_buffer[print_pos]);
          print_pos++;
        }
        return;
      }
      print_pos=0;
    }
    perimeter_bFlagIT = false;

    if (print_pos<0) {
      coilSigSum[idxCoil]+=corrFilter();
      coilSigN[idxCoil]++;
      idxCoil ++;
      if(idxCoil == COIL_OFF){
        idxCoil = COIL_LEFT;
      }
      perimeter_SetCoil(idxCoil);
    }
    HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t*)&pu16_PerimeterADC_buffer[0],PERIMETER_NBPTS);
    __HAL_DMA_DISABLE_IT(&hdma_adc,DMA_IT_HT);
  }
}

void Perimeter_ListenOn(uint8_t sig) {
  const int32_t *oldsigcode=sigcode;
  switch (sig) {
    case 1:
    case 0x80:
    case 0x81:
    case 0x82:
      sigcode=sigcode1;
      sigcode_length=SIGCODE1_LENGTH;
      break;
    case 2:
      sigcode=sigcode2;
      sigcode_length=SIGCODE2_LENGTH;
      break;
    default:
      sigcode=NULL;
  }
  if (sigcode) {
    print_pos=sig & 0x80 ? 0 : -1;
    if (!oldsigcode) {
      idxCoil=COIL_LEFT;
      perimeter_SetCoil(idxCoil);
      for (int i=0; i<COIL_OFF; i++) {
        coilSigSum[i]=coilSigN[i]=0;
      }
      HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t*)&pu16_PerimeterADC_buffer[0],PERIMETER_NBPTS);
      __HAL_DMA_DISABLE_IT(&hdma_adc,DMA_IT_HT);
    }
    if (sig & 0x80) {
      idxCoil=sig & 3;
      perimeter_SetCoil(idxCoil);
    }
  } else {
    print_pos=-1;
  }
}

int Perimeter_IsActive(void) {
  return sigcode!=NULL;
}

int Perimeter_UpdateMsg(float *left,float *center,float *right) {
  if (!sigcode || coilSigN[COIL_LEFT]<PERIMETER_AVERAGE_N
      || coilSigN[COIL_MIDDLE]<PERIMETER_AVERAGE_N  || coilSigN[COIL_RIGHT]<PERIMETER_AVERAGE_N)
  {
    return 0;
  }
	*left=coilSigSum[COIL_LEFT]/coilSigN[COIL_LEFT];
	*center=coilSigSum[COIL_MIDDLE]/coilSigN[COIL_MIDDLE];
	*right=coilSigSum[COIL_RIGHT]/coilSigN[COIL_RIGHT];
  for (int i=0; i<COIL_OFF; i++) coilSigSum[i]=coilSigN[i]=0;
  return 1;
}

int Perimeter_UsesDebug(void) {
  return print_pos>=0;
}

void PERIMETER_vITHandle(void){
  /* stop DMA and set flag */
  HAL_ADC_Stop_DMA(&ADC_Handle);
  perimeter_bFlagIT = true;
}


/******************************************************************************
*  Private Functions
*******************************************************************************/
/** 
 * @brief matched filter (cross correlation)
 * @return detected signal strength
 */
double corrFilter() {

  /* Calculate oversampling: n=effective number of samples */
  const int n=PERIMETER_NBPTS/PERIMETER_OVERSAMPLING;
  #if PERIMETER_OVERSAMPLING>16
  #error Possible overflow in unit16_t
  #endif
  {
    uint16_t *p=pu16_PerimeterADC_buffer;
    for (int i=0; i<n; i++) {
      uint16_t tmp=*(p++);
      for (int j=PERIMETER_OVERSAMPLING; --j>0; ) {
        tmp+=*(p++);
      }
      pu16_PerimeterADC_buffer[i]=tmp;
    }
  }

  /* We store the correlation result in the unused part of the same array! */
#if PERIMETER_OVERSAMPLING<3
#error Need oversampling for data storage
#endif
  int32_t *correlations=(int32_t*) (pu16_PerimeterADC_buffer+n);

  int32_t corr_max_abs=0,corr_max=0; // Maximum (absolute) correlation
  int corr_max_pos; // Position of the maximum correlation

  for (int i=0; i<=n-sigcode_length; i++) {
    int32_t sum=0;
    uint16_t *datap=pu16_PerimeterADC_buffer+i;
    const int32_t *sigcodep=sigcode;

    for (int j=0; j<sigcode_length; j++) sum+=*(datap++)**(sigcodep++);
    correlations[i]=sum;
    if (sum<0) {
      if (-sum>corr_max_abs) {
        corr_max=sum;
        corr_max_abs=-sum;
        corr_max_pos=i;
      }
    } else {
      if (sum>corr_max_abs) {
        corr_max_abs=corr_max=sum;
        corr_max_pos=i;
      }
    }
  }

  if (corr_max_abs==0) return 0;

  /* The perimeter signal seems to be automatically amplified until the whole ADC range is used.
   * => Calculate signal from signal to noise ratio.
   */
  int64_t sx2=0,sx=0;
  int sn=0;
  
  int count=n/5;
  int p=corr_max_pos-3*sigcode_length/2; // Distance to signal;
  while (count>0 && p>=0) {
    int64_t s=correlations[p];
    sx+=s;
    sx2+=s*s;
    p--;
    count--;
    sn++;
  }
  count=n/5;
  p=corr_max_pos+3*sigcode_length/2; // Distance to signal;
  while (count>0 && p<n-sigcode_length) {
    int64_t s=correlations[p];
    sx+=s;
    sx2+=s*s;
    p++;
    count--;
    sn++;
  }

  if (sn<=1) return 0;
  double noiseDeviation=sqrt((sx2-sx*sx/(double) sn)/(sn-1));
  if (noiseDeviation<1) return corr_max;
  return corr_max/noiseDeviation;
}

void perimeter_SetCoil(perimeter_CoilNumber_e idx){
  switch (idx)
  {
  case COIL_RIGHT:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    break;
    }
  case COIL_MIDDLE:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    break;
    }
  case COIL_LEFT:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    break;
    }
  case COIL_OFF:
  default:
    {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    break;
    }

    
  }
}
#endif // OPTION_PERIMETER
