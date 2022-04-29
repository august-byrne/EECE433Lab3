
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
* Edits for eece433 lab 3 were done by August Byrne.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
* 05/06/2021 August Byrne
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/*****************************************************************************************************
* Defined constants for processing
*****************************************************************************************************/
#define FFT_LENGTH      512
//Supported Lengths: 32, 64, 128, 256, 512, 1024, 2048
                                //Must be <= DSP_SAMPLES_PER_BLOCK unless zero padded
//#define SAMPLE_RATE_HZ  48000
//#define FREQ_BIN_SIZE   SAMPLE_RATE_HZ/FFT_LENGTH
/******************************************************************************************/
static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];
static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;
/*****************************************************************************************************
* Public Function Prototypes
*****************************************************************************************************/
// FFT output buffers and variables.
static q31_t fftResultLeft[DSP_SAMPLES_PER_BLOCK*2];
static q31_t fftResultMagLeft[DSP_SAMPLES_PER_BLOCK];
static q31_t fftResultRight[DSP_SAMPLES_PER_BLOCK*2];
static q31_t fftResultMagRight[DSP_SAMPLES_PER_BLOCK];

static q31_t maxValueLeft = 0;
static uint32_t testIndexLeft = 0;
static uint32_t peakFreqLeft = 0;
static q31_t maxValueRight = 0;
static uint32_t testIndexRight = 0;
static uint32_t peakFreqRight = 0;

static float32_t cosineVals[DSP_SAMPLES_PER_BLOCK];
static float32_t BufferLeft[DSP_SAMPLES_PER_BLOCK];
static float32_t BufferRight[DSP_SAMPLES_PER_BLOCK];

static float32_t RxBufferLeft[DSP_SAMPLES_PER_BLOCK];
static float32_t RxBufferRight[DSP_SAMPLES_PER_BLOCK];

//define FFT instances
static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Left;
static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Right;
/*******************************************************************************************
* Private Function Prototypes
*******************************************************************************************/
static void  dspTask(void *p_arg);
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};
/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;

    for(int i=0;i<DSP_SAMPLES_PER_BLOCK;i++){
    	cosineVals[i] = arm_cos_f32(2*PI*i*20000/48000);
    }

    //init Real FFT instance
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Left,DSP_SAMPLES_PER_BLOCK,0,1);
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Right,DSP_SAMPLES_PER_BLOCK,0,1);

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();
}

/*******************************************************************************************
* dspTask
*******************************************************************************************/
static void dspTask(void *p_arg){
    OS_ERR os_err;
    INT8U buffer_index;
    (void)p_arg;
    while(1){
        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
        //AM Generator Code
        arm_q31_to_float(&dspInBuffer[DSP_LEFT_CH][buffer_index].samples[0],&BufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_q31_to_float(&dspInBuffer[DSP_RIGHT_CH][buffer_index].samples[0],&BufferRight[0],DSP_SAMPLES_PER_BLOCK);

        arm_mult_f32(&cosineVals[0],&BufferLeft[0],&RxBufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_mult_f32(&cosineVals[0],&BufferRight[0],&RxBufferRight[0],DSP_SAMPLES_PER_BLOCK);

        arm_float_to_q31(&RxBufferLeft[0],&dspOutBuffer[DSP_LEFT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);
        arm_float_to_q31(&RxBufferRight[0],&dspOutBuffer[DSP_RIGHT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);

        //FFT for both channels and FFT magnitude
        //(NOTE: The FFT modifies it's input variables, so the input of dspOutBuffer is no longer a valid source of data to be used afterwards.)
        // Note: if your computer can not handle the pass-through task and the FFT task at the same time, feel free to comment out the pass through task above.
        //Left Channel
        arm_rfft_q31(&arm_rfft_sR_q31_len128Left,&dspOutBuffer[DSP_LEFT_CH][buffer_index].samples[0],&fftResultLeft[0]);
        arm_cmplx_mag_q31(&fftResultLeft[0],&fftResultMagLeft[0],DSP_SAMPLES_PER_BLOCK);
        //Right Channel
        arm_rfft_q31(&arm_rfft_sR_q31_len128Right,&dspOutBuffer[DSP_RIGHT_CH][buffer_index].samples[0],&fftResultRight[0]);
        arm_cmplx_mag_q31(&fftResultRight[0],&fftResultMagRight[0],DSP_SAMPLES_PER_BLOCK);

        arm_max_q31(&fftResultMagLeft[0],DSP_SAMPLES_PER_BLOCK,&maxValueLeft,&testIndexLeft);
        arm_max_q31(&fftResultMagRight[0],DSP_SAMPLES_PER_BLOCK,&maxValueRight,&testIndexRight);

        peakFreqLeft = testIndexLeft*DSPSampleRateGet()/DSP_SAMPLES_PER_BLOCK;
        peakFreqRight = testIndexRight*DSPSampleRateGet()/DSP_SAMPLES_PER_BLOCK;

        // The following code implements a pass through
        //dspOutBuffer[DSP_LEFT_CH][buffer_index] = dspInBuffer[DSP_LEFT_CH][buffer_index]; //Left Channel
        //dspOutBuffer[DSP_RIGHT_CH][buffer_index] = dspInBuffer[DSP_RIGHT_CH][buffer_index]; //Right Channel

        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}

/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){
    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];
}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){
    return dspParams.ssize;
}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){
    return dspParams.srate;
}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){
    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];
}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}


