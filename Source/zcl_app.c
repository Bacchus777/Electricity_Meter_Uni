
#include "AF.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_PwrMgr.h"
#include "ZComDef.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "math.h"
#include <stdint.h>
#include <stdlib.h>

#include "nwk_util.h"
#include "zcl.h"
#include "zcl_app.h"
#include "zcl_diagnostic.h"
#include "zcl_general.h"
#include "zcl_ms.h"
#include "zcl_se.h"
#include "zcl_electrical_measurement.h"

#include "bdb.h"
#include "bdb_interface.h"
//#include "bdb_touchlink.h"
//#include "bdb_touchlink_target.h"

#include "gp_interface.h"

#include "Debug.h"

#include "OnBoard.h"

#include "commissioning.h"
#include "factory_reset.h"

#include "ds18b20.h"

/* HAL */
#include "hal_drivers.h"
#include "hal_key.h"
#include "hal_led.h"
#include "mercury200.h"
#include "mercury230.h"
#include "utils.h"
#include "version.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte    zclApp_TaskID;
uint16  OldDeviceModel = DEV_MERCURY_1PH;

uint8 SeqNum = 0;
//afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)Addr16Bit, .endPoint = 1, .addr.shortAddr = 0};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */
void user_delay_ms(uint32_t period);
void user_delay_ms(uint32_t period) { MicroWait(period * 1000); }
/*********************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclApp_BasicResetCB(void);
static ZStatus_t zclApp_ReadWriteAuthCB(afAddrType_t *srcAddr, zclAttrRec_t *pAttr, uint8 oper);

static void zclApp_RestoreAttributesFromNV(void);
static void zclApp_SaveAttributesToNV(void);

static void zclApp_Report(void);
static void zclApp_ReadMeter(void);
static void zclApp_ReadMercury_1ph(void);
static void zclApp_ReadMercury_3ph(void);

static void zclApp_HandleKeys(byte portAndAction, byte keyCode);

static void zclApp_InitMercuryUart(void);

static void zclApp_SetDeviceModel(uint8 DeviceModel);

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclApp_CmdCallbacks = {
    zclApp_BasicResetCB, // Basic Cluster Reset command
    NULL,                // Identify Trigger Effect command
    NULL,                // On/Off cluster commands
    NULL,                // On/Off cluster enhanced command Off with Effect
    NULL,                // On/Off cluster enhanced command On with Recall Global Scene
    NULL,                // On/Off cluster enhanced command On with Timed Off
    NULL,                // RSSI Location command
    NULL                 // RSSI Location Response command
};

void zclApp_Init(byte task_id) {
    HalLedSet(HAL_LED_ALL, HAL_LED_MODE_BLINK);

    zclApp_RestoreAttributesFromNV();
    zclApp_InitMercuryUart();
    zclApp_TaskID = task_id;

    bdb_RegisterSimpleDescriptor(&zclApp_FirstEP);
    zclGeneral_RegisterCmdCallbacks(zclApp_FirstEP.EndPoint, &zclApp_CmdCallbacks);
    zcl_registerAttrList(zclApp_FirstEP.EndPoint, zclApp_AttrsCount_FirstEP, zclApp_Attrs_FirstEP);
    zcl_registerReadWriteCB(zclApp_FirstEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
    
    bdb_RegisterSimpleDescriptor(&zclApp_SecondEP);
    zclGeneral_RegisterCmdCallbacks(zclApp_SecondEP.EndPoint, &zclApp_CmdCallbacks);
    zcl_registerAttrList(zclApp_SecondEP.EndPoint, zclApp_AttrsCount_SecondEP, zclApp_Attrs_SecondEP);
    zcl_registerReadWriteCB(zclApp_SecondEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);

    zcl_registerForMsg(zclApp_TaskID);


    RegisterForKeys(zclApp_TaskID);

    LREP("Build %s \r\n", zclApp_DateCodeNT);

    zclApp_SetDeviceModel(zclApp_Config.DeviceModel);

    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_EVT, zclApp_Config.MeasurementPeriod * 1000);
}

static void zclApp_HandleKeys(byte portAndAction, byte keyCode) {
    LREP("zclApp_HandleKeys portAndAction=0x%X keyCode=0x%X\r\n", portAndAction, keyCode);
    zclFactoryResetter_HandleKeys(portAndAction, keyCode);
    zclCommissioning_HandleKeys(portAndAction, keyCode);
    if (portAndAction & HAL_KEY_PRESS) {
        LREPMaster("Key press\r\n");
        zclApp_Report();
    }
}

static void zclApp_InitMercuryUart(void) {
    LREPMaster("Initializing Mercury UART \r\n");
    halUARTCfg_t halUARTConfig;
    halUARTConfig.configured = TRUE;
    halUARTConfig.baudRate = HAL_UART_BR_9600;
    halUARTConfig.flowControl = FALSE;
    halUARTConfig.flowControlThreshold = 48; // this parameter indicates number of bytes left before Rx Buffer
                                             // reaches maxRxBufSize
    halUARTConfig.idleTimeout = 10;          // this parameter indicates rx timeout period in millisecond
    halUARTConfig.rx.maxBufSize = 15;
    halUARTConfig.tx.maxBufSize = 15;
    halUARTConfig.intEnable = TRUE;
    halUARTConfig.callBackFunc = NULL;
    HalUARTInit();
    if (HalUARTOpen(MERCURY_PORT, &halUARTConfig) == HAL_UART_SUCCESS) {
        LREPMaster("Initialized Mercury UART \r\n");
    }
}

uint16 zclApp_event_loop(uint8 task_id, uint16 events) {
    LREP("events 0x%x \r\n", events);
    if (events & SYS_EVENT_MSG) {
        afIncomingMSGPacket_t *MSGpkt;
        while ((MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(zclApp_TaskID))) {
            LREP("MSGpkt->hdr.event 0x%X clusterId=0x%X\r\n", MSGpkt->hdr.event, MSGpkt->clusterId);
            switch (MSGpkt->hdr.event) {
            case KEY_CHANGE:
                zclApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
                break;

            case ZCL_INCOMING_MSG:
                if (((zclIncomingMsg_t *)MSGpkt)->attrCmd) {
                    osal_mem_free(((zclIncomingMsg_t *)MSGpkt)->attrCmd);
                }
                break;

            default:
                break;
            }

            // Release the memory
            osal_msg_deallocate((uint8 *)MSGpkt);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
    if (events & APP_REPORT_EVT) {
        LREPMaster("APP_REPORT_EVT\r\n");
        zclApp_Report();
        return (events ^ APP_REPORT_EVT);
    }

    if (events & APP_SAVE_ATTRS_EVT) {
        LREPMaster("APP_SAVE_ATTRS_EVT\r\n");
        zclApp_SaveAttributesToNV();
        return (events ^ APP_SAVE_ATTRS_EVT);
    }
    if (events & APP_READ_SENSORS_EVT) {
        LREPMaster("APP_READ_SENSORS_EVT\r\n");
        zclApp_ReadMeter();
        return (events ^ APP_READ_SENSORS_EVT);
    }
    return 0;
}

static void zclApp_ReadMeter(void) 
{
  switch (zclApp_Config.DeviceModel){
  case DEV_MERCURY_1PH:
    zclApp_ReadMercury_1ph();
    break;
  case DEV_MERCURY_3PH:
    zclApp_ReadMercury_3ph();
    break;
  default:
    break;
  }
}

static void zclApp_ReadMercury_1ph(void) 
{
  zclMercury_1ph_t const *mercury_1ph = &mercury_1ph_dev;
  static uint8 currentSensorsReadingPhase = 0;
  current_values_t CurrentValues;
  int16 temp;

  LREP("currentSensorsReadingPhase %d\r\n", currentSensorsReadingPhase);
    // FYI: split reading sensors into phases, so single call wouldn't block processor
    // for extensive ammount of time
  switch (currentSensorsReadingPhase++) {
  case 0: // 
    HalLedSet(HAL_LED_1, HAL_LED_MODE_FLASH);
    (*mercury_1ph->RequestMeasure)(zclApp_Config.DeviceAddress, 0x63);
    break;
  case 1:
    
    CurrentValues = (*mercury_1ph->ReadCurrentValues)(0);
    if (CurrentValues.Voltage[0] == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from meter\r\n");
      break;
    }
    zclApp_CurrentValues = CurrentValues;
  
//    bdb_RepChangedAttrValue(FIRST_ENDPOINT, ELECTRICAL, ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE);
    zclReportCmd_t *pReportCmd;
//    const uint8 NUM_ATTRIBUTES = 9;

    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (9 * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
     pReportCmd->numAttr = 9;

     pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE;
     pReportCmd->attrList[0].dataType = ZCL_UINT16;
     pReportCmd->attrList[0].attrData = (void *)(zclApp_CurrentValues.Voltage[0]); 
     
     pReportCmd->attrList[1].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT;
     pReportCmd->attrList[1].dataType = ZCL_UINT16;
     pReportCmd->attrList[1].attrData = (void *)(zclApp_CurrentValues.Current[0]); 
     
     pReportCmd->attrList[2].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER;
     pReportCmd->attrList[2].dataType = ZCL_INT16;
     pReportCmd->attrList[2].attrData = (void *)(zclApp_CurrentValues.Power[0]); 
     
     pReportCmd->attrList[3].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_PH_B;
     pReportCmd->attrList[3].dataType = ZCL_UINT16;
     pReportCmd->attrList[3].attrData = (void *)(zclApp_CurrentValues.Voltage[1]); 
     
     pReportCmd->attrList[4].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT_PH_B;
     pReportCmd->attrList[4].dataType = ZCL_UINT16;
     pReportCmd->attrList[4].attrData = (void *)(zclApp_CurrentValues.Current[1]); 
     
     pReportCmd->attrList[5].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_PH_B;
     pReportCmd->attrList[5].dataType = ZCL_INT16;
     pReportCmd->attrList[5].attrData = (void *)(zclApp_CurrentValues.Power[1]); 
     
     pReportCmd->attrList[6].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_PH_C;
     pReportCmd->attrList[6].dataType = ZCL_UINT16;
     pReportCmd->attrList[6].attrData = (void *)(zclApp_CurrentValues.Voltage[0]); 
     
     pReportCmd->attrList[7].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT_PH_C;
     pReportCmd->attrList[7].dataType = ZCL_UINT16;
     pReportCmd->attrList[7].attrData = (void *)(zclApp_CurrentValues.Current[0]); 
     
     pReportCmd->attrList[8].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_PH_C;
     pReportCmd->attrList[8].dataType = ZCL_INT16;
     pReportCmd->attrList[8].attrData = (void *)(zclApp_CurrentValues.Power[0]); 
     
     
//     zcl_SendReportCmd(zclApp_FirstEP.EndPoint, &inderect_DstAddr, ELECTRICAL, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, true, SeqNum++);
    }
    osal_mem_free(pReportCmd); 

    break;
  case 2:
    (*mercury_1ph->RequestMeasure)(zclApp_Config.DeviceAddress, 0x27);
    break;
  case 3:
    CurrentValues = (*mercury_1ph->ReadEnergy)(0);
    if (CurrentValues.Energy[1] == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from meter\r\n");
      break;
    }
    zclApp_CurrentValues.Energy[1] = CurrentValues.Energy[1];
    zclApp_CurrentValues.Energy[2] = CurrentValues.Energy[2];
    zclApp_CurrentValues.Energy[3] = CurrentValues.Energy[3];
    zclApp_CurrentValues.Energy[4] = CurrentValues.Energy[4];
    zclApp_CurrentValues.Energy[0] = CurrentValues.Energy[1] + CurrentValues.Energy[2] + CurrentValues.Energy[3] + CurrentValues.Energy[4];
//    bdb_RepChangedAttrValue(SECOND_ENDPOINT, SE_METERING, ATTRID_SE_METERING_CURR_TIER1_SUMM_DLVD);
    break;
  case 4:
    temp = readTemperature();
    if (temp == 1) {
      LREPMaster("ReadDS18B20 error\r\n");
      break;
    } else {
      zclApp_Temperature = temp;
      LREP("ReadDS18B20 t=%d\r\n", zclApp_Temperature);
    }
 //   bdb_RepChangedAttrValue(FIRST_ENDPOINT, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
    break;
  default:
    HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    osal_stop_timerEx(zclApp_TaskID, APP_READ_SENSORS_EVT);
    osal_clear_event(zclApp_TaskID, APP_READ_SENSORS_EVT);
    currentSensorsReadingPhase = 0;
    break;

  }

};


static void zclApp_ReadMercury_3ph(void) 
{
  zclMercury_3ph_t const *mercury_3ph = &mercury_3ph_dev;
  static uint8 currentSensorsReadingPhase = 0;
  current_values_t CurrentValues;
  uint32 Energy;
  int16 temp;

  LREP("currentSensorsReadingPhase %d\r\n", currentSensorsReadingPhase);
    // FYI: split reading sensors into phases, so single call wouldn't block processor
    // for extensive ammount of time
  switch (currentSensorsReadingPhase++) {
  case 0: // 
    HalLedSet(HAL_LED_1, HAL_LED_MODE_FLASH);
    (*mercury_3ph->StartStopData)(zclApp_Config.DeviceAddress, 0x01);
    break;
  case 1:
    if ((*mercury_3ph->CheckReady)()) 
      LREPMaster("Open OK\r\n");
    else 
      LREPMaster("Open FAIL\r\n");
    break;
  case 2:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_VOLTAGE);
    break;
  case 3:
    CurrentValues = (*mercury_3ph->ReadCurrentValues)(REQ_VOLTAGE);
    if (CurrentValues.Voltage[0] == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
      break;
    }
    for (int i = 0; i <= 2; i++) 
      zclApp_CurrentValues.Voltage[i] = CurrentValues.Voltage[i];
    LREP("Voltage 1 = %d\r\n", zclApp_CurrentValues.Voltage[0]);
    LREP("Voltage 2 = %d\r\n", zclApp_CurrentValues.Voltage[1]);
    LREP("Voltage 3 = %d\r\n", zclApp_CurrentValues.Voltage[2]);
    break;
  case 4:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_CURRENT);
    break;
  case 5:
    CurrentValues = (*mercury_3ph->ReadCurrentValues)(REQ_CURRENT);
    if (CurrentValues.Current[0] == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
      break;
    }
    for (int i = 0; i <= 2; i++) 
      zclApp_CurrentValues.Current[i] = CurrentValues.Current[i];
    LREP("Current 1 = %d\r\n", zclApp_CurrentValues.Current[0]);
    LREP("Current 2 = %d\r\n", zclApp_CurrentValues.Current[1]);
    LREP("Current 3 = %d\r\n", zclApp_CurrentValues.Current[2]);
    break;
  case 6:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_POWER);
    break;
  case 7:
    CurrentValues = (*mercury_3ph->ReadCurrentValues)(REQ_POWER);
    if (CurrentValues.Power[0] == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
      break;
    }
    for (int i = 0; i <= 2; i++) 
      zclApp_CurrentValues.Power[i] = CurrentValues.Power[i];
    LREP("Power 1 = %d\r\n", zclApp_CurrentValues.Power[0]);
    LREP("Power 2 = %d\r\n", zclApp_CurrentValues.Power[1]);
    LREP("Power 3 = %d\r\n", zclApp_CurrentValues.Power[2]);

/*    zclReportCmd_t *pReportCmd;
//    const uint8 NUM_ATTRIBUTES = 9;

    afAddrType_t *dstAddr;  
    dstAddr = (afAddrType_t *)osal_mem_alloc( sizeof(afAddrType_t));
    dstAddr->addr.shortAddr = 0;
    dstAddr->addrMode = afAddr16Bit;
    dstAddr->endPoint = FIRST_ENDPOINT;
    
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (9 * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
     pReportCmd->numAttr = 9;

     pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE;
     pReportCmd->attrList[0].dataType = ZCL_UINT16;
     pReportCmd->attrList[0].attrData = (void *)(zclApp_CurrentValues.Voltage[0]); 
     
     pReportCmd->attrList[1].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT;
     pReportCmd->attrList[1].dataType = ZCL_UINT16;
     pReportCmd->attrList[1].attrData = (void *)(zclApp_CurrentValues.Current[0]); 
     
     pReportCmd->attrList[2].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER;
     pReportCmd->attrList[2].dataType = ZCL_INT16;
     pReportCmd->attrList[2].attrData = (void *)(zclApp_CurrentValues.Power[0]); 
     
     pReportCmd->attrList[3].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_PH_B;
     pReportCmd->attrList[3].dataType = ZCL_UINT16;
     pReportCmd->attrList[3].attrData = (void *)(zclApp_CurrentValues.Voltage[1]); 
     
     pReportCmd->attrList[4].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT_PH_B;
     pReportCmd->attrList[4].dataType = ZCL_UINT16;
     pReportCmd->attrList[4].attrData = (void *)(zclApp_CurrentValues.Current[1]); 
     
     pReportCmd->attrList[5].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_PH_B;
     pReportCmd->attrList[5].dataType = ZCL_INT16;
     pReportCmd->attrList[5].attrData = (void *)(zclApp_CurrentValues.Power[1]); 
     
     pReportCmd->attrList[6].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_PH_C;
     pReportCmd->attrList[6].dataType = ZCL_UINT16;
     pReportCmd->attrList[6].attrData = (void *)(zclApp_CurrentValues.Voltage[0]); 
     
     pReportCmd->attrList[7].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT_PH_C;
     pReportCmd->attrList[7].dataType = ZCL_UINT16;
     pReportCmd->attrList[7].attrData = (void *)(zclApp_CurrentValues.Current[0]); 
     
     pReportCmd->attrList[8].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_PH_C;
     pReportCmd->attrList[8].dataType = ZCL_INT16;
     pReportCmd->attrList[8].attrData = (void *)(zclApp_CurrentValues.Power[0]); 
     
     
     zcl_SendReportCmd(1, dstAddr, ELECTRICAL, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, true, SeqNum++);
    }
    osal_mem_free(pReportCmd); 
    osal_mem_free(dstAddr);  
*/    
    bdb_RepChangedAttrValue(FIRST_ENDPOINT, ELECTRICAL, ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER);
    break;
  case 8:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_ENERGY_T1);
    break;
  case 9:
    Energy = (*mercury_3ph->ReadEnergy)(REQ_ENERGY_T1).Energy[1];
    if (Energy == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
    }
    else {
      zclApp_CurrentValues.Energy[1] = Energy;
      LREP("Energy 1 = %x\r\n", CurrentValues.Energy[1]);
    }
    break;
  case 10:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_ENERGY_T2);
    break;
  case 11:
    Energy = (*mercury_3ph->ReadEnergy)(REQ_ENERGY_T2).Energy[2];
    if (Energy == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
    }
    else {
      zclApp_CurrentValues.Energy[2] = Energy;
      LREP("Energy 2 = %x\r\n", CurrentValues.Energy[2]);
    }
    break;
  case 12:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_ENERGY_T3);
    break;
  case 13:
    Energy = (*mercury_3ph->ReadEnergy)(REQ_ENERGY_T3).Energy[3];
    if (Energy == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
    }
    else{
      zclApp_CurrentValues.Energy[3] = Energy;
      LREP("Energy 3 = %d\r\n", CurrentValues.Energy[3]);
    }
    break;
  case 14:
    (*mercury_3ph->RequestMeasure)(zclApp_Config.DeviceAddress, REQ_ENERGY_T4);
    break;
  case 15:
    Energy = (*mercury_3ph->ReadEnergy)(REQ_ENERGY_T4).Energy[4];
    if (Energy == METER_INVALID_RESPONSE) {
      LREPMaster("Invalid response from counter\r\n");
      break;
    }
    else {
      zclApp_CurrentValues.Energy[4] = Energy;
      LREP("Energy 4 = %x\r\n", CurrentValues.Energy[4]);
    }
    zclApp_CurrentValues.Energy[0] = zclApp_CurrentValues.Energy[1] + zclApp_CurrentValues.Energy[2] + zclApp_CurrentValues.Energy[3] + zclApp_CurrentValues.Energy[4];
    LREP("Energy SUM = %x\r\n", zclApp_CurrentValues.Energy[0]);
    bdb_RepChangedAttrValue(SECOND_ENDPOINT, SE_METERING, ATTRID_SE_METERING_CURR_SUMM_DLVD);
    break;
  case 16:
    (*mercury_3ph->StartStopData)(zclApp_Config.DeviceAddress, 0x02);
    break;
  case 17:
    if ((*mercury_3ph->CheckReady)()) 
      LREPMaster("Close OK\r\n");
    else 
      LREPMaster("Close FAIL\r\n");
    break;
  case 18:
    temp = readTemperature();
    if (temp == 1) {
      LREPMaster("ReadDS18B20 error\r\n");
      break;
    } else {
      zclApp_Temperature = temp;
      LREP("ReadDS18B20 t=%d\r\n", zclApp_Temperature);
    }
//    bdb_RepChangedAttrValue(FIRST_ENDPOINT, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
    break;
  default:
    HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    osal_stop_timerEx(zclApp_TaskID, APP_READ_SENSORS_EVT);
    osal_clear_event(zclApp_TaskID, APP_READ_SENSORS_EVT);
    currentSensorsReadingPhase = 0;
    break;

  }

}

static void zclApp_Report(void) 
{ 
  osal_start_reload_timer(zclApp_TaskID, APP_READ_SENSORS_EVT, 500); 
}

static void zclApp_BasicResetCB(void) {
    LREPMaster("BasicResetCB\r\n");
    zclApp_ResetAttributesToDefaultValues();
    zclApp_SaveAttributesToNV();
}

static ZStatus_t zclApp_ReadWriteAuthCB(afAddrType_t *srcAddr, zclAttrRec_t *pAttr, uint8 oper) {
    LREPMaster("AUTH CB called\r\n");
    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 2000);
    return ZSuccess;
}

static void zclApp_SaveAttributesToNV(void) {
  uint8 writeStatus = osal_nv_write(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
  LREP("Saving attributes to NV write=%d\r\n", writeStatus);
  osal_start_reload_timer(zclApp_TaskID, APP_REPORT_EVT, (uint32)zclApp_Config.MeasurementPeriod * (uint32)1000);
  if (OldDeviceModel !=  zclApp_Config.DeviceModel) {
    LREP("Change device model, old = %d. new = %d\r\n", OldDeviceModel, zclApp_Config.DeviceModel);
    zclApp_SetDeviceModel(zclApp_Config.DeviceModel);
    OldDeviceModel = zclApp_Config.DeviceModel;
  }
}

static void zclApp_RestoreAttributesFromNV(void) {
    uint8 status = osal_nv_item_init(NW_APP_CONFIG, sizeof(application_config_t), NULL);
    LREP("Restoring attributes from NV  status=%d \r\n", status);
    if (status == NV_ITEM_UNINIT) {
        uint8 writeStatus = osal_nv_write(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
        LREP("NV was empty, writing %d\r\n", writeStatus);
    }
    if (status == ZSUCCESS) {
        LREPMaster("Reading from NV\r\n");
        osal_nv_read(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
    }
}

static void zclApp_SetDeviceModel(uint8 DeviceModel) {
//  zclReportCmd_t *pReportCmd;

  switch(DeviceModel) {
  case DEV_MERCURY_1PH:
    zclApp_Config.VoltageDivisor = 10;
    zclApp_Config.CurrentDivisor = 100;
    zclApp_Config.PowerDivisor   = 1;
    zclApp_Config.EnergyDivisor  = 100;
    break;
  case DEV_MERCURY_3PH:
    zclApp_Config.VoltageDivisor = 100;
    zclApp_Config.CurrentDivisor = 1000;
    zclApp_Config.PowerDivisor   = 1;
    zclApp_Config.EnergyDivisor  = 1000;
    break;
  default:
    break;
  }
  zclApp_Config.VoltageMultiplier = 1;
  zclApp_Config.CurrentMultiplier = 1;
  zclApp_Config.PowerMultiplier   = 1;
  zclApp_Config.EnergyMultiplier  = 1;
  /*
  pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (6 * sizeof(zclReport_t)));

  afAddrType_t *dstAddr;  
  dstAddr = (afAddrType_t *)osal_mem_alloc( sizeof(afAddrType_t));
  dstAddr->addr.shortAddr = 0;
  dstAddr->addrMode = afAddr16Bit;
  dstAddr->endPoint = FIRST_ENDPOINT;

  if (pReportCmd != NULL) {
    pReportCmd->numAttr = 6;
    pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_AC_VOLTAGE_DIVISOR;
    pReportCmd->attrList[0].dataType = ZCL_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.VoltageDivisor);

    pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_AC_CURRENT_DIVISOR;
    pReportCmd->attrList[0].dataType = ZCL_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.CurrentDivisor);

    pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_AC_POWER_DIVISOR;
    pReportCmd->attrList[0].dataType = ZCL_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.PowerDivisor);

    pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_AC_VOLTAGE_MULTIPLIER;
    pReportCmd->attrList[0].dataType = ZCL_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.VoltageMultiplier);

    pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_AC_CURRENT_MULTIPLIER;
    pReportCmd->attrList[0].dataType = ZCL_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.CurrentMultiplier);

    pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_AC_POWER_MULTIPLIER;
    pReportCmd->attrList[0].dataType = ZCL_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.PowerMultiplier);

    zcl_SendReportCmd(FIRST_ENDPOINT, dstAddr, ELECTRICAL, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, true, SeqNum++);
  }
  osal_mem_free(pReportCmd);
  osal_mem_free(dstAddr);  
*/
}

/****************************************************************************
****************************************************************************/
