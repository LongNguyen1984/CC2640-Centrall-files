/******************************************************************************

 @file  simple_central.c

 @brief This file contains the Simple Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include "bcomdef.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "central.h"
#include "simple_gatt_profile.h"

#include "board_key.h"
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SBC_STATE_CHANGE_EVT                  0x0001
#define SBC_KEY_CHANGE_EVT                    0x0002
#define SBC_RSSI_READ_EVT                     0x0004
#define SBC_PAIRING_STATE_EVT                 0x0008
#define SBC_PASSCODE_NEEDED_EVT               0x0010

// Simple Central Task Events
#define SBC_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBC_START_DISCOVERY_EVT               Event_Id_00
#define SBC_CONN_EST_TIMEOUT_EVT              Event_Id_01

#define SBC_ALL_EVENTS                        (SBC_ICALL_EVT            | \
                                               SBC_QUEUE_EVT            | \
                                               SBC_CONN_EST_TIMEOUT_EVT | \
                                               SBC_START_DISCOVERY_EVT)

// Enable/Disable Unlimited Scanning Feature
#define ENABLE_UNLIMITED_SCAN_RES             FALSE

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE//TRUE

    #define DEFAULT_SCAN_INTERVAL                 240 //(240 * 0.625) = 150 ms
    // Scan window in units of 0.625 ms
    #define DEFAULT_SCAN_WINDOW                   240 //(240 * 0.625) = 150 ms

// Set desired policy to use during discovery (use values from GAP_Disc_Filter_Policies)
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE//GAP_DISC_FILTER_POLICY_ALL

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// After the connection is formed, the central will accept connection parameter
// update requests from the peripheral
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE//TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        31//15

// Connection esablishement timeout
#define DEFAULT_CONN_SETUP_TIMEOUT            5000

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define SBC_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define SBC_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define SBC_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // Display_DISABLE_ALL
  #define SBC_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// Key states for connections
typedef enum {
  GATT_RW,                 // Perform GATT Read/Write
  RSSI,                    // Toggle RSSI updates
  CONN_UPDATE,             // Send Connection Parameter Update
  GET_CONN_INFO,           // Display Current Connection Information
  DISCONNECT               // Disconnect
} keyPressConnOpt_t;

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Clock object used to establishement connection timeout
static Clock_Struct startLinkEstClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Central";

// Number of scan results and scan result index
static uint8_t scanRes = 0;
static int8_t scanIdx = -1;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

// Key option state.
static keyPressConnOpt_t keyPressConnOpt = DISCONNECT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleCentral_init(void);
static void SimpleCentral_taskFxn(UArg a0, UArg a1);

static void SimpleCentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleCentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleCentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleCentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleCentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleCentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleCentral_startDiscovery(void);
static void SimpleCentral_stopEstablishing(void);

static bool SimpleCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleCentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleCentral_processPairState(uint8_t state, uint8_t status);
static void SimpleCentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleCentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static bStatus_t SimpleCentral_StartRssi(uint16_t connHandle, uint16_t period);
static bStatus_t SimpleCentral_CancelRssi(uint16_t connHandle);
static readRssi_t *SimpleCentral_RssiAlloc(uint16_t connHandle);
static readRssi_t *SimpleCentral_RssiFind(uint16_t connHandle);
static void SimpleCentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleCentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleCentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison);
static void SimpleCentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SimpleCentral_startDiscHandler(UArg a0);
void SimpleCentral_linkEstClockHandler(UArg a0);
void SimpleCentral_keyChangeHandler(uint8 keys);
void SimpleCentral_readRssiHandler(UArg a0);

static uint8_t SimpleCentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

#ifdef FPGA_AUTO_CONNECT
static void SimpleCentral_startGapDiscovery(void);
static void SimpleCentral_connectToFirstDevice(void);
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

const char *AdvTypeStrings[] = {"Connectable undirected","Connectable directed", "Scannable undirected", "Non-connectable undirected", "Scan response"};
// ...
char *Util_convertBytes2Str(uint8_t *pData, uint8_t length)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[(3*31)+1];
  char        *pStr = str;

  //*pStr++ = '0';
  //*pStr++ = 'x';

  for (charCnt = 0; charCnt < length; charCnt++)
  {
    *pStr++ = hex[*pData >> 4];
    *pStr++ = hex[*pData++ & 0x0F];
    *pStr++ = ':';
  }
  pStr = NULL;

  return str;
}
// ...
/*********************************************************************
 * PROFILE CALLBACKS
 */

// Central GAPRole Callbacks
static gapCentralRoleCB_t SimpleCentral_roleCB =
{
  SimpleCentral_eventCB     // GAPRole Event Callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleCentral_bondCB =
{
  SimpleCentral_passcodeCB, // Passcode callback
  SimpleCentral_pairStateCB // Pairing / Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#ifdef FPGA_AUTO_CONNECT
/*********************************************************************
 * @fn      SimpleCentral_startGapDiscovery
 *
 * @brief   Start discovering devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleCentral_startGapDiscovery(void)
{
  // Start discovery
  if ((state != BLE_STATE_CONNECTED) && (!scanningStarted))
  {
    scanningStarted = TRUE;
    scanRes = 0;

    Display_print0(dispHandle, 2, 0, "Discovering...");
    Display_clearLines(dispHandle, 3, 4);

    GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                  DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                  DEFAULT_DISCOVERY_WHITE_LIST);
  }
}

/*********************************************************************
 * @fn      SimpleCentral_connectToFirstDevice
 *
 * @brief   Connect to first device in list of discovered devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleCentral_connectToFirstDevice(void)
{
  uint8_t addrType;
  uint8_t *peerAddr;

  scanIdx = 0;

  if (state == BLE_STATE_IDLE)
  {
    // connect to current device in scan result
    peerAddr = devList[scanIdx].addr;
    addrType = devList[scanIdx].addrType;

    state = BLE_STATE_CONNECTING;

    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, peerAddr);

    Display_print0(dispHandle, 2, 0, "Connecting");
    Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
    Display_clearLine(dispHandle, 4);
  }
}
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * @fn      SimpleCentral_createTask
 *
 * @brief   Task creation function for the Simple Central.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleCentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleCentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleCentral_Init
 *
 * @brief   Initialization function for the Simple Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleCentral_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);


  static uint8 bdAddressPeer[6] = {0x84,0x86,0x77,0xAB,0x78,0xCC};//{0x90,0x78,0x56,0x34,0x12,0x00};
  HCI_LE_AddWhiteListCmd(ADDRMODE_PUBLIC, bdAddressPeer);

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleCentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  Util_constructClock(&startLinkEstClock, SimpleCentral_linkEstClockHandler,
                      DEFAULT_CONN_SETUP_TIMEOUT, 0, false, 0);

  Board_initKeys(SimpleCentral_keyChangeHandler);

  dispHandle = Display_open(SBC_DISPLAY_TYPE, NULL);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }

  // Setup the Central GAPRole Profile. For more information see the GAP section
  // in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/

  {
    uint8_t scanRes = 0;

    // In case that the Unlimited Scanning feature is disabled
    // send the number of scan results to the GAP
    if(ENABLE_UNLIMITED_SCAN_RES == FALSE)
    {
        scanRes = DEFAULT_MAX_SCAN_RES;
    }

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Set GAP Parameters to set the discovery duration
  // For more information, see the GAP section of the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);

  GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INTERVAL); //period for one scan channel

  GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WINDOW); //active scanning time within interval
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Don't send a pairing request after connecting; the device waits for the
    // application to start pairing
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    // Do not use authenticated pairing
    uint8_t mitm = DEFAULT_MITM_MODE;
    // This is a display only device
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    // Create a bond during the pairing process
    uint8_t bonding = DEFAULT_BONDING_MODE;
    // Whether to replace the least recently used entry when bond list is full,
    // and a new device is bonded.
    // Alternative is pairing succeeds but bonding fails, unless application has
    // manually erased at least one bond.
    uint8_t replaceBonds = FALSE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_LRU_BOND_REPLACEMENT, sizeof(uint8_t), &replaceBonds);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleCentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleCentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Set default values for Data Length Extension
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
    //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  Display_print0(dispHandle, 0, 0, "BLE Central");
}

/*********************************************************************
 * @fn      SimpleCentral_taskFxn
 *
 * @brief   Application task entry point for the Simple Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleCentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleCentral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SBC_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleCentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & SBC_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message
            SimpleCentral_processAppMsg(pMsg);

            // Free the space from the message
            ICall_free(pMsg);
          }
        }
      }

      if (events & SBC_START_DISCOVERY_EVT)
      {
        SimpleCentral_startDiscovery();
      }

      if (events & SBC_CONN_EST_TIMEOUT_EVT)
      {
        SimpleCentral_stopEstablishing();
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleCentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleCentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleCentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleCentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleCentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleCentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SimpleCentral_handleKeys(0, pMsg->hdr.state);
      break;

    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleCentral_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SimpleCentral_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleCentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");

        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");

#ifdef FPGA_AUTO_CONNECT
        SimpleCentral_startGapDiscovery();
#endif // FPGA_AUTO_CONNECT
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        uint8 bAddDevice = FALSE;

        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
            if (SimpleCentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                          pEvent->deviceInfo.pEvtData,
                                          pEvent->deviceInfo.dataLen))
            {
                bAddDevice = TRUE;
            }
        }

        if((ENABLE_UNLIMITED_SCAN_RES == TRUE) && (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE))
        {
            bAddDevice = TRUE;
        }

        if(bAddDevice)
        {
            SimpleCentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                        pEvent->deviceInfo.addrType);
        }

        // === SOLUTION [Print scan results] ===
                    //Print scan response data or advertising data
                    if(pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP)
                    {
                        Display_print1(dispHandle, 4, 0, "ScanResponseAddr: %s", Util_convertBdAddr2Str(pEvent->deviceInfo.addr));
                        Display_print1(dispHandle, 5, 0, "ScanResponseData: %s", Util_convertBytes2Str(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen));
                    }
                    else
                    {
                          Display_print3(dispHandle, 6, 0, "Advertising Addr: %s RSSI: %d Advertising Type: %s", Util_convertBdAddr2Str(pEvent->deviceInfo.addr), pEvent->deviceInfo.rssi, AdvTypeStrings[pEvent->deviceInfo.eventType]);

                          Display_print1(dispHandle, 7, 0, "Advertising Data: %s", Util_convertBytes2Str(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen));
                    }

        // ==== END SOLUTION ====
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        if(pEvent->gap.hdr.status == SUCCESS)
        {
            // discovery complete
            scanningStarted = FALSE;

            // if not filtering device discovery results based on service UUID
            if ((DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE) && (ENABLE_UNLIMITED_SCAN_RES == FALSE))
            {
              // Copy results
              scanRes = pEvent->discCmpl.numDevs;
              memcpy(devList, pEvent->discCmpl.pDevList,
                     (sizeof(gapDevRec_t) * scanRes));
            }

            Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);

            if (scanRes > 0)
            {
#ifndef FPGA_AUTO_CONNECT
              Display_print0(dispHandle, 3, 0, "<- To Select");
            }

            // Initialize scan index.
            scanIdx = -1;

            // Prompt user that re-performing scanning at this state is possible.
            Display_print0(dispHandle, 5, 0, "Discover ->");

#else // FPGA_AUTO_CONNECT
              SimpleCentral_connectToFirstDevice();
            }
#endif // FPGA_AUTO_CONNECT
          }
        else
        {
            if(pEvent->gap.hdr.status == GAP_LLERROR_INVALID_PARAMETERS)
            {
                Display_print0(dispHandle, 3, 0, "INVALID PARAMETERS");
            }
            else if(pEvent->gap.hdr.status == GAP_LLERROR_COMMAND_DISALLOWED)
            {
                Display_print0(dispHandle, 3, 0, "COMMAND DISALLOWED");
            }
            else
            {
                Display_print0(dispHandle, 3, 0, "ERROR");
            }
        }
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        Util_stopClock(&startLinkEstClock);
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;
          procedureInProgress = TRUE;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
          HCI_LE_ReadRemoteUsedFeaturesCmd(connHandle);
          // Display the initial options for a Right key press.
          SimpleCentral_handleKeys(0, KEY_LEFT);
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;
        procedureInProgress = FALSE;
        keyPressConnOpt = DISCONNECT;
        scanIdx = -1;

        // Cancel RSSI reads
        SimpleCentral_CancelRssi(pEvent->linkTerminate.connectionHandle);

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);
        Display_clearLine(dispHandle, 6);

        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleCentral_handleKeys(uint8_t shift, uint8_t keys)
{
  hciActiveConnInfo_t *pConnInfo; // pointer to hold return connection information
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // If not connected
    if (state == BLE_STATE_IDLE)
    {
      // If not currently scanning
      if (!scanningStarted)
      {
        // Increment index of current result.
        scanIdx++;

        // If there are no scanned devices
        if (scanIdx >= scanRes)
        {
          // Prompt the user to begin scanning again.
          scanIdx = -1;
          Display_print0(dispHandle, 2, 0, "");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 5, 0, "Discover ->");
        }
        else
        {
          // Display the indexed scanned device.
          Display_print1(dispHandle, 2, 0, "Device %d", (scanIdx + 1));
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
          Display_print0(dispHandle, 5, 0, "Connect ->");
          Display_print0(dispHandle, 6, 0, "<- Next Option");
        }
      }
    }
    else if (state == BLE_STATE_CONNECTED)
    {
      keyPressConnOpt = (keyPressConnOpt == DISCONNECT) ? GATT_RW :
                                          (keyPressConnOpt_t) (keyPressConnOpt + 1);

      //clear excess lines to keep display clean if another option chosen
      Display_doClearLines(dispHandle, 7, 16);

      switch (keyPressConnOpt)
      {
        case GATT_RW:
          Display_print0(dispHandle, 5, 0, "GATT Read/Write ->");
          break;

        case RSSI:
          Display_print0(dispHandle, 5, 0, "Toggle Read RSSI ->");
          break;

        case CONN_UPDATE:
          Display_print0(dispHandle, 5, 0, "Connection Update ->");
          break;

        case GET_CONN_INFO:
          Display_print0(dispHandle, 5, 0, "Connection Info ->");
          break;

        case DISCONNECT:
          Display_print0(dispHandle, 5, 0, "Disconnect ->");
          break;

        default:
          break;
      }

      Display_print0(dispHandle, 6, 0, "<- Next Option");
    }

    return;
  }

  if (keys & KEY_RIGHT)
  {
    if (state == BLE_STATE_IDLE)
    {
      if (scanIdx == -1)
      {
        if (!scanningStarted)
        {
          scanningStarted = TRUE;
          scanRes = 0;

          Display_print0(dispHandle, 2, 0, "Discovering...");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 4, 0, "");
          Display_print0(dispHandle, 5, 0, "");
          Display_print0(dispHandle, 6, 0, "");

          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST);
        }
      }
      // Connect if there is a scan result
      else
      {
        // connect to current device in scan result
        uint8_t *peerAddr = devList[scanIdx].addr;
        uint8_t addrType = devList[scanIdx].addrType;

        state = BLE_STATE_CONNECTING;

        // Set a timelimit on the connection establishement
        Util_startClock(&startLinkEstClock);

        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                     DEFAULT_LINK_WHITE_LIST,
                                     addrType, peerAddr);

        Display_print0(dispHandle, 2, 0, "Connecting");
        Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
        Display_clearLine(dispHandle, 4);

        // Forget the scan results.
        scanRes = 0;
        scanIdx = -1;
      }
    }
    else if (state == BLE_STATE_CONNECTED)
    {
      switch (keyPressConnOpt)
      {
        case GATT_RW:
          if (charHdl != 0 &&
              procedureInProgress == FALSE)
          {
            uint8_t status;

            // Do a read or write as long as no other read or write is in progress
            if (doWrite)
            {
              // Do a write
              attWriteReq_t req;

              req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
              if ( req.pValue != NULL )
              {
                req.handle = charHdl;
                req.len = 1;
                req.pValue[0] = charVal;
                req.sig = 0;
                req.cmd = 0;

                status = GATT_WriteCharValue(connHandle, &req, selfEntity);
                if ( status != SUCCESS )
                {
                  GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
                }
              }
              else
              {
                status = bleMemAllocError;
              }
            }
            else
            {
              // Do a read
              attReadReq_t req;

              req.handle = charHdl;
              status = GATT_ReadCharValue(connHandle, &req, selfEntity);
            }

            if (status == SUCCESS)
            {
              procedureInProgress = TRUE;
              doWrite = !doWrite;
            }
          }
          break;

        case RSSI:
          // Start or cancel RSSI polling
          if (SimpleCentral_RssiFind(connHandle) == NULL)
          {
            SimpleCentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
          }
          else
          {
            SimpleCentral_CancelRssi(connHandle);

            Display_print0(dispHandle, 4, 0, "RSSI Cancelled");
          }
          break;

        case CONN_UPDATE:
           // Connection update
          GAPCentralRole_UpdateLink(connHandle,
                                    DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                    DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                    DEFAULT_UPDATE_SLAVE_LATENCY,
                                    DEFAULT_UPDATE_CONN_TIMEOUT);
          break;

        case GET_CONN_INFO:
          pConnInfo= ICall_malloc(sizeof(hciActiveConnInfo_t));

          if (pConnInfo != NULL)
          {
            // This is hard coded to assume we want connection info for a single
            // valid connection as is the normal use case for simple central.
            // A full featured application may chose to use HCI_EXT_GetConnInfoCmd()
            // to obtain a full list of all active connections and their connId's
            // to retrive more specific conneciton information if more than one
            // valid connectin is expected to exist.
            HCI_EXT_GetActiveConnInfoCmd(0, pConnInfo);
            Display_print1(dispHandle, 7, 0, "AccessAddress: 0x%x", pConnInfo->accessAddr);
            Display_print1(dispHandle, 8, 0, "Connection Interval: %d", pConnInfo->connInterval);
            Display_print3(dispHandle, 9, 0, "HopVal:%d, nxtCh:%d, mSCA:%d", \
                           pConnInfo->hopValue, pConnInfo->nextChan, \
                           pConnInfo->mSCA);
            Display_print5(dispHandle, 10, 0, "ChanMap: \"%x:%x:%x:%x:%x\"",\
                           pConnInfo->chanMap[4], pConnInfo->chanMap[3],\
                           pConnInfo->chanMap[2], pConnInfo->chanMap[1],\
                           pConnInfo->chanMap[0]);

            ICall_free(pConnInfo);
          }
          else
          {
            Display_print0(dispHandle, 4, 0, "ERROR: Failed to allocate memory for return connection information");
          }
          break;

        case DISCONNECT:
          state = BLE_STATE_DISCONNECTING;

          GAPCentralRole_TerminateLink(connHandle);

          Display_print0(dispHandle, 2, 0, "Disconnecting");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 4, 0, "");
          Display_print0(dispHandle, 5, 0, "");

          keyPressConnOpt = GATT_RW;
          break;

        default:
          break;
      }
    }

    return;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleCentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

      procedureInProgress = FALSE;
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal++);
      }

      procedureInProgress = FALSE;

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleCentral_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleCentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
#ifndef Display_DISABLE_ALL
        int8 rssi = (int8)pMsg->pReturnParam[3];

        Display_print1(dispHandle, 4, 0, "RSSI dB: %d", (uint32_t)(rssi));
#endif
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static bStatus_t SimpleCentral_StartRssi(uint16_t connHandle, uint16_t period)
{
  readRssi_t *pRssi;

  // Verify link is up
  if (!linkDB_Up(connHandle))
  {
    return bleIncorrectMode;
  }

  // If already allocated
  if ((pRssi = SimpleCentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    pRssi->period = period;
  }
  // Allocate structure
  else if ((pRssi = SimpleCentral_RssiAlloc(connHandle)) != NULL)
  {
    pRssi->period = period;
  }
  // Allocate failed
  else
  {
    return bleNoResources;
  }

  // Start timer
  Util_restartClock(pRssi->pClock, period);

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleCentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static bStatus_t SimpleCentral_CancelRssi(uint16_t connHandle)
{
  readRssi_t *pRssi;

  if ((pRssi = SimpleCentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    // Free RSSI structure
    SimpleCentral_RssiFree(connHandle);

    return SUCCESS;
  }

  // Not found
  return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiAlloc
 *
 * @brief   Allocate an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if allocation failed.
 */
static readRssi_t *SimpleCentral_RssiAlloc(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
    {
      readRssi_t *pRssi = &readRssi[i];

      pRssi->pClock = (Clock_Struct *)ICall_malloc(sizeof(Clock_Struct));
      if (pRssi->pClock)
      {
        Util_constructClock(pRssi->pClock, SimpleCentral_readRssiHandler,
                            0, 0, false, i);
        pRssi->connHandle = connHandle;

        return pRssi;
      }
    }
  }

  // No free structure found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
static readRssi_t *SimpleCentral_RssiFind(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      return &readRssi[i];
    }
  }

  // Not found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void SimpleCentral_RssiFree(uint16_t connHandle)
{
  uint8_t i;

  // Find RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      readRssi_t *pRssi = &readRssi[i];
      if (pRssi->pClock)
      {
        Clock_destruct(pRssi->pClock);

        // Free clock struct
        ICall_free(pRssi->pClock);
        pRssi->pClock = NULL;
      }

      pRssi->connHandle = GAP_CONNHANDLE_ALL;
      break;
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleCentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleCentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleCentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleCentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleCentral_stopEstablishing
 *
 * @brief   Stop Connection Establishement.
 *
 * @return  none
 */
static void SimpleCentral_stopEstablishing(void)
{
  GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);
  Display_print0(dispHandle, 4, 0, "Conn. Establishement Timeout");
  state = BLE_STATE_IDLE;
}

/*********************************************************************
 * @fn      SimpleCentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleCentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      // Just in case we're using the default MTU size (23 octets)
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", ATT_MTU_SIZE);

      discState = BLE_DISC_STATE_SVC;

      // Discovery simple service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_DiscCharsByUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                             pMsg->msg.readByTypeRsp.pDataList[4]);

      Display_print0(dispHandle, 2, 0, "Simple Svc Found");
      procedureInProgress = FALSE;
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  if (dataLen > 0)
  {
    pEnd = pData + dataLen - 1;

    // While end of data not reached
    while (pData < pEnd)
    {
      // Get length of next AD item
      adLen = *pData++;
      if (adLen > 0)
      {
        adType = *pData;

        // If AD type is for 16-bit service UUID
        if ((adType == GAP_ADTYPE_16BIT_MORE) ||
            (adType == GAP_ADTYPE_16BIT_COMPLETE))
        {
          pData++;
          adLen--;

          // For each UUID in list
          while (adLen >= 2 && pData < pEnd)
          {
            // Check for match
            if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
            {
              // Match found
              return TRUE;
            }

            // Go to next
            pData += 2;
            adLen -= 2;
          }

          // Handle possible erroneous extra byte in UUID list
          if (adLen == 1)
          {
            pData++;
          }
        }
        else
        {
          // Go to next item
          pData += adLen;
        }
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleCentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleCentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleCentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleCentral_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleCentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleCentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleCentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleCentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleCentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleCentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleCentral_linkEstClockHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleCentral_linkEstClockHandler(UArg a0)
{
  Event_post(syncEvent, SBC_CONN_EST_TIMEOUT_EVT);
}

/*********************************************************************
 * @fn      SimpleCentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleCentral_startDiscHandler(UArg a0)
{
  Event_post(syncEvent, SBC_START_DISCOVERY_EVT);
}

/*********************************************************************
 * @fn      SimpleCentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleCentral_keyChangeHandler(uint8 keys)
{
  SimpleCentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleCentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleCentral_readRssiHandler(UArg a0)
{
  SimpleCentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleCentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleCentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
*********************************************************************/
