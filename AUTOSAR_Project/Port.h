 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: mohamed tarek
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Dio Module Id */
#define PORT_MODULE_ID    (120U)

/* Dio Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)
   
   
 /*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)
   
/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif
   
/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif
 
   
/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
*                      API Service Id Macros                                 *
******************************************************************************/
/* Service ID for PORT init */
#define PORT_INIT_SID           (uint8)0x00
   
/* Service ID for PORT set pin direction */
#define PORT_SET_UP_PIN_DIRECTION_SID (uint8)0x01
 
/* Service ID for PORT refresh port direction */
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02
   
/* Service ID for PORT get version info */
#define PORT_GET_VERSION_INFO_SID (uint8)0x03
   
/* Service ID for PORT set pin Mode */
#define PORT_SET_PIN_MODE_SID (uint8)0x04
   
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
   
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN (uint8)0x0A
   
/* DET code to report Port Pin not configured as changeable*/
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B
   
/* DET Code to report API Port_Init service called with wrong parameter. */
#define PORT_E_PARAM_CONFIG (uint8)0x0C
   
/*DET code to report an error when mode is unchangeable. */
#define PORT_E_PARAM_INVALID_MODE (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE (uint8)0x0E
   
/*DET code to report an error if API service called without module initialization */
#define PORT_E_UNINIT (uint8)0x0F
   
/*DET code to report an error if API service called with NULL Pointer */
#define PORT_E_PARAM_POINTER (uint8)0x10



/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
   
 

/* The type Port_PinType shall be used for the symbolic name of a Port Pin.*/
typedef uint8 Port_PinType;

/* Description: Enum to state the mode of PIN */
/* PORT_PIN_MODE_0 --> DIO*/
/* PORT_PIN_MODE_1 --> UART*/
/* PORT_PIN_MODE_2 --> SSI*/
/* PORT_PIN_MODE_3 --> I2C/CAN*/
/* PORT_PIN_MODE_4 --> PWM/FAULT*/
/* PORT_PIN_MODE_5 --> PWM/FAULT*/
/* PORT_PIN_MODE_6 --> ID/PhA/PhB*/
/* PORT_PIN_MODE_7 --> TCCP/WTCCP*/
/* PORT_PIN_MODE_8 --> CANRX/UTS/USB/NMI*/
/* PORT_PIN_MODE_9 --> C0o/C1o*/
/* PORT_PIN_MODE_14 --> TRD*/

typedef enum
{
  PORT_PIN_MODE_ADC,
  PORT_PIN_MODE_0,
  PORT_PIN_MODE_1,
  PORT_PIN_MODE_2,
  PORT_PIN_MODE_3,
  PORT_PIN_MODE_4,
  PORT_PIN_MODE_5,
  PORT_PIN_MODE_6,
  PORT_PIN_MODE_7,
  PORT_PIN_MODE_8,
  PORT_PIN_MODE_9,
  PORT_PIN_MODE_14
}Port_PinMode;

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;


/* The type Port_PinModeType shall be used with the function call Port_SetPinMode*/
typedef uint8 Port_PinModeType ;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to state the possibility of changing the direction of PIN */
typedef enum
{
    PIN_DIRECTION_UNCHANGEABLE,PIN_DIRECTION_CHANGEABLE
}Port_PinDirectionChangeable;

/* Description: Enum to state the possibility of changing the mode of PIN */
typedef enum
{
    PIN_MODE_UNCHANGEABLE,PIN_MODE_CHANGEABLE
}Port_PinModeChangeable;



/* Description: Enum to state the initial value of PIN */
/*
  PORT_PIN_LEVEL_LOW--> Pin=0
  PORT_PIN_LEVEL_HIGH--> Pin=1
*/
typedef enum
{
  PORT_PIN_LEVEL_LOW,
  PORT_PIN_LEVEL_HIGH
}Port_PinValue;

  /* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 *      5. Possibility of changing pin Direction--> OFF , ON
 *      6. Possibility of changing pin Mode--> OFF , ON
 *      7. Pin Mode --> MODE_ADC,MODE_0,MODE_1,MODE_2,MODE_3,MODE_4,MODE_5,MODE_6
 *      MODE_7,MODE_8,MODE_9,MODE_14
 */


typedef struct 
{
    uint8 port_num; 
    uint8 pin_num; 
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    Port_PinValue initial_value;
    Port_PinDirectionChangeable change_direction;
    Port_PinModeChangeable change_mode;
    Port_PinMode pin_initial_mode;
}Port_ConfigPin;


typedef struct
{
  Port_ConfigPin pin[PORT_CONFIGURED_PINS];
}Port_ConfigType;


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_init
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_init(const Port_ConfigType *ConfigPtr );

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin-->Port Pin Number ID - Direction-->Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction);
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy:reentrant
* Parameters (in): Pin-->Port Pin ID Number - Mode-->New Port Pin mode to be set on port pin. 
* Parameters (inout): None
* Parameters (out): VersionInfo--> Pointer to where to store the version information of this module
* Return value: None
* Description: Sets the port pin mode.  
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction. 
************************************************************************************/
#if (PORT_REFRESH_PORT_API == STD_ON)
  void Port_RefreshPortDirection( void );
#endif

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None 
* Parameters (inout): None
* Parameters (out): VersionInfo--> Pointer to where to store the version information of this module
* Return value: None
* Description: Returns the version information of this module 
************************************************************************************/

#if (PORT_VERSION_INFO_API == STD_ON)
  void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );
#endif

/* Post build structure used with Port_Init API */
extern const Port_ConfigType Port_Configuration;


#endif /* PORT_H */
