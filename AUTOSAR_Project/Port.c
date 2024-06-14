 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
#include "Port_Regs.h"
#include "Std_Types.h"
#include "Common_Macros.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigPin * pin = NULL_PTR;

STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;





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
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_Init( const Port_ConfigType* ConfigPtr )
{
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
	}
	else
#endif
        {
          /*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status  = PORT_INITIALIZED;
               /* Port_ConfigPtr = ConfigPtr ; */
                /*Address of the first pin structure */
		pin = ConfigPtr->pin;
                
		/* for loop for looping on all pins */
		for (Port_PinType Pin=0;Pin<PORT_CONFIGURED_PINS;Pin++){
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
   
    
    switch(pin[Pin].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1<<pin[Pin].port_num);
    delay = SYSCTL_REGCGC2_REG;
    
    /*Locking pins so we need to unlock then commit */
    if( ((pin[Pin].port_num == PORT_PortD) && (pin[Pin].pin_num == PORT_Pin7 )) || ((pin[Pin].port_num  == PORT_PortF) && (pin[Pin].pin_num == PORT_Pin0  )) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , pin[Pin].pin_num );  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (pin[Pin].port_num == PORT_PortC) && (pin[Pin].pin_num<= PORT_Pin3  ) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
     /*Check pin direction is output or input  */
        if(pin[Pin].direction == PORT_PIN_OUT)
    {
	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , pin[Pin].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(pin[Pin].initial_value == PORT_PIN_LEVEL_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , pin[Pin].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , pin[Pin].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(pin[Pin].direction == PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , pin[Pin].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(pin[Pin].resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , pin[Pin].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(pin[Pin].resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , pin[Pin].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , pin[Pin].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , pin[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }
    




    
    /* pin mode */
    
    if (pin[Pin].pin_initial_mode == PORT_PIN_MODE_0)
		{
			/* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , pin[Pin].pin_num);

			/* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin[Pin].pin_num);

			/* Clear the PMCx bits for this pin */
			*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin[Pin].pin_num * 4));

			/* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) ,pin[Pin].pin_num);
		}
            /*Enable analog for analog mode*/
		else if (pin[Pin].pin_initial_mode  == PORT_PIN_MODE_ADC)
		{
			/* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , pin[Pin].pin_num);

			/* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin[Pin].pin_num);

			/* Clear the PMCx bits for this pin */
			*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin[Pin].pin_num * 4));

			/* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , pin[Pin].pin_num);
		}
     /*Enable digital for all digital modes */
		else 
		{
			/* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , pin[Pin].pin_num);

			/* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin[Pin].pin_num);

			/* Set the PMCx bits for this pin */
			*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)pin[Pin].pin_initial_mode & 0x0000000F << (pin[Pin].pin_num * 4));

			/* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , pin[Pin].pin_num);
		}
                }
    
}
}




/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): - Pin - Port Pin ID number
* Direction -Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if (PORT_PIN_DIRECTION_CHANGEABLE_API == STD_ON)

void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction)
{

  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  volatile uint32 delay = 0;
	#if (PORT_DEV_ERROR_DETECT == STD_ON)
		/* Check if the Driver is initialized before using this function */
		if(Port_Status == PORT_NOT_INITIALIZED)
		{
			Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_UNINIT);
                    
		}
		else
		{	
                  /* Do Nothing */	
                }

		/* check if incorrect Port Pin ID passed */
		if(Pin >= PORT_CONFIGURED_PINS)
		{
			Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_PARAM_PIN);
                    
		}
		else
		{	/* Do Nothing */	}
                
                if(Pin >= PORT_CONFIGURED_CHANNELS)
		{
			Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_PARAM_PIN);
		}
		else
		{	/* Do Nothing */	}

		/* check if Port Pin not configured as changeable */
		if(Port_Port_Pins[Pin].Port_PinChangeableDirection == STD_OFF)
		{
			Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_DIRECTION_UNCHANGEABLE);
		}
		else
		{	
                  /* Do Nothing */	
                }
	#endif



    
   
    
    switch(pin[Pin].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    if(!((pin[Pin].port_num == PORT_PortC) && (Pin <=  PORT_Pin3)) ) /* PC0 to PC3 */
    {

      if(Direction == PORT_PIN_OUT)
      {
    	 /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , pin[Pin].pin_num);
      }
      else if(Direction == PORT_PIN_IN)
      {
    	/* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr+ PORT_DIR_REG_OFFSET) , pin[Pin].pin_num);
        
      }
      else 
      {
         /* Do Nothing */
      }
    }
    else
    {
        /* Do Nothing */
    }
    
  }
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

#if ( PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if the Driver is initialized before using this function */
        if(Port_Status == PORT_NOT_INITIALIZED)
        {			
          Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_UP_PIN_DIRECTION_SID, PORT_E_UNINIT);
        }
        else
	{	
          /* Do Nothing */	
        }

	/*if  Port Pin ID passed is greater than the number stored in PORT_CONFIGURED_PINS , Report a det Error */
	if(Pin >= PORT_CONFIGURED_PINS)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_UP_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
	}
	else
	{	/* Do Nothing */	}

	/* check if Port Pin not configured as changeable */
	if(Port_ConfigPtr->pin[Pin].change_mode == PIN_MODE_UNCHANGEABLE)
	{
	   Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_API, PORT_E_MODE_UNCHANGEABLE);
	}
	else
	{	/* Do Nothing */	}
        
        
        /* Check if invalid Mode is passed to the function */
        if(Mode > PORT_PIN_MODE_14)
        {
          Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_API, PORT_E_PARAM_INVALID_MODE);
        }
        else
        {
          /* Do Nothing */
        }
   #endif
  
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* Poiner to point to the required Port Registers base address */
        

	switch(Port_ConfigPtr->pin[Pin].port_num)
	{
		case  0: 
                  PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		  break;
		case  1: 
                  PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		  break;
		case  2: 
                  PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		  break;
		case  3: 
                  PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		  break;
		case  4: 
                  PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		  break;
		case  5: 
                  PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		  break;
	}
        if( (Port_ConfigPtr->pin[Pin].port_num == PORT_PortC) && (Port_ConfigPtr->pin[Pin].pin_num <= PORT_Pin3) ) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
            return ;
        }
        
        
        if (Mode == PORT_PIN_MODE_0)
	{
		/* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);

		/* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);

		/* Clear the PMCx bits for this pin */
		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr->pin[Pin].pin_num * 4));

		/* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);
	}
        /*Enable analog for analog mode*/
	else if (Mode == PORT_PIN_MODE_ADC)
	{
		/* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);

		/* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);

		/* Clear the PMCx bits for this pin */
		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_ConfigPtr->pin[Pin].pin_num * 4));

		/* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);
	}
        /*Enable digital for all digital modes */
	else 
	{
		/* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);

		/* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);

		/* Set the PMCx bits for this pin */
		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Mode & 0x0000000F << (Port_ConfigPtr->pin[Pin].pin_num * 4));

		/* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr->pin[Pin].pin_num);
	}
}
#endif



/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in):None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description:Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void){
  
  /* point to the required Port Registers base address */
  volatile uint32 *PortGpio_Ptr = NULL_PTR;
  
  /*Variable used as an Index for the Port_Pin structures*/
  volatile uint8 Pin=0;
  
    /*Det Error */
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the input configuration pointer is not a NULL_PTR */
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIR_SID,
                    PORT_E_UNINIT);
  }
  else 
  {
     /* Do Nothing */
  }
  
#endif

  /*PORT061:  The function Port_RefreshPortDirection shall exclude those port 
            pins from refreshing that are configured as ‘pin direction changeable during runtime‘. */
        /* PORT060: The function Port_RefreshPortDirection shall refresh the direction 
         of all configured ports to the configured direction (PortPinDirection).*/
        /* Refreshing pin means : If pin is input --> make it input ******** if pin is output --> make it output */
        /* Loop on all the pins in all ports*/
  for(Port_PinType Pin = 0;Pin < PORT_CONFIGURED_PINS; Pin++)
  {

    switch(pin[Pin].port_num )
    {
    	case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    	break;
    	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    	break;
    	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    	break;
    	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    	break;
    	case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    	break;
    	case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    	break;
    }



    if(!((pin[Pin].port_num == PORT_PortC) && (Pin <=  PORT_Pin3)) ) /* PC0 to PC3 */
    {
       if (pin[Pin].change_direction == PIN_MODE_UNCHANGEABLE)
       {
        if(pin[Pin].direction == PORT_PIN_OUT)
        {
          /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , pin[Pin].pin_num);
         }
        else if(pin[Pin].direction== PORT_PIN_IN)
        {
          /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) ,pin[Pin].pin_num);
        }
      else
      {
        /* Do Nothing */
      }
    }
    }
    else
    {
      /* Do Nothing ... JTAG Pins */
    }
   
  }
}



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
  void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
  {
    #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
	/* Report to DET  */
	Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
          
          /* Check if the Driver is initialized before using this function */
	if(Port_Status == PORT_NOT_INITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
	}
	else
	{	/* Do Nothing */	}
    #endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
  }
#endif
