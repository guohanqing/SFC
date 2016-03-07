/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4XX_EIH_H
#define __STM32F4XX_EIH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32F ETH dirver options */
#define RMII_MODE                       /* MII_MODE or RMII_MODE */
#define RMII_TX_GPIO_GROUP        1     /* 1:GPIOB or 2:GPIOG */
#define CHECKSUM_BY_HARDWARE	 
	  
/* STM32 ETH HW initialization */
void rt_hw_stm32_eth_init(void);
	 
	 
	 

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4XX_EIH_H */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
