/*
 * STM32 Eth Driver for RT-Thread
 * Change Logs:
 * Date           Author       Notes
 * 2009-10-05     Bernard      eth interface driver for STM32F107 CL
 */
 
#include "stm32f4xx.h"
#include "stm32f4xx_eth.h"
#include "stm32f4xx_eth_driver.h"
#include <rtthread.h>
#include <netif/ethernetif.h>
#include "lwipopts.h"
/* RT-Thread Device Interface */
#include <netif/etharp.h>
#include <lwip/icmp.h>


/* debug option */
//#define ETH_DEBUG
#define ETH_RX_DUMP
#define ETH_TX_DUMP

#ifdef ETH_DEBUG
#define STM32_ETH_PRINTF          rt_kprintf
#else
#define STM32_ETH_PRINTF(...)
#endif

#define ETH_RXBUFNB        	4
#define ETH_TXBUFNB        	2
static ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];
static rt_uint8_t Rx_Buff[ETH_RXBUFNB][ETH_MAX_PACKET_SIZE], Tx_Buff[ETH_TXBUFNB][ETH_MAX_PACKET_SIZE];

#define MAX_ADDR_LEN 6
struct rt_stm32_eth
{
	/* inherit from ethernet device */
	struct eth_device parent;

	/* interface address info. */
	rt_uint8_t  dev_addr[MAX_ADDR_LEN];			/* hw address	*/

	uint32_t    ETH_Speed; /*!< @ref ETH_Speed */
	uint32_t    ETH_Mode;  /*!< @ref ETH_Duplex_Mode */
};
static struct rt_stm32_eth stm32_eth_device;
static struct rt_semaphore tx_wait;
static rt_bool_t tx_is_waiting = RT_FALSE;

/* initialize the interface */
static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)dev;
    ETH_InitTypeDef ETH_InitStructure;

    /* Enable ETHERNET clock  */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx |
                           RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);

    SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);

    /* Reset ETHERNET on AHB Bus */
    ETH_DeInit();

    /* Software reset */
    ETH_SoftwareReset();

    /* Wait for software reset */
    while (ETH_GetSoftwareResetStatus() == SET);

    /* ETHERNET Configuration --------------------------------------------------*/
    /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
    ETH_StructInit(&ETH_InitStructure);

    /* Fill ETH_InitStructure parametrs */
    /*------------------------   MAC   -----------------------------------*/
    ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
    ETH_InitStructure.ETH_Speed = stm32_eth->ETH_Speed;
    ETH_InitStructure.ETH_Mode  = stm32_eth->ETH_Mode;

    ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
    ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
    ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
    ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
    ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
    ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif

    /*------------------------   DMA   -----------------------------------*/

    /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
    the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
    if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
    ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
    ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;

    ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
    ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
    ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
    ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
    ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

    /* configure Ethernet */
    ETH_Init(&ETH_InitStructure);

    /* Enable DMA Receive interrupt (need to enable in this case Normal interrupt) */
    ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, ENABLE);

    /* Initialize Tx Descriptors list: Chain Mode */
    ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    /* Initialize Rx Descriptors list: Chain Mode  */
    ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

    /* MAC address configuration */
    ETH_MACAddressConfig(ETH_MAC_Address0, (u8*)&stm32_eth_device.dev_addr[0]);

    /* Enable MAC and DMA transmission and reception */
    ETH_Start();

    return RT_EOK;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

static rt_size_t rt_stm32_eth_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch(cmd)
	{
	case NIOCTL_GADDR:
		/* get mac address */
		if(args) rt_memcpy(args, stm32_eth_device.dev_addr, 6);
		else return -RT_ERROR;
		break;

	default :
		break;
	}

	return RT_EOK;
}

/* ethernet device interface */
/* transmit packet. */
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

extern ETH_DMADESCTypeDef  *DMAPTPTxDescToSet;
extern ETH_DMADESCTypeDef  *DMAPTPRxDescToGet;	

rt_err_t rt_stm32_eth_tx( rt_device_t dev, struct pbuf* p)
{
    struct pbuf* q;
    rt_uint32_t offset;

    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    while ((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (uint32_t)RESET)
    {
        rt_err_t result;
        rt_uint32_t level;

        level = rt_hw_interrupt_disable();
        tx_is_waiting = RT_TRUE;
        rt_hw_interrupt_enable(level);

        /* it's own bit set, wait it */
        result = rt_sem_take(&tx_wait, RT_WAITING_FOREVER);
        if (result == RT_EOK) break;
        if (result == -RT_ERROR) return -RT_ERROR;
    }

    offset = 0;
    for (q = p; q != NULL; q = q->next)
    {
        uint8_t *to;

        /* Copy the frame to be sent into memory pointed by the current ETHERNET DMA Tx descriptor */
        to = (uint8_t*)((DMATxDescToSet->Buffer1Addr) + offset);
        memcpy(to, q->payload, q->len);
        offset += q->len;
    }
#ifdef ETH_TX_DUMP
    {
        rt_uint32_t i;
        rt_uint8_t *ptr = (rt_uint8_t*)(DMATxDescToSet->Buffer1Addr);

        STM32_ETH_PRINTF("tx_dump, len:%d\r\n", p->tot_len);
        for(i=0; i<p->tot_len; i++)
        {
            STM32_ETH_PRINTF("%02x ",*ptr);
            ptr++;

            if(((i+1)%8) == 0)
            {
                STM32_ETH_PRINTF("  ");
            }
            if(((i+1)%16) == 0)
            {
                STM32_ETH_PRINTF("\r\n");
            }
        }
        STM32_ETH_PRINTF("\r\ndump done!\r\n");
    }
#endif

    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (p->tot_len & ETH_DMATxDesc_TBS1);
    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;
    /* Enable TX Completion Interrupt */
    DMATxDescToSet->Status |= ETH_DMATxDesc_IC;
#ifdef CHECKSUM_BY_HARDWARE
    DMATxDescToSet->Status |= ETH_DMATxDesc_ChecksumTCPUDPICMPFull;
    /* clean ICMP checksum STM32F need */
    {
        struct eth_hdr *ethhdr = (struct eth_hdr *)(DMATxDescToSet->Buffer1Addr);
        /* is IP ? */
        if( ethhdr->type == htons(ETHTYPE_IP) )
        {
            struct ip_hdr *iphdr = (struct ip_hdr *)(DMATxDescToSet->Buffer1Addr + SIZEOF_ETH_HDR);
            /* is ICMP ? */
            if( IPH_PROTO(iphdr) == IP_PROTO_ICMP )
            {
                struct icmp_echo_hdr *iecho = (struct icmp_echo_hdr *)(DMATxDescToSet->Buffer1Addr + SIZEOF_ETH_HDR + sizeof(struct ip_hdr) );
                iecho->chksum = 0;
            }
        }
    }
#endif
    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;
    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    if ((ETH->DMASR & ETH_DMASR_TBUS) != (uint32_t)RESET)
    {
        /* Clear TBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_TBUS;
        /* Transmit Poll Demand to resume DMA transmission*/
        ETH->DMATPDR = 0;
    }

    /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);

    /* Return SUCCESS */
    return RT_EOK;
}

/* reception packet. */
struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
    struct pbuf* p;
    rt_uint32_t offset = 0, framelength = 0;
		
		#define  ETH_DMARXDESC_FRAME_LENGTHSHIFT    16 	/*define in stm32f4xx_eth.c line 94*/
	
    /* init p pointer */
    p = RT_NULL;

    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if(((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (uint32_t)RESET))
        return p;

    if (((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (uint32_t)RESET) &&
            ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (uint32_t)RESET) &&
            ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (uint32_t)RESET))
    {
        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
        framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARXDESC_FRAME_LENGTHSHIFT) - 4;

        /* allocate buffer */
        p = pbuf_alloc(PBUF_LINK, framelength, PBUF_RAM);
        if (p != RT_NULL)
        {
            struct pbuf* q;

            for (q = p; q != RT_NULL; q= q->next)
            {
                /* Copy the received frame into buffer from memory pointed by the current ETHERNET DMA Rx descriptor */
                memcpy(q->payload, (uint8_t *)((DMARxDescToGet->Buffer1Addr) + offset), q->len);
                offset += q->len;
            }
#ifdef ETH_RX_DUMP
            {
                rt_uint32_t i;
                rt_uint8_t *ptr = (rt_uint8_t*)(DMARxDescToGet->Buffer1Addr);

                STM32_ETH_PRINTF("rx_dump, len:%d\r\n", p->tot_len);
                for(i=0; i<p->tot_len; i++)
                {
                    STM32_ETH_PRINTF("%02x ", *ptr);
                    ptr++;

                    if(((i+1)%8) == 0)
                    {
                        STM32_ETH_PRINTF("  ");
                    }
                    if(((i+1)%16) == 0)
                    {
                        STM32_ETH_PRINTF("\r\n");
                    }
                }
                STM32_ETH_PRINTF("\r\ndump done!\r\n");
            }
#endif
        }
    }

    /* Set Own bit of the Rx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMARxDescToGet->Status = ETH_DMARxDesc_OWN;

    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
    if ((ETH->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
    {
        /* Clear RBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        ETH->DMARPDR = 0;
    }

    /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */
    /* Chained Mode */
    if((DMARxDescToGet->ControlBufferSize & ETH_DMARxDesc_RCH) != (uint32_t)RESET)
    {
        /* Selects the next DMA Rx descriptor list for next buffer to read */
        DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);
    }
    else /* Ring Mode */
    {
        if((DMARxDescToGet->ControlBufferSize & ETH_DMARxDesc_RER) != (uint32_t)RESET)
        {
            /* Selects the first DMA Rx descriptor for next buffer to read: last Rx descriptor was used */
            DMARxDescToGet = (ETH_DMADESCTypeDef*) (ETH->DMARDLAR);
        }
        else
        {
            /* Selects the next DMA Rx descriptor list for next buffer to read */
            DMARxDescToGet = (ETH_DMADESCTypeDef*) ((uint32_t)DMARxDescToGet + 0x10 + ((ETH->DMABMR & ETH_DMABMR_DSL) >> 2));
        }
    }

    return p;
}

static void ETH_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the Ethernet global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * GPIO Configuration for ETH
 */

static void ETH_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    /* config MDIO and MDC. */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH); /* config ETH_MDIO */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH); /* config ETH_MDC */
    /* config PA2: MDIO */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* config PC1: MDC */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Ethernet pins configuration ************************************************/
#if defined(MII_MODE)
/*
    ETH_MDIO ------------> PA2
    ETH_MDC -------------> PC1

    ETH_MII_CRS ---------> PA0
    ETH_MII_COL ---------> PA3

    ETH_MII_RX_CLK ------> PA1
    ETH_MII_RX_ER -------> PB10
    ETH_MII_RX_ER -------> PI10
    ETH_MII_RX_DV -------> PA7
    ETH_MII_RXD0 --------> PC4
    ETH_MII_RXD1 --------> PC5
    ETH_MII_RXD2 --------> PB0
    ETH_MII_RXD3 --------> PB1

    ETH_MII_TX_EN -------> PB11
    ETH_MII_TX_EN -------> PG11
    ETH_MII_TX_CLK ------> PC3
    ETH_MII_TXD0 --------> PB12
    ETH_MII_TXD0 --------> PG13
    ETH_MII_TXD1 --------> PB13
    ETH_MII_TXD1 --------> PG14
    ETH_MII_TXD2 --------> PC2
    ETH_MII_TXD3 --------> PB8
    ETH_MII_TXD3  -------> PE2
*/

#error insert MII GPIO initial.
#elif defined(RMII_MODE)
/*
    ETH_MDIO ------------> PA2
    ETH_MDC -------------> PC1

    ETH_RMII_REF_CLK ----> PA1

    ETH_RMII_CRS_DV -----> PA7
    ETH_RMII_RXD0 -------> PC4
    ETH_RMII_RXD1 -------> PC5

    ETH_RMII_TX_EN ------> PG11
    ETH_RMII_TXD0 -------> PG13
    ETH_RMII_TXD1 -------> PG14

    ETH_RMII_TX_EN ------> PB11
    ETH_RMII_TXD0 -------> PB12
    ETH_RMII_TXD1 -------> PB13
*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH); /* RMII_REF_CLK */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH); /* RMII_CRS_DV */

    /* configure PA1:RMII_REF_CLK, PA7:RMII_CRS_DV. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH); /* RMII_RXD0 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH); /* RMII_RXD1 */

    /* configure PC4:RMII_RXD0, PC5:RMII_RXD1. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#if RMII_TX_GPIO_GROUP == 1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH); /* RMII_TX_EN */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH); /* RMII_TXD0 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH); /* RMII_TXD1 */

    /* configure PB11:RMII_TX_EN, PB12:RMII_TXD0, PB13:RMII_TXD1 */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif RMII_TX_GPIO_GROUP == 2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_ETH); /* RMII_TX_EN */
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource13, GPIO_AF_ETH); /* RMII_TXD0 */
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_ETH); /* RMII_TXD1 */

    /* configure PG11:RMII_TX_EN, PG13:RMII_TXD0, PG14:RMII_TXD1 */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
#else
#error RMII_TX_GPIO_GROUP setting error!
#endif /*RMII_TX_GPIO_GROUP */
#else
#error MII_MODE or RMII_MODE  not define!
#endif /* RMII_MODE */

}

/* PHY: DM9161A	//LAN8720 */
static uint8_t phy_speed = 0;
#define PHY_LINK_MASK       (1<<0)
#define PHY_100M_MASK       (1<<1)
#define PHY_DUPLEX_MASK     (1<<2)		
static void phy_monitor_thread_entry(void *parameter)
{
    uint8_t phy_addr = 0xFF;
    uint8_t phy_speed_new = 0;

    /* phy search */
    {
        rt_uint32_t i;
        rt_uint16_t temp;

        for(i=0; i<=0x1F; i++)
        {
            temp = ETH_ReadPHYRegister(i, 0x02);

            if( temp != 0xFFFF )
            {
                phy_addr = i;
                break;
            }
        }
    } /* phy search */

    if(phy_addr == 0xFF)
    {
        STM32_ETH_PRINTF("phy not probe!\r\n");
        return;
    }
    else
    {
        STM32_ETH_PRINTF("found a phy, address:0x%02X\r\n", phy_addr);
    }

    /* RESET PHY */
    STM32_ETH_PRINTF("RESET PHY!\r\n");
    ETH_WritePHYRegister(phy_addr, PHY_BCR, PHY_Reset);
    rt_thread_delay(RT_TICK_PER_SECOND * 2);
    ETH_WritePHYRegister(phy_addr, PHY_BCR, PHY_AutoNegotiation);
		
		{
			uint16_t id1  = ETH_ReadPHYRegister(phy_addr, 0x02);
			uint16_t id2  = ETH_ReadPHYRegister(phy_addr, 0x03);
			STM32_ETH_PRINTF("DM9161A ID1:0x%04X\r\n", id1);	
			STM32_ETH_PRINTF("DM9161A ID2:0x%04X\r\n", id2);
		}
		
    while(1)
    {
        uint16_t status  = ETH_ReadPHYRegister(phy_addr, PHY_BSR);
				
			
        STM32_ETH_PRINTF("DM9161A status:0x%04X\r\n", status);
				
        phy_speed_new = 0;

        if(status & (PHY_AutoNego_Complete | PHY_Linked_Status))/*Auto-negotiation process completed or Valid link is established*/
        {
            uint16_t tempSR1,tempSR2;

            tempSR1 = ETH_ReadPHYRegister(phy_addr, PHY_SR+1);/*REG17[15:12]100FDX:100HDX:10FDX:10HDX*/
            STM32_ETH_PRINTF("DM9161A REG 17:0x%04X\r\n", tempSR1);

            tempSR2 = (tempSR1 >> 15); /* Bit 15 PHY_FULLDUPLEX_100M Speed Indication. */
            phy_speed_new = PHY_LINK_MASK;

            if((tempSR2&0x01) == 1)
            {
                phy_speed_new |= PHY_100M_MASK;
								phy_speed_new |= PHY_DUPLEX_MASK;
            }

            //if(SR & 0x04)
            //{
                //phy_speed_new |= PHY_DUPLEX_MASK;
            //}
        }

        /* linkchange */
        if(phy_speed_new != phy_speed)
        {
            if(phy_speed_new & PHY_LINK_MASK)
            {
                STM32_ETH_PRINTF("link up ");

                if(phy_speed_new & PHY_100M_MASK)
                {
                    STM32_ETH_PRINTF("100Mbps");
                    stm32_eth_device.ETH_Speed = ETH_Speed_100M;
                }
                else
                {
                    stm32_eth_device.ETH_Speed = ETH_Speed_10M;
                    STM32_ETH_PRINTF("10Mbps");
                }

                if(phy_speed_new & PHY_DUPLEX_MASK)
                {
                    STM32_ETH_PRINTF(" full-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_FullDuplex;
                }
                else
                {
                    STM32_ETH_PRINTF(" half-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_HalfDuplex;
                }
                rt_stm32_eth_init((rt_device_t)&stm32_eth_device);

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
            } /* link up. */
            else
            {
                STM32_ETH_PRINTF("link down\r\n");
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
            } /* link down. */

            phy_speed = phy_speed_new;
        } /* linkchange */

        rt_thread_delay(RT_TICK_PER_SECOND);
    } /* while(1) */
}

void rt_hw_stm32_eth_init(void)
{
    /* PHY RESET: PC0 */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
        rt_thread_delay(2);
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
        rt_thread_delay(2);
    }

    ETH_GPIO_Configuration();
    ETH_NVIC_Configuration();

    stm32_eth_device.ETH_Speed = ETH_Speed_100M;
    stm32_eth_device.ETH_Mode  = ETH_Mode_FullDuplex;

    /* OUI 00-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = 0x00;
    stm32_eth_device.dev_addr[1] = 0x80;
    stm32_eth_device.dev_addr[2] = 0xE1;
    /* generate MAC addr from 96bit unique ID (only for test). */
    stm32_eth_device.dev_addr[3] = *(rt_uint8_t*)(0x1FFF7A10+4);
    stm32_eth_device.dev_addr[4] = *(rt_uint8_t*)(0x1FFF7A10+2);
    stm32_eth_device.dev_addr[5] = *(rt_uint8_t*)(0x1FFF7A10+0);

    stm32_eth_device.parent.parent.init       = rt_stm32_eth_init;
    stm32_eth_device.parent.parent.open       = rt_stm32_eth_open;
    stm32_eth_device.parent.parent.close      = rt_stm32_eth_close;
    stm32_eth_device.parent.parent.read       = rt_stm32_eth_read;
    stm32_eth_device.parent.parent.write      = rt_stm32_eth_write;
    stm32_eth_device.parent.parent.control    = rt_stm32_eth_control;
    stm32_eth_device.parent.parent.user_data  = RT_NULL;

    stm32_eth_device.parent.eth_rx     = rt_stm32_eth_rx;
    stm32_eth_device.parent.eth_tx     = rt_stm32_eth_tx;

    /* init tx semaphore */
    rt_sem_init(&tx_wait, "tx_wait", 0, RT_IPC_FLAG_FIFO);

    /* register eth device */
    eth_device_init(&(stm32_eth_device.parent), "eth0");

    /* start phy monitor */
    {
        rt_thread_t tid;
        tid = rt_thread_create("phy",
                               phy_monitor_thread_entry,
                               RT_NULL,
                               512,
                               RT_THREAD_PRIORITY_MAX - 2,
                               2);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }
}

/* interrupt service routine */
void ETH_IRQHandler(void)
{
    rt_uint32_t status, ier;

    /* enter interrupt */
    rt_interrupt_enter();

    status = ETH->DMASR;
    ier = ETH->DMAIER;

    if(status & ETH_DMA_IT_MMC)
    {
        STM32_ETH_PRINTF("ETH_DMA_IT_MMC\r\n");
        ETH_DMAClearITPendingBit(ETH_DMA_IT_MMC);
    }

    if(status & ETH_DMA_IT_NIS)
    {
        rt_uint32_t nis_clear = ETH_DMA_IT_NIS;

        /* [0]:Transmit Interrupt. */
        if((status & ier) & ETH_DMA_IT_T) /* packet transmission */
        {
            STM32_ETH_PRINTF("ETH_DMA_IT_T\r\n");

            if (tx_is_waiting == RT_TRUE)
            {
                tx_is_waiting = RT_FALSE;
                rt_sem_release(&tx_wait);
            }

            nis_clear |= ETH_DMA_IT_T;
        }

        /* [2]:Transmit Buffer Unavailable. */

        /* [6]:Receive Interrupt. */
        if((status & ier) & ETH_DMA_IT_R) /* packet reception */
        {
            STM32_ETH_PRINTF("ETH_DMA_IT_R\r\n");
            /* a frame has been received */
            eth_device_ready(&(stm32_eth_device.parent));

            nis_clear |= ETH_DMA_IT_R;
        }

        /* [14]:Early Receive Interrupt. */

        ETH_DMAClearITPendingBit(nis_clear);
    }

    if(status & ETH_DMA_IT_AIS)
    {
        rt_uint32_t ais_clear = ETH_DMA_IT_AIS;
        STM32_ETH_PRINTF("ETH_DMA_IT_AIS\r\n");

        /* [1]:Transmit Process Stopped. */
        if(status & ETH_DMA_IT_TPS)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_TPS\r\n");
            ais_clear |= ETH_DMA_IT_TPS;
        }

        /* [3]:Transmit Jabber Timeout. */
        if(status & ETH_DMA_IT_TJT)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_TJT\r\n");
            ais_clear |= ETH_DMA_IT_TJT;
        }

        /* [4]: Receive FIFO Overflow. */
        if(status & ETH_DMA_IT_RO)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_RO\r\n");
            ais_clear |= ETH_DMA_IT_RO;
        }

        /* [5]: Transmit Underflow. */
        if(status & ETH_DMA_IT_TU)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_TU\r\n");
            ais_clear |= ETH_DMA_IT_TU;
        }

        /* [7]: Receive Buffer Unavailable. */
        if(status & ETH_DMA_IT_RBU)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_RBU\r\n");
            ais_clear |= ETH_DMA_IT_RBU;
        }

        /* [8]: Receive Process Stopped. */
        if(status & ETH_DMA_IT_RPS)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_RPS\r\n");
            ais_clear |= ETH_DMA_IT_RPS;
        }

        /* [9]: Receive Watchdog Timeout. */
        if(status & ETH_DMA_IT_RWT)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_RWT\r\n");
            ais_clear |= ETH_DMA_IT_RWT;
        }

        /* [10]: Early Transmit Interrupt. */

        /* [13]: Fatal Bus Error. */
        if(status & ETH_DMA_IT_FBE)
        {
            STM32_ETH_PRINTF("AIS ETH_DMA_IT_FBE\r\n");
            ais_clear |= ETH_DMA_IT_FBE;
        }

        ETH_DMAClearITPendingBit(ais_clear);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
