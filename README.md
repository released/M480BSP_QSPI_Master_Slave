# M480BSP_QSPI_Master_Slave
 M480BSP_QSPI_Master_Slave

update @ 2020/02/21

use QSPI DMA interrupt to performace master and slave transmit

- PA0 : QSPI0_MOSI0 , PA1 : QSPI0_MISO0 , 

- PA2 : QSPI0_CLK , 

- PA3 : QSPI0_SS , 

- PA4 : QSPI0_MOSI1 , PA5 : QSPI0_MISO1 , 

In KeilC project define ENABLE_QSPI_MASTER , ENABLE_QSPI_SLAVE , use M487 EVM x 2 for QSPI master and slave

under QSPI_TX in QSPI_Master_PDMA_Enable and QSPI_Slave_PDMA_Enable function , to change TX data

Master TX buffer : g_au8MasterToSlaveTestPattern

Slave TX buffer : g_au8SlaveToMasterTestPattern

- check below array for RX data

Master RX buffer : g_au8MasterRxBuffer

Slave RX buffer : g_au8SlaveRxBuffer

![image](https://github.com/released/M480BSP_QSPI_Master_Slave/blob/master/SampleCode/Template/Buffer_Transmit.jpg)
