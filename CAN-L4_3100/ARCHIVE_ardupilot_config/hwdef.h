/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MHZ (1000U*1000U)
#define KHZ (1000U)


#define AP_SIGNED_FIRMWARE 0

#define HAL_ENABLE_DFU_BOOT FALSE
#define CHIBIOS_BOARD_NAME "MatekL431-MagHiRes"
// MCU type (ChibiOS define)
#define STM32L431_MCUCONF
#define STM32L431_MCUCONF

#define STM32L431xx

// crystal frequency
#define STM32_HSECLK 8000000U

// UART used for stdout (printf)
#define HAL_STDOUT_SERIAL SD1

// baudrate used for stdout (printf)
#define HAL_STDOUT_BAUDRATE 57600

#define HAL_USE_FATFS FALSE

#define HAL_USE_SDC FALSE
#ifndef STM32L4
#define STM32L4 1
#endif
#define HAL_USE_HW_RNG FALSE
#define HAL_PROCESS_STACK_SIZE 0xA00
#define HAL_STORAGE_SIZE 800
#define STM32_ST_USE_TIMER 15
#define CH_CFG_ST_RESOLUTION 16
#define SERIAL_BUFFERS_SIZE 512
#define PORT_INT_REQUIRED_STACK 64
#define DMA_RESERVE_SIZE 0
#define AP_PARAM_MAX_EMBEDDED_PARAM 512
#define HAL_LED_ON 1
#define HAL_I2C_CLEAR_ON_TIMEOUT 0
#define HAL_I2C_INTERNAL_MASK 1
#define HAL_CAN_POOL_SIZE 6000
#define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0
#define HAL_RCIN_THREAD_ENABLED 1
#define AP_PERIPH_MAG_ENABLED 1
#define HAL_COMPASS_MAX_SENSORS 1
#define AP_PERIPH_MAG_HIRES 1
#define AP_PERIPH_MAG_MAX_RATE 0
#define HAL_USE_ADC FALSE
#define HAL_CAN_IFACE1_ENABLE
#define HAL_CAN_INTERFACE_LIST 0
#define HAL_CAN_INTERFACE_REV_LIST 0,-1,-1
#define HAL_CAN_BASE_LIST reinterpret_cast<bxcan::CanType*>(uintptr_t(CAN1_BASE))
#define HAL_NUM_CAN_IFACES 1
#define HAL_CANFD_SUPPORTED 0
#define BOARD_FLASH_SIZE 256

// location of loaded firmware
#define FLASH_LOAD_ADDRESS 0x0800a000
#define EXT_FLASH_SIZE_MB 0
#define EXT_FLASH_RESERVE_START_KB 0
#define EXT_FLASH_RESERVE_END_KB 0
#define CRT0_AREAS_NUMBER 1
#define __EXTFLASHFUNC__
#define STORAGE_FLASH_PAGE 18
#ifndef AP_CRASHDUMP_ENABLED
#define AP_CRASHDUMP_ENABLED 0
#endif

#define HAL_RAM0_START 0x20000000
#define HAL_RAM_RESERVE_START 0x00000100
// memory regions
#define HAL_MEMORY_REGIONS {(void*)0x20000100, 0x0000ff00, 0x01 }
#define HAL_CC_MEMORY_REGIONS {0x20000000, 0x20010000, CRASH_CATCHER_BYTE }
#define HAL_MEMORY_TOTAL_KB 63

// CPU serial number (12 bytes)
#define UDID_START UID_BASE


// APJ board ID (for bootloaders)
#define APJ_BOARD_ID 1062

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif
#define HAL_EXPECTED_SYSCLOCK 80000000
#define HAL_ROMFS_UNCOMPRESSED
#define STM32_DMA_REQUIRED TRUE


#ifndef HAL_FLASH_PROTECTION
#define HAL_FLASH_PROTECTION 0
#endif
#define HAL_SPI1_CONFIG { &SPID1, 1, STM32_SPI_SPI1_DMA_STREAMS, PAL_LINE(GPIOA,5U) }
#define HAL_SPI_BUS_LIST HAL_SPI1_CONFIG


// SPI device table
#define HAL_SPI_DEVICE0  SPIDesc("rm3100"         ,  0,  1, PAL_LINE(GPIOA,4U) , SPIDEV_MODE0,   1*MHZ,   1*MHZ)

// spi devices table
#define HAL_SPI_DEVICE_LIST \
   HAL_SPI_DEVICE0
#define HAL_WITH_SPI_RM3100 1

// ADC config
#define HAL_ANALOG_PINS \


// GPIO config
// full pin define list
#define HAL_GPIO_PIN_CAN1_RX              PAL_LINE(GPIOB,8U)
#define HAL_GPIO_PIN_CAN1_TX              PAL_LINE(GPIOB,9U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,13U)
#define HAL_GPIO_PIN_I2C2_SDA             PAL_LINE(GPIOB,14U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,13U)
#define HAL_GPIO_PIN_JTCK_SWCLK           PAL_LINE(GPIOA,14U)
#define HAL_GPIO_PIN_JTMS_SWDIO           PAL_LINE(GPIOA,13U)
#define HAL_GPIO_PIN_LED                  PAL_LINE(GPIOB,3U)
#define HAL_GPIO_PIN_MAG_CS               PAL_LINE(GPIOA,4U)
#define HAL_GPIO_PIN_SPARE_CS             PAL_LINE(GPIOB,2U)
#define HAL_GPIO_PIN_SPI1_MISO            PAL_LINE(GPIOB,4U)
#define HAL_GPIO_PIN_SPI1_MOSI            PAL_LINE(GPIOA,7U)
#define HAL_GPIO_PIN_SPI1_SCK             PAL_LINE(GPIOA,5U)
#define HAL_GPIO_PIN_USART1_RX            PAL_LINE(GPIOB,7U)
#define HAL_GPIO_PIN_USART1_TX            PAL_LINE(GPIOB,6U)
#define HAL_GPIO_PIN_USART2_RX            PAL_LINE(GPIOA,3U)
#define HAL_GPIO_PIN_USART2_TX            PAL_LINE(GPIOA,2U)
#define HAL_GPIO_PIN_USART3_RX            PAL_LINE(GPIOB,11U)
#define HAL_GPIO_PIN_USART3_TX            PAL_LINE(GPIOB,10U)

#define HAL_MAG_PROBE1  {add_backend(DRIVER_RM3100, AP_Compass_RM3100::probe(hal.spi->get_device("rm3100"),false,ROTATION_PITCH_180));RETURN_IF_NO_SPACE;}
#undef AP_COMPASS_RM3100_ENABLED
#define AP_COMPASS_RM3100_ENABLED 1
#define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1


#ifndef AP_CHECK_FIRMWARE_ENABLED
#define AP_CHECK_FIRMWARE_ENABLED 1
#endif
// peripherals enabled
#define STM32_I2C_USE_I2C2                  TRUE
#define STM32_SPI_USE_SPI1                  TRUE
#ifndef STM32_SERIAL_USE_USART1
#define STM32_SERIAL_USE_USART1 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART2
#define STM32_SERIAL_USE_USART2 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART3
#define STM32_SERIAL_USE_USART3 TRUE
#endif
#define AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED 0


// auto-generated DMA mapping from dma_resolver.py
#define STM32_I2C_I2C2_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 5)
#define STM32_I2C_I2C2_RX_DMA_CHAN     3
#define STM32_I2C_I2C2_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 4)
#define STM32_I2C_I2C2_TX_DMA_CHAN     3
#define STM32_SPI_SPI1_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 2)
#define STM32_SPI_SPI1_RX_DMA_CHAN     1
#define STM32_SPI_SPI1_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 3)
#define STM32_SPI_SPI1_TX_DMA_CHAN     1
#define STM32_UART_USART1_RX_DMA_STREAM STM32_DMA_STREAM_ID(2, 7)
#define STM32_UART_USART1_RX_DMA_CHAN  2
#define STM32_UART_USART1_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 6)
#define STM32_UART_USART1_TX_DMA_CHAN  2
#define STM32_UART_USART2_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 6)
#define STM32_UART_USART2_RX_DMA_CHAN  2
#define STM32_UART_USART2_TX_DMA_STREAM STM32_DMA_STREAM_ID(1, 7)
#define STM32_UART_USART2_TX_DMA_CHAN  2

// Mask of DMA streams which are shared
#define SHARED_DMA_MASK 0


// generated UART DMA configuration lines
#define STM32_USART1_RX_DMA_CONFIG true, STM32_UART_USART1_RX_DMA_STREAM, STM32_UART_USART1_RX_DMA_CHAN
#define STM32_USART1_TX_DMA_CONFIG true, STM32_UART_USART1_TX_DMA_STREAM, STM32_UART_USART1_TX_DMA_CHAN
#define STM32_USART2_RX_DMA_CONFIG true, STM32_UART_USART2_RX_DMA_STREAM, STM32_UART_USART2_RX_DMA_CHAN
#define STM32_USART2_TX_DMA_CONFIG true, STM32_UART_USART2_TX_DMA_STREAM, STM32_UART_USART2_TX_DMA_CHAN
#define STM32_USART3_RX_DMA_CONFIG false, 0, 0
#define STM32_USART3_TX_DMA_CONFIG false, 0, 0


// generated SPI DMA configuration lines
#define STM32_SPI_SPI1_DMA_STREAMS STM32_SPI_SPI1_TX_DMA_STREAM, STM32_SPI_SPI1_RX_DMA_STREAM
#define HAL_PWM_COUNT 0

#ifndef HAL_USE_PWM
#define HAL_USE_PWM FALSE
#endif

// No Alarm output pin defined
#undef HAL_PWM_ALARM

// PWM timer config

// PWM output config
#define HAL_PWM_GROUPS 

// I2C configuration

#if defined(STM32_I2C_I2C2_RX_DMA_STREAM) && defined(STM32_I2C_I2C2_TX_DMA_STREAM)
#define HAL_I2C2_CONFIG { &I2CD2, 2, STM32_I2C_I2C2_RX_DMA_STREAM, STM32_I2C_I2C2_TX_DMA_STREAM, PAL_LINE(GPIOB,13U), PAL_LINE(GPIOB,14U) }
#else
#define HAL_I2C2_CONFIG { &I2CD2, 2, SHARED_DMA_NONE, SHARED_DMA_NONE, PAL_LINE(GPIOB,13U), PAL_LINE(GPIOB,14U) }
#endif


// i2c devices table
#define HAL_I2C_DEVICE_LIST \
   HAL_I2C2_CONFIG

// UART configuration
#define HAL_HAVE_SERIAL0 1
#define HAL_HAVE_SERIAL0_PARAMS 1
#define HAL_HAVE_SERIAL1 1
#define HAL_HAVE_SERIAL1_PARAMS 1
#define HAL_HAVE_SERIAL2 1
#define HAL_HAVE_SERIAL2_PARAMS 1
#define HAL_NUM_SERIAL_PORTS 3
#define HAL_SERIAL0_DRIVER ChibiOS::UARTDriver serial0Driver(0)
#define HAL_SERIAL1_DRIVER ChibiOS::UARTDriver serial1Driver(1)
#define HAL_SERIAL2_DRIVER ChibiOS::UARTDriver serial2Driver(2)
#define HAL_SERIAL3_DRIVER Empty::UARTDriver serial3Driver
#define HAL_SERIAL4_DRIVER Empty::UARTDriver serial4Driver
#define HAL_SERIAL5_DRIVER Empty::UARTDriver serial5Driver
#define HAL_SERIAL6_DRIVER Empty::UARTDriver serial6Driver
#define HAL_SERIAL7_DRIVER Empty::UARTDriver serial7Driver
#define HAL_SERIAL8_DRIVER Empty::UARTDriver serial8Driver
#define HAL_SERIAL9_DRIVER Empty::UARTDriver serial9Driver
#define HAL_WITH_IO_MCU 0

#define HAL_USART1_CONFIG { (BaseSequentialStream*) &SD1, 1, false, STM32_USART1_RX_DMA_CONFIG, STM32_USART1_TX_DMA_CONFIG, PAL_LINE(GPIOB,6U), PAL_LINE(GPIOB,7U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_USART2_CONFIG { (BaseSequentialStream*) &SD2, 2, false, STM32_USART2_RX_DMA_CONFIG, STM32_USART2_TX_DMA_CONFIG, PAL_LINE(GPIOA,2U), PAL_LINE(GPIOA,3U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_USART3_CONFIG { (BaseSequentialStream*) &SD3, 3, false, STM32_USART3_RX_DMA_CONFIG, STM32_USART3_TX_DMA_CONFIG, PAL_LINE(GPIOB,10U), PAL_LINE(GPIOB,11U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}

// serial devices table
#define HAL_SERIAL_DEVICE_LIST \
   HAL_USART1_CONFIG,\
   HAL_USART2_CONFIG,\
   HAL_USART3_CONFIG
#define HAL_UART_NUM_SERIAL_PORTS 3
#define AP_BOOTLOADER_FLASHING_ENABLED 1

/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/* PORTA:
 PA2 USART2_TX USART2 AF7
 PA3 USART2_RX USART2 AF7
 PA4 MAG_CS CS
 PA5 SPI1_SCK SPI1 AF5
 PA7 SPI1_MOSI SPI1 AF5
 PA13 JTMS-SWDIO SWD AF0
 PA14 JTCK-SWCLK SWD AF0
*/

#define VAL_GPIOA_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_OUTPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOA_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOA_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_HIGH(2U) | \
                           PIN_OSPEED_HIGH(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOA_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_PULLUP(2U) | \
                           PIN_PUPDR_PULLUP(3U) | \
                           PIN_PUPDR_PULLUP(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_PULLDOWN(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOA_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOA_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 7U) | \
                           PIN_AFIO_AF(3U, 7U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 5U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 5U))

#define VAL_GPIOA_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTB:
 PB2 SPARE_CS CS
 PB3 LED OUTPUT
 PB4 SPI1_MISO SPI1 AF5
 PB6 USART1_TX USART1 AF7
 PB7 USART1_RX USART1 AF7
 PB8 CAN1_RX CAN1 AF9
 PB9 CAN1_TX CAN1 AF9
 PB10 USART3_TX USART3 AF7
 PB11 USART3_RX USART3 AF7
 PB13 I2C2_SCL I2C2 AF4
 PB14 I2C2_SDA I2C2 AF4
*/

#define VAL_GPIOB_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_OUTPUT(2U) | \
                           PIN_MODE_OUTPUT(3U) | \
                           PIN_MODE_ALTERNATE(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOB_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_OPENDRAIN(13U) | \
                           PIN_OTYPE_OPENDRAIN(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOB_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_HIGH(6U) | \
                           PIN_OSPEED_HIGH(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_HIGH(10U) | \
                           PIN_OSPEED_HIGH(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOB_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_PULLUP(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_PULLUP(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_PULLUP(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOB_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOB_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 5U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 7U) | \
                           PIN_AFIO_AF(7U, 7U))

#define VAL_GPIOB_AFRH    (PIN_AFIO_AF(8U, 9U) | \
                           PIN_AFIO_AF(9U, 9U) | \
                           PIN_AFIO_AF(10U, 7U) | \
                           PIN_AFIO_AF(11U, 7U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 4U) | \
                           PIN_AFIO_AF(14U, 4U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTC:
*/

#define VAL_GPIOC_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOC_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOC_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOC_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOC_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOC_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOC_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTD:
*/

#define VAL_GPIOD_MODER               0x0
#define VAL_GPIOD_OTYPER              0x0
#define VAL_GPIOD_OSPEEDR             0x0
#define VAL_GPIOD_PUPDR               0x0
#define VAL_GPIOD_ODR                 0x0
#define VAL_GPIOD_AFRL                0x0
#define VAL_GPIOD_AFRH                0x0



/* PORTE:
*/

#define VAL_GPIOE_MODER               0x0
#define VAL_GPIOE_OTYPER              0x0
#define VAL_GPIOE_OSPEEDR             0x0
#define VAL_GPIOE_PUPDR               0x0
#define VAL_GPIOE_ODR                 0x0
#define VAL_GPIOE_AFRL                0x0
#define VAL_GPIOE_AFRH                0x0



/* PORTF:
*/

#define VAL_GPIOF_MODER               0x0
#define VAL_GPIOF_OTYPER              0x0
#define VAL_GPIOF_OSPEEDR             0x0
#define VAL_GPIOF_PUPDR               0x0
#define VAL_GPIOF_ODR                 0x0
#define VAL_GPIOF_AFRL                0x0
#define VAL_GPIOF_AFRH                0x0



/* PORTG:
*/

#define VAL_GPIOG_MODER               0x0
#define VAL_GPIOG_OTYPER              0x0
#define VAL_GPIOG_OSPEEDR             0x0
#define VAL_GPIOG_PUPDR               0x0
#define VAL_GPIOG_ODR                 0x0
#define VAL_GPIOG_AFRL                0x0
#define VAL_GPIOG_AFRH                0x0



/* PORTH:
*/

#define VAL_GPIOH_MODER               0x0
#define VAL_GPIOH_OTYPER              0x0
#define VAL_GPIOH_OSPEEDR             0x0
#define VAL_GPIOH_PUPDR               0x0
#define VAL_GPIOH_ODR                 0x0
#define VAL_GPIOH_AFRL                0x0
#define VAL_GPIOH_AFRH                0x0



/* PORTI:
*/

#define VAL_GPIOI_MODER               0x0
#define VAL_GPIOI_OTYPER              0x0
#define VAL_GPIOI_OSPEEDR             0x0
#define VAL_GPIOI_PUPDR               0x0
#define VAL_GPIOI_ODR                 0x0
#define VAL_GPIOI_AFRL                0x0
#define VAL_GPIOI_AFRH                0x0



/* PORTJ:
*/

#define VAL_GPIOJ_MODER               0x0
#define VAL_GPIOJ_OTYPER              0x0
#define VAL_GPIOJ_OSPEEDR             0x0
#define VAL_GPIOJ_PUPDR               0x0
#define VAL_GPIOJ_ODR                 0x0
#define VAL_GPIOJ_AFRL                0x0
#define VAL_GPIOJ_AFRH                0x0



/* PORTK:
*/

#define VAL_GPIOK_MODER               0x0
#define VAL_GPIOK_OTYPER              0x0
#define VAL_GPIOK_OSPEEDR             0x0
#define VAL_GPIOK_PUPDR               0x0
#define VAL_GPIOK_ODR                 0x0
#define VAL_GPIOK_AFRL                0x0
#define VAL_GPIOK_AFRH                0x0




// AP_Periph defaults

// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for AP_Periph builds

#ifndef AP_SCHEDULER_ENABLED
#define AP_SCHEDULER_ENABLED 0
#endif

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

/*
  AP_Periph doesn't include the SERIAL parameter tree, instead each
  supported serial device type has it's own parameter within AP_Periph
  for which port is used.
 */
#ifndef DEFAULT_SERIAL0_PROTOCOL
#define DEFAULT_SERIAL0_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL1_PROTOCOL
#define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL2_PROTOCOL
#define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL3_PROTOCOL
#define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL4_PROTOCOL
#define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL5_PROTOCOL
#define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL6_PROTOCOL
#define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL7_PROTOCOL
#define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL8_PROTOCOL
#define DEFAULT_SERIAL8_PROTOCOL SerialProtocol_None
#endif
#ifndef DEFAULT_SERIAL9_PROTOCOL
#define DEFAULT_SERIAL9_PROTOCOL SerialProtocol_None
#endif

#ifndef HAL_LOGGING_MAVLINK_ENABLED
#define HAL_LOGGING_MAVLINK_ENABLED 0
#endif

#ifndef AP_AHRS_ENABLED
#define AP_AHRS_ENABLED 0
#endif

#ifndef AP_MISSION_ENABLED
#define AP_MISSION_ENABLED 0
#endif

#ifndef HAL_RALLY_ENABLED
#define HAL_RALLY_ENABLED 0
#endif

#ifndef HAL_NMEA_OUTPUT_ENABLED
#define HAL_NMEA_OUTPUT_ENABLED 0
#endif

#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID 0
#endif

#define PERIPH_FW TRUE
#define HAL_BUILD_AP_PERIPH

#ifndef HAL_WATCHDOG_ENABLED_DEFAULT
#define HAL_WATCHDOG_ENABLED_DEFAULT true
#endif

#ifndef AP_FETTEC_ONEWIRE_ENABLED
#define AP_FETTEC_ONEWIRE_ENABLED 0
#endif

#ifndef HAL_TORQEEDO_ENABLED
#define HAL_TORQEEDO_ENABLED 0
#endif

#ifndef AP_KDECAN_ENABLED
#define AP_KDECAN_ENABLED 0
#endif

#ifndef HAL_GENERATOR_ENABLED
#define HAL_GENERATOR_ENABLED 0
#endif

#ifndef HAL_BARO_WIND_COMP_ENABLED
#define HAL_BARO_WIND_COMP_ENABLED 0
#endif

#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED (HAL_GCS_ENABLED || HAL_LOGGING_ENABLED)
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 0
#endif

#ifndef AP_AIRSPEED_AUTOCAL_ENABLE
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#endif

#ifndef AP_STATS_ENABLED
#define AP_STATS_ENABLED 0
#endif

#ifndef AP_VOLZ_ENABLED
#define AP_VOLZ_ENABLED 0
#endif

#ifndef AP_ROBOTISSERVO_ENABLED
#define AP_ROBOTISSERVO_ENABLED 0
#endif

#ifndef AP_SBUSOUTPUT_ENABLED
#define AP_SBUSOUTPUT_ENABLED 0
#endif

// by default an AP_Periph defines as many servo output channels as
// there are PWM outputs:
#ifndef NUM_SERVO_CHANNELS
#ifdef HAL_PWM_COUNT
#define NUM_SERVO_CHANNELS HAL_PWM_COUNT
#else
#define NUM_SERVO_CHANNELS 0
#endif
#endif

#ifndef AP_BATTERY_ESC_ENABLED
#define AP_BATTERY_ESC_ENABLED 0
#endif

#ifndef AP_BATTERY_WATT_MAX_ENABLED
#define AP_BATTERY_WATT_MAX_ENABLED 0
#endif

// disable compass calibrations on periphs; cal is done on the autopilot
#ifndef COMPASS_CAL_ENABLED
#define COMPASS_CAL_ENABLED 0
#endif
#ifndef AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
#define AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED 0
#endif
#ifndef COMPASS_MOT_ENABLED
#define COMPASS_MOT_ENABLED 0
#endif
#ifndef COMPASS_LEARN_ENABLED
#define COMPASS_LEARN_ENABLED 0
#endif

#ifndef AP_EXTERNAL_AHRS_ENABLED
#define AP_EXTERNAL_AHRS_ENABLED 0
#endif

// disable RC_Channels library:
#ifndef AP_RC_CHANNEL_ENABLED
#define AP_RC_CHANNEL_ENABLED 0
#endif

#define HAL_CRSF_TELEM_ENABLED 0
#define AP_GHST_TELEM_ENABLED 0

#ifndef AP_SERVORELAYEVENTS_ENABLED
#define AP_SERVORELAYEVENTS_ENABLED 0
#endif

/*
 * sanity checks that hwdefs are up-to-date in terms of how they are
 * trying to configure the peripheral:
 */
#ifdef HAL_PERIPH_ENABLE_DEVICE_TEMPERATURE
#error "Change 'define HAL_PERIPH_ENABLE_DEVICE_TEMPERATURE' to 'define AP_PERIPH_DEVICE_TEMPERATURE_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_MSP
#error "Change 'define HAL_PERIPH_ENABLE_MSP' to 'define AP_PERIPH_MSP_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_NOTIFY
#error "Change 'define HAL_PERIPH_ENABLE_NOTIFY' to 'define AP_PERIPH_NOTIFY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_SERIAL_OPTIONS
#error "Change 'define HAL_PERIPH_ENABLE_SERIAL_OPTIONS' to 'define AP_PERIPH_SERIAL_OPTIONS_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
#error "Change 'define HAL_PERIPH_ENABLE_GPS' to 'define AP_PERIPH_GPS_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_BATTERY
#error "Change 'define HAL_PERIPH_ENABLE_BATTERY' to 'define AP_PERIPH_BATTERY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_BATTERY_BALANCE
#error "Change 'define HAL_PERIPH_ENABLE_BATTERY_BALANCE' to 'define AP_PERIPH_BATTERY_BALANCE_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
#error "Change 'define HAL_PERIPH_ENABLE_MAG' to 'define AP_PERIPH_MAG_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
#error "Change 'define HAL_PERIPH_ENABLE_BARO' to 'define AP_PERIPH_BARO_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RC_OUT
#error "Change 'define HAL_PERIPH_ENABLE_RC_OUT' to 'define AP_PERIPH_RC_OUT_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
#error "Change 'define HAL_PERIPH_ENABLE_RANGEFINDER' to 'define AP_PERIPH_RANGEFINDER_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_EFI
#error "Change 'define AP_PERIPH_EFI_ENABLED' to 'define AP_PERIPH_EFI_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_IMU
#error "Change 'define HAL_PERIPH_ENABLE_IMU' to 'define AP_PERIPH_IMU_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RTC
#error "Change 'define HAL_PERIPH_ENABLE_RTC' to 'define AP_PERIPH_RTC_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_ADSB
#error "Change 'define HAL_PERIPH_ENABLE_ADSB' to 'define AP_PERIPH_ADSB_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RELAY
#error "Change 'define HAL_PERIPH_ENABLE_RELAY' to 'define AP_PERIPH_RELAY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RCIN
#error "Change 'define HAL_PERIPH_ENABLE_RCIN' to 'define AP_PERIPH_RCIN_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RPM
#error "Change 'define HAL_PERIPH_ENABLE_RPM' to 'define AP_PERIPH_RPM_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_RPM_STREAM
#error "Change 'define HAL_PERIPH_ENABLE_RPM_STREAM' to 'define AP_PERIPH_RPM_STREAM_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_PROXIMITY
#error "Change 'define HAL_PERIPH_ENABLE_PROXIMITY' to 'define AP_PERIPH_PROXIMITY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
#error "Change 'define HAL_PERIPH_ENABLE_AIRSPEED' to 'define AP_PERIPH_AIRSPEED_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_HWESC
#error "Change 'define HAL_PERIPH_ENABLE_HWESC' to 'define AP_PERIPH_HOBBYWING_ESC_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_NETWORKING
#error "Change 'define HAL_PERIPH_ENABLE_NETWORKING' to 'define AP_PERIPH_NETWORKING_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
#error "Change 'define HAL_PERIPH_ENABLE_PWM_HARDPOINT' to 'define AP_PERIPH_PWM_HARDPOINT_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_ESC_APD
#error "Change 'define HAL_PERIPH_ENABLE_ESC_APD' to 'define AP_PERIPH_ESC_APD_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_NCP5623_LED_WITHOUT_NOTIFY
#error "Change 'define HAL_PERIPH_ENABLE_NCP5623_LED_WITHOUT_NOTIFY' to 'define AP_PERIPH_NCP5623_LED_WITHOUT_NOTIFY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_TOSHIBA_LED_WITHOUT_NOTIFY
#error "Change 'define HAL_PERIPH_ENABLE_TOSHIBA_LED_WITHOUT_NOTIFY' to 'define AP_PERIPH_TOSHIBA_LED_WITHOUT_NOTIFY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_NCP5623_BGR_LED_WITHOUT_NOTIFY
#error "Change 'define HAL_PERIPH_ENABLE_NCP5623_BGR_LED_WITHOUT_NOTIFY' to 'define AP_PERIPH_NCP5623_BGR_LED_WITHOUT_NOTIFY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY
#error "Change 'define HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY' to 'define AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED 1'"
#endif
#ifdef HAL_PERIPH_ENABLE_BUZZER
#error "Change 'define HAL_PERIPH_ENABLE_BUZZER' to 'define AP_PERIPH_BUZZER_ENABLED 1'"
#endif

/*
 * defaults for various AP_Periph features:
 */
#ifndef AP_PERIPH_DEVICE_TEMPERATURE_ENABLED
#define AP_PERIPH_DEVICE_TEMPERATURE_ENABLED 0
#endif
#ifndef AP_PERIPH_MSP_ENABLED
#define AP_PERIPH_MSP_ENABLED 0
#endif
#ifndef AP_PERIPH_NOTIFY_ENABLED
#define AP_PERIPH_NOTIFY_ENABLED 0
#endif
#ifndef AP_PERIPH_SERIAL_OPTIONS_ENABLED
#define AP_PERIPH_SERIAL_OPTIONS_ENABLED 0
#endif
#ifndef AP_PERIPH_BATTERY_ENABLED
#define AP_PERIPH_BATTERY_ENABLED 0
#endif
#ifndef AP_PERIPH_RELAY_ENABLED
#define AP_PERIPH_RELAY_ENABLED 0
#endif
#ifndef AP_PERIPH_BATTERY_BALANCE_ENABLED
#define AP_PERIPH_BATTERY_BALANCE_ENABLED 0
#endif
#ifndef AP_PERIPH_BATTERY_TAG_ENABLED
#define AP_PERIPH_BATTERY_TAG_ENABLED 0
#endif
#ifndef AP_PERIPH_PROXIMITY_ENABLED
#define AP_PERIPH_PROXIMITY_ENABLED 0
#endif
#ifndef AP_PERIPH_GPS_ENABLED
#define AP_PERIPH_GPS_ENABLED 0
#endif
#ifndef AP_PERIPH_ADSB_ENABLED
#define AP_PERIPH_ADSB_ENABLED 0
#endif
#ifndef AP_PERIPH_MAG_ENABLED
#define AP_PERIPH_MAG_ENABLED 0
#endif
#ifndef AP_PERIPH_BARO_ENABLED
#define AP_PERIPH_BARO_ENABLED 0
#endif
#ifndef AP_PERIPH_RANGEFINDER_ENABLED
#define AP_PERIPH_RANGEFINDER_ENABLED 0
#endif
#ifndef AP_PERIPH_IMU_ENABLED
#define AP_PERIPH_IMU_ENABLED 0
#endif
#ifndef AP_PERIPH_RC_OUT_ENABLED
#define AP_PERIPH_RC_OUT_ENABLED 0
#endif
#ifndef AP_PERIPH_EFI_ENABLED
#define AP_PERIPH_EFI_ENABLED 0
#endif
#ifndef AP_PERIPH_RTC_ENABLED
#define AP_PERIPH_RTC_ENABLED AP_PERIPH_BATTERY_TAG_ENABLED
#endif
#ifndef AP_PERIPH_RTC_GLOBALTIME_ENABLED
#define AP_PERIPH_RTC_GLOBALTIME_ENABLED 0
#endif
#ifndef AP_PERIPH_RCIN_ENABLED
#define AP_PERIPH_RCIN_ENABLED 0
#endif
#ifndef AP_PERIPH_RPM_ENABLED
#define AP_PERIPH_RPM_ENABLED 0
#endif
#ifndef AP_PERIPH_RPM_STREAM_ENABLED
#define AP_PERIPH_RPM_STREAM_ENABLED AP_PERIPH_RPM_ENABLED
#endif
#ifndef AP_PERIPH_AIRSPEED_ENABLED
#define AP_PERIPH_AIRSPEED_ENABLED 0
#endif
#ifndef AP_PERIPH_HOBBYWING_ESC_ENABLED
#define AP_PERIPH_HOBBYWING_ESC_ENABLED 0
#endif
#ifndef AP_PERIPH_NETWORKING_ENABLED
#define AP_PERIPH_NETWORKING_ENABLED 0
#endif
#ifndef AP_PERIPH_PWM_HARDPOINT_ENABLED
#define AP_PERIPH_PWM_HARDPOINT_ENABLED 0
#endif
#ifndef AP_PERIPH_ESC_APD_ENABLED
#define AP_PERIPH_ESC_APD_ENABLED 0
#endif
#ifndef AP_PERIPH_NCP5623_LED_WITHOUT_NOTIFY_ENABLED
#define AP_PERIPH_NCP5623_LED_WITHOUT_NOTIFY_ENABLED 0
#endif
#ifndef AP_PERIPH_TOSHIBA_LED_WITHOUT_NOTIFY_ENABLED
#define AP_PERIPH_TOSHIBA_LED_WITHOUT_NOTIFY_ENABLED 0
#endif
#ifndef AP_PERIPH_NCP5623_BGR_LED_WITHOUT_NOTIFY_ENABLED
#define AP_PERIPH_NCP5623_BGR_LED_WITHOUT_NOTIFY_ENABLED 0
#endif
#ifndef AP_PERIPH_BUZZER_ENABLED
#define AP_PERIPH_BUZZER_ENABLED 0
#endif
#ifndef AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED
#define AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED 0
#endif
#ifndef AP_PERIPH_ACTUATOR_TELEM_ENABLED
#define AP_PERIPH_ACTUATOR_TELEM_ENABLED 0
#endif

/*
 * turning on of ArduPilot features based on which AP_Periph features
 * are enabled:
 */
#define AP_BATTERY_ENABLED AP_PERIPH_BATTERY_ENABLED
#define AP_GPS_ENABLED AP_PERIPH_GPS_ENABLED
#define AP_COMPASS_ENABLED AP_PERIPH_MAG_ENABLED
#define AP_BARO_ENABLED AP_PERIPH_BARO_ENABLED
#ifndef AP_RANGEFINDER_ENABLED
#define AP_RANGEFINDER_ENABLED AP_PERIPH_RANGEFINDER_ENABLED
#endif
#define AP_INERTIALSENSOR_ENABLED AP_PERIPH_IMU_ENABLED
#define AP_INERTIALSENSOR_ALLOW_NO_SENSORS AP_PERIPH_IMU_ENABLED
#define AP_RTC_ENABLED AP_PERIPH_RTC_ENABLED
#ifndef AP_RCPROTOCOL_ENABLED
#define AP_RCPROTOCOL_ENABLED AP_PERIPH_RCIN_ENABLED
#endif
#ifndef AP_RPM_ENABLED
#define AP_RPM_ENABLED AP_PERIPH_RPM_ENABLED
#endif
#ifndef AP_TEMPERATURE_SENSOR_ENABLED
#define AP_TEMPERATURE_SENSOR_ENABLED AP_PERIPH_DEVICE_TEMPERATURE_ENABLED
#endif
#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED AP_PERIPH_MSP_ENABLED
#endif
#ifndef AP_RELAY_ENABLED
#define AP_RELAY_ENABLED AP_PERIPH_RELAY_ENABLED
#endif
#ifndef HAL_PROXIMITY_ENABLED
#define HAL_PROXIMITY_ENABLED AP_PERIPH_PROXIMITY_ENABLED
#endif
#ifndef HAL_EFI_ENABLED
#define HAL_EFI_ENABLED AP_PERIPH_EFI_ENABLED
#endif

/*
 * GPS Backends - we selectively turn backends on.
 *   Note also that f103-GPS explicitly disables some of these backends.
 */
#define AP_GPS_BACKEND_DEFAULT_ENABLED 0
#ifndef AP_GPS_UBLOX_ENABLED
#define AP_GPS_UBLOX_ENABLED AP_PERIPH_GPS_ENABLED
#endif
#ifndef HAL_MSP_GPS_ENABLED
#define HAL_MSP_GPS_ENABLED AP_PERIPH_GPS_ENABLED && HAL_MSP_SENSORS_ENABLED
#endif

#ifndef AP_GPS_ERB_ENABLED
#define AP_GPS_ERB_ENABLED 0
#endif

#ifndef AP_GPS_GSOF_ENABLED
#define AP_GPS_GSOF_ENABLED AP_PERIPH_GPS_ENABLED
#endif

#ifndef AP_GPS_NMEA_ENABLED
#define AP_GPS_NMEA_ENABLED 0
#endif

#ifndef AP_GPS_SBF_ENABLED
#define AP_GPS_SBF_ENABLED AP_PERIPH_GPS_ENABLED
#endif

#ifndef AP_GPS_SBP_ENABLED
#define AP_GPS_SBP_ENABLED 0
#endif

#ifndef AP_GPS_SBP2_ENABLED
#define AP_GPS_SBP2_ENABLED 0
#endif

#ifndef AP_GPS_SIRF_ENABLED
#define AP_GPS_SIRF_ENABLED 0
#endif

#ifndef AP_GPS_MAV_ENABLED
#define AP_GPS_MAV_ENABLED 0
#endif

#ifndef AP_GPS_NOVA_ENABLED
#define AP_GPS_NOVA_ENABLED AP_PERIPH_GPS_ENABLED
#endif

#ifndef AP_SIM_GPS_ENABLED
#define AP_SIM_GPS_ENABLED (AP_SIM_ENABLED && AP_GPS_ENABLED)
#endif

#ifndef AP_SIM_VICON_ENABLED
#define AP_SIM_VICON_ENABLED 0
#endif  // AP_SIM_VICON_ENABLED

/*
 * Airspeed Backends - we selectively turn backends *off*
 */
#ifndef AP_AIRSPEED_ANALOG_ENABLED
#define AP_AIRSPEED_ANALOG_ENABLED 0
#endif

// disable various rangefinder backends
#define AP_RANGEFINDER_ANALOG_ENABLED 0
#define AP_RANGEFINDER_HC_SR04_ENABLED 0
#define AP_RANGEFINDER_PWM_ENABLED 0

// AP_Periph expects ROTATION_NONE
#ifndef AP_RANGEFINDER_DEFAULT_ORIENTATION
#define AP_RANGEFINDER_DEFAULT_ORIENTATION ROTATION_NONE
#endif

// no CAN manager in AP_Periph:
#define HAL_CANMANAGER_ENABLED 0

// SLCAN is off by default:
#ifndef AP_CAN_SLCAN_ENABLED
#define AP_CAN_SLCAN_ENABLED 0
#endif

// Periphs don't use the FFT library:
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

// MSP parsing is off by default in AP_Periph:
#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED 0
#endif

// periph does not make use of compass scaling or diagonals
#ifndef AP_COMPASS_DIAGONALS_ENABLED
#define AP_COMPASS_DIAGONALS_ENABLED 0
#endif

// disable various battery monitor backends:
#ifndef AP_BATTERY_SYNTHETIC_CURRENT_ENABLED
#define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED 0
#endif

#ifndef AP_BATT_MONITOR_MAX_INSTANCES
#define AP_BATT_MONITOR_MAX_INSTANCES 1
#endif

#ifndef AP_BATTERY_SUM_ENABLED
#define AP_BATTERY_SUM_ENABLED 0  // needs three backends
#endif  // AP_BATTERY_SUM_ENABLED

// Capacity tracking off
#ifndef AP_BATT_MONITOR_BATTERY_CAPACITY
#define AP_BATT_MONITOR_BATTERY_CAPACITY 0
#endif

#ifndef RANGEFINDER_MAX_INSTANCES
#define RANGEFINDER_MAX_INSTANCES 1
#endif

#ifndef HAL_ADSB_ENABLED
#define HAL_ADSB_ENABLED 0
#endif

#ifndef AP_AIS_ENABLED
#define AP_AIS_ENABLED 0
#endif

// no fence by default in AP_Periph:
#ifndef AP_FENCE_ENABLED
#define AP_FENCE_ENABLED 0
#endif

// periph does not save temperature cals etc:
#ifndef HAL_ENABLE_SAVE_PERSISTENT_PARAMS
#define HAL_ENABLE_SAVE_PERSISTENT_PARAMS 0
#endif

#ifndef AP_WINCH_ENABLED
#define AP_WINCH_ENABLED 0
#endif

#ifndef HAL_VISUALODOM_ENABLED
#define HAL_VISUALODOM_ENABLED 0
#endif

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 0
#endif

#ifndef AP_FRSKY_TELEM_ENABLED
#define AP_FRSKY_TELEM_ENABLED 0
#endif

#ifndef HAL_SPEKTRUM_TELEM_ENABLED
#define HAL_SPEKTRUM_TELEM_ENABLED 0
#endif

#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED 0
#endif

#ifndef NOTIFY_LED_OVERRIDE_DEFAULT
#define NOTIFY_LED_OVERRIDE_DEFAULT 1       // rgb_source_t::mavlink
#endif

#ifndef HAL_PROXIMITY_ENABLED
#define HAL_PROXIMITY_ENABLED 0
#endif

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

#ifndef HAL_SERIAL_ESC_COMM_ENABLED
#define HAL_SERIAL_ESC_COMM_ENABLED 0
#endif

#ifndef HAL_RCIN_THREAD_ENABLED
#define HAL_RCIN_THREAD_ENABLED 0
#endif

#ifndef HAL_MONITOR_THREAD_ENABLED
#define HAL_MONITOR_THREAD_ENABLED 0
#endif

#ifndef HAL_SCHEDULER_LOOP_DELAY_ENABLED
#define HAL_SCHEDULER_LOOP_DELAY_ENABLED 0
#endif

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM 0
#endif

#ifndef AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
#define AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED 0
#endif

#define AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED 0

#ifndef AP_RPM_STREAM_ENABLED
#define AP_RPM_STREAM_ENABLED AP_PERIPH_RPM_STREAM_ENABLED
#endif

#ifndef AP_BOOTLOADER_ALWAYS_ERASE
#define AP_BOOTLOADER_ALWAYS_ERASE 1
#endif

#ifndef GPS_MOVING_BASELINE
#define GPS_MOVING_BASELINE 0
#endif

#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED AP_PERIPH_SERIAL_OPTIONS_ENABLED || (AP_PERIPH_GPS_ENABLED && (GPS_MOVING_BASELINE || BOARD_FLASH_SIZE>=256))
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif

#ifndef HAL_MAVLINK_BINDINGS_ENABLED
#define HAL_MAVLINK_BINDINGS_ENABLED AP_PERIPH_ADSB_ENABLED || HAL_GCS_ENABLED
#endif

// for boards other than AP_Periph we are always expecting delays when
// not initialised.  We can't afford that on AP_Periph as you may end
// up with a bricked node if you write a bad firmware to it.
#ifndef AP_HAL_CHIBIOS_IN_EXPECTED_DELAY_WHEN_NOT_INITIALISED
#define AP_HAL_CHIBIOS_IN_EXPECTED_DELAY_WHEN_NOT_INITIALISED 0
#endif

#ifndef AP_SERIALLED_ENABLED
#define AP_SERIALLED_ENABLED 0
#endif

#ifndef AP_OPTICALFLOW_ENABLED
#define AP_OPTICALFLOW_ENABLED 0
#endif

#ifndef HAL_BUTTON_ENABLED
#define HAL_BUTTON_ENABLED 0
#endif

#ifndef AP_NOTIFY_SCRIPTING_LED_ENABLED
#define AP_NOTIFY_SCRIPTING_LED_ENABLED 0
#endif

#ifndef AP_SCRIPTING_BINDING_VEHICLE_ENABLED
#define AP_SCRIPTING_BINDING_VEHICLE_ENABLED 0
#endif  // AP_SCRIPTING_BINDING_VEHICLE_ENABLED

#ifndef AP_PARAM_DYNAMIC_ENABLED
#define AP_PARAM_DYNAMIC_ENABLED 0
#endif

#ifndef HAL_MOUNT_ENABLED
#define HAL_MOUNT_ENABLED 0
#endif

#ifndef AP_CAMERA_ENABLED
#define AP_CAMERA_ENABLED 0
#endif

#ifndef AP_TERRAIN_AVAILABLE
#define AP_TERRAIN_AVAILABLE 0
#endif

#ifndef AP_ICENGINE_ENABLED
#define AP_ICENGINE_ENABLED 0
#endif

#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED 0
#endif

#ifndef AP_ADVANCEDFAILSAFE_ENABLED
#define AP_ADVANCEDFAILSAFE_ENABLED 0
#endif

#ifndef AP_ARMING_ENABLED
#define AP_ARMING_ENABLED 0
#endif

#ifndef AP_LTM_TELEM_ENABLED
#define AP_LTM_TELEM_ENABLED 0
#endif

#ifndef AP_GRIPPER_ENABLED
#define AP_GRIPPER_ENABLED 0
#endif

#ifndef HAL_SPRAYER_ENABLED
#define HAL_SPRAYER_ENABLED 0
#endif

#ifndef AP_VEHICLE_ENABLED
#define AP_VEHICLE_ENABLED 0
#endif

#ifndef OSD_ENABLED
#define OSD_ENABLED 0
#endif

#ifndef OSD_PARAM_ENABLED
#define OSD_PARAM_ENABLED 0
#endif

#ifndef AP_SCHEDULER_ENABLED
#define AP_SCHEDULER_ENABLED 0
#endif

#ifndef AP_RC_CHANNEL_ENABLED
#define AP_RC_CHANNEL_ENABLED 0
#endif

#ifndef AP_CUSTOMROTATIONS_ENABLED
#define AP_CUSTOMROTATIONS_ENABLED 0
#endif

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED 0
#endif

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 0
#endif

#ifndef HAL_USE_LOAD_MEASURE
#define HAL_USE_LOAD_MEASURE 0
#endif


// end AP_Periph defaults
