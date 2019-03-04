/*
 * Low level Mbed OS driver for ROHM BD2808 24-channel LED Controller
 * on Future Electronics Sequana board (PSoC 6 platform).
 *
 * Copyright (c) 2018 Future Electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cmsis.h"
#include "mbed_assert.h"
#include "mbed_error.h"
#include "mbed_debug.h"
#include "PeripheralPins.h"
#include "pinmap.h"
#include "bd2808_hal.h"
#include "psoc6_utils.h"
#include "cy_dma.h"
#include "cy_trigmux.h"
#include "string.h"


 // ROHHM BD2808 LED controller uses a serial, byte oriented protocol
 // (similar to SPI) for configuration and control, see its datasheet.
 //
 // https://www.rohm.com/products/power-management/led-drivers/constant-current-serial-in-parallel-out/bd2808muv-m-product
 //
 // On Sequana board we use a hardware controller implemented in UDBs
 // to interface it. This controller used 16-bit wide data register and optionally DMA
 // for efficiency, but is swapping bytes internally so the data stream in
 // memory is still organized in a sequence defined by the protocol.

 // Default serial speed (1MHz / 1mbps)
#define BD2808_DEFAULT_SPEED        1000000

// DMA declarations
#define BD2808_DMA_DW_CHANNEL       0
#define BD2808_DMA_HW               DW0
#define BD2808_DMA_INTR_MASK        CY_DMA_INTR_MASK


// UDB device API declarations
#define BD2808_FIFO_REG             (UDB->WRKMULT.F0[8])
#define BD2808_STATUS_REG           (UDB->WRKMULT.ST[9])

/* Status register bits */
#define SHIFTER_STS_TX_READY        (0x01u)
#define SHIFTER_STS_TX_NOT_EMPTY    (0x02u)
#define SHIFTER_STS_SM_STATE_MASK   (0x0Cu)
#define SHIFTER_STS_SM_IDLE         (0x00u)


////////////////////////////////////
////     UDB CONFIGURATION      ////
////////////////////////////////////

 // Include UDB configuration data.
#include "udb_config_data.h"


static void do_memcpy_config(const cfg_memcpy_data_t *ptr)
{
    while (ptr->data != NULL) {
        memcpy((void*)(ptr->address), ptr->data, ptr->count * sizeof(*ptr->data));
        ++ptr;
    }
}

// Configure sparse set of registers
static void do_configure(const cfg_init_t *ptr)
{
    uint32_t i;

    while (ptr->data != NULL) {
        uint8_t *dst = (uint8_t *)(ptr->address);
        for (i = 0; i < ptr->count; ++i) {
            dst[ptr->data[i].offset] = ptr->data[i].value;
        }
        ++ptr;
    }
}

 // Configure UDB array to implement a serial device.
static void configure_UDBs(void)
{
    uint32_t pair, i;
    volatile uint8_t temp;

    /* Reset the UDB array */
    CPUSS->UDB_PWR_CTL = 0x05FA0000;
    for (i = 0; i < 1000; ++i) temp = 0;
    (void)temp;

    /* Power on the UDB array */
    CPUSS->UDB_PWR_CTL = 0x05FA0003;

    /* Clear all the UDB configuration */
    for (pair = 0; pair <= 5; ++pair) {
        for (i = 0; i <2; ++i) {
            memset(&UDB->UDBPAIR[pair].UDBSNG[i], 0, 116 /*sizeof(UDB_UDBPAIR_UDBSNG_Type)*/);
        }
    }

    /* Copy UDB configuration data into registers */
    do_memcpy_config(memcpy_table);
    do_configure(config_table);

    /* Enable UDB array and digital routing */
    UDB->UDBIF.BANK_CTL = UDB->UDBIF.BANK_CTL | 0x106;
}


////////////////////////////////////
////     DMA CONFIGURATION      ////
////////////////////////////////////

const cy_stc_dma_descriptor_config_t bd2808_dma_descr_config =
{
    .retrigger       = CY_DMA_RETRIG_16CYC,
    .interruptType   = CY_DMA_DESCR,
    .triggerOutType  = CY_DMA_DESCR,
    .channelState    = CY_DMA_CHANNEL_DISABLED,
    .triggerInType   = CY_DMA_1ELEMENT,
    .dataSize        = CY_DMA_HALFWORD,
    .srcTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
    .dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
    .descriptorType  = CY_DMA_1D_TRANSFER,
    .srcAddress      = NULL,
    .dstAddress      = NULL,
    .srcXincrement   = 1,
    .dstXincrement   = 0,
    .xCount          = 15,
    .srcYincrement   = 1,
    .dstYincrement   = 0,
    .yCount          = 1,
    .nextDescriptor  = NULL
};

cy_stc_dma_descriptor_t bd2808_dma_descriptor1 =
{
    .ctl = 0,
    .src = 0,
    .dst = 0,
    .xCtl = 0,
    .yCtl = 0,
    .nextPtr = 0
};

static void configure_DMA(void)
{
    cy_stc_dma_channel_config_t channel_config;

	/* Perform Trigger Mux configuration */
	Cy_TrigMux_Connect(TRIG14_IN_UDB_TR_UDB6, TRIG14_OUT_TR_GROUP0_INPUT50, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_LEVEL);
	Cy_TrigMux_Connect(TRIG0_IN_TR_GROUP14_OUTPUT7, TRIG0_OUT_CPUSS_DW0_TR_IN0, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_UDB_TR_UDB__LEVEL);

    Cy_DMA_Descriptor_Init(&bd2808_dma_descriptor1, &bd2808_dma_descr_config);

    channel_config.descriptor  = &bd2808_dma_descriptor1;
    channel_config.preemptable = false;
    channel_config.priority    = 3;
    channel_config.enable      = false;
    channel_config.bufferable  = false;

    Cy_DMA_Channel_Init(BD2808_DMA_HW, BD2808_DMA_DW_CHANNEL, &channel_config);

    Cy_DMA_Enable(BD2808_DMA_HW);
}


static void trigger_DMA(void *ptr, uint32_t count)
{
    // Check and wait if DMA is currently busy.
    if (BD2808_DMA_HW->CH_STRUCT[BD2808_DMA_DW_CHANNEL].CH_CTL & DW_CH_STRUCT_CH_CTL_ENABLED_Msk) {
        // Busy wait till completion.
        while (Cy_DMA_Channel_GetStatus(BD2808_DMA_HW, BD2808_DMA_DW_CHANNEL) == CY_DMA_INTR_CAUSE_NO_INTR) {
        }
    }

    Cy_DMA_Descriptor_SetSrcAddress(&bd2808_dma_descriptor1, (const void *)ptr);
    Cy_DMA_Descriptor_SetDstAddress(&bd2808_dma_descriptor1, (const void *)&BD2808_FIFO_REG);
    Cy_DMA_Descriptor_SetXloopDataCount(&bd2808_dma_descriptor1, count);
    Cy_DMA_Channel_SetDescriptor(BD2808_DMA_HW, BD2808_DMA_DW_CHANNEL, &bd2808_dma_descriptor1);
    Cy_DMA_Channel_Enable(BD2808_DMA_HW, BD2808_DMA_DW_CHANNEL);
}


////////////////////////////////////
////    CLOCK CONFIGURATION     ////
////////////////////////////////////

static int allocate_divider(bd2808_t *obj)
{
    if (obj->div_num == CY_INVALID_DIVIDER) {
        obj->div_type = CY_SYSCLK_DIV_16_BIT;
        obj->div_num = cy_clk_allocate_divider(CY_SYSCLK_DIV_16_BIT);
    }
    return (obj->div_num == CY_INVALID_DIVIDER)? -1 : 0;
}

/*
 * Initializes clock for the required speed
 */
static cy_en_sysclk_status_t init_clock(bd2808_t *obj, uint32_t frequency)
{
    cy_en_sysclk_status_t status = CY_SYSCLK_INVALID_STATE;
    uint32_t div_value;

    if (obj->div_num == CY_INVALID_DIVIDER) {
        if (allocate_divider(obj) < 0) {
            error("BD2808: cannot allocate clock divider.");
            return CY_SYSCLK_INVALID_STATE;
        }
    }

    // Set up proper frequency; round up the divider so the frequency is not higher than specified.
    div_value = (CY_CLK_PERICLK_FREQ_HZ + frequency) / (frequency * 2);
    obj->clk_frequency = CY_CLK_PERICLK_FREQ_HZ / div_value / 2;
    Cy_SysClk_PeriphDisableDivider(obj->div_type, obj->div_num);
    if (Cy_SysClk_PeriphSetDivider(obj->div_type, obj->div_num, div_value) != CY_SYSCLK_SUCCESS) {
        obj->div_num = CY_INVALID_DIVIDER;
    }
    Cy_SysClk_PeriphEnableDivider(obj->div_type, obj->div_num);

    if (obj->div_num != CY_INVALID_DIVIDER) {
        status = Cy_SysClk_PeriphAssignDivider(obj->clock, obj->div_type, obj->div_num);
        if (status != CY_SYSCLK_SUCCESS) {
            error("BD2808: cannot assign clock divider.");
            return status;
        }
    }
    return CY_SYSCLK_SUCCESS;
}

/*
 * Initializes i/o pins for UDB serial device.
 */
static void init_pins(bd2808_t *obj)
{
    if (cy_reserve_io_pin(obj->pin_sclk) ||
            cy_reserve_io_pin(obj->pin_mosi)) {
        error("BD2808: pin reservation conflict.");
    }
    MBED_ASSERT(obj->pin_sclk = P5_6);
    MBED_ASSERT(obj->pin_mosi = P5_4);
    pin_function(obj->pin_sclk, CY_PIN_OUT_FUNCTION(P5_6_DSI_GPIO, obj->clock));
    pin_function(obj->pin_mosi, CY_PIN_OUT_FUNCTION(P5_4_DSI_GPIO, obj->clock));
    // Enable outputs.
    Cy_GPIO_Write(Cy_GPIO_PortToAddr(CY_PORT(obj->pin_mosi)), CY_PIN(obj->pin_mosi), 1);
    Cy_GPIO_Write(Cy_GPIO_PortToAddr(CY_PORT(obj->pin_sclk)), CY_PIN(obj->pin_sclk), 1);
}

/*
 * Hardware initialization. Pins are hard coded because of static UDB configuration.
 */
void bd2808_init(bd2808_t *obj)
{
    uint32_t spi = (uint32_t)NC;

    obj->base = (CySCB_Type*)spi;
    obj->spi_id = 0;
    obj->pin_mosi = P5_4;
    obj->pin_miso = NC;
    obj->pin_sclk = P5_6;
    obj->pin_ssel = NC;
    obj->data_bits = 16;
    obj->clock = PCLK_UDB_CLOCKS0;
    obj->div_num = CY_INVALID_DIVIDER;
    obj->ms_mode = CY_SCB_SPI_MASTER;
#if DEVICE_SPI_ASYNCH
    obj->pending = 0;
    obj->events = 0;
    obj->tx_buffer = NULL;
    obj->tx_buffer_size = 0;
    obj->rx_buffer = NULL;
    obj->rx_buffer_size = 0;
#endif // DEVICE_SPI_ASYNCH
    init_pins(obj);
    init_clock(obj, BD2808_DEFAULT_SPEED);
    configure_UDBs();
    configure_DMA();
}


/*
 *
 */
static void send_data_word(uint16_t word)
{
    while ((BD2808_STATUS_REG & SHIFTER_STS_TX_READY) == 0);
    BD2808_FIFO_REG = word;
}

/*
 *
 */
void bd2808_send_command(uint8_t *command, uint32_t size, bool use_dma)
{
    MBED_ASSERT((size & 0x1) == 0); // Make sure the length is even.
    MBED_ASSERT(((uint32_t)command & 0x1) == 0); // Make address is aligned.

    if (use_dma) {
        trigger_DMA(command, size/2);
    } else {
        uint32_t i;
        uint16_t *ptr = (uint16_t*)command;

        for (i = 0; i < size; i+= sizeof(*ptr)) {
            send_data_word(*ptr++);
        }
    }
}


