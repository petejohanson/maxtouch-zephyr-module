#pragma once

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

struct mxt_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;

    uint16_t t2_encryption_status_address;
    uint16_t t5_message_processor_address;
    uint16_t t5_max_message_size;
    uint16_t t6_command_processor_address;
    uint16_t t7_powerconfig_address;
    uint16_t t8_acquisitionconfig_address;
    uint16_t t44_message_count_address;
    uint16_t t46_cte_config_address;
    uint16_t t100_multiple_touch_touchscreen_address;

    uint16_t t100_first_report_id;
};

struct mxt_config {
    const struct i2c_dt_spec bus;
    const struct gpio_dt_spec chg;
    const uint8_t idle_acq_time;
    const uint8_t active_acq_time;
    const uint8_t active_to_idle_timeout;
    const uint8_t max_touch_points;
    const bool swap_xy;
    const bool invert_x;
    const bool invert_y;
    const bool repeat_each_cycle;
};

#define MXT_REG_INFORMATION_BLOCK (0)

// These are peacock specific, they are used for handling CPI calculations.
#define MXT_SENSOR_WIDTH_MM 156
#define MXT_SENSOR_HEIGHT_MM 91

struct mxt_object_table_element {
    uint8_t type;
    uint16_t position;
    uint8_t size_minus_one;
    uint8_t instances_minus_one;
    uint8_t report_ids_per_instance;
} __packed;

struct mxt_information_block {
    uint8_t family_id;
    uint8_t variant_id;
    uint8_t version;
    uint8_t build;
    uint8_t matrix_x_size;
    uint8_t matrix_y_size;
    uint8_t num_objects;
} __packed;

struct mxt_message {
    uint8_t report_id;
    uint8_t data[5];
} __packed;

struct mxt_message_count {
    uint8_t count;
} __packd;

struct mxt_gen_encryptionstatus_t2 {
    uint16_t status;
    uint8_t payloadcrc[3];
    uint8_t enccustcrc[3];
    uint8_t error;
} __packed;

struct mxt_gen_commandprocessor_t6 {
    uint8_t reset;
    uint8_t backupnv;
    uint8_t calibrate;
    uint8_t reportall;
    uint8_t debugctrl;
    uint8_t diagnostic;
    uint8_t debugctrl2;
} __packed;

struct mxt_gen_powerconfig_t7 {
    uint8_t idleacqint;
    uint8_t actacqint;
    uint8_t actv2idleto;
    uint8_t cfg;
    uint8_t cfg2;
    uint8_t idleacqintfine;
    uint8_t actvaqintfine;
} __packed;

#define MXT_T7_CFG_INITACTV             BIT(7)
#define MXT_T7_CFG_OVFRPTSUP            BIT(6)
#define MXT_T7_CFG_ACTV2IDLETOMSB_SHIFT 2
#define MXT_T7_CFG_ACTV2IDLETOMSB_MASK  0x3C
#define MXT_T7_CFG_ACTVPIPEEN           BIT(1)
#define MXT_T7_CFG_IDLEPIPEEN           BIT(0)

struct mxt_gen_acquisitionconfig_t8 {
    uint8_t chrgtime;
    uint8_t reserved;
    uint8_t tchdrift;
    uint8_t driftst;
    uint8_t tchautocal;
    uint8_t sync;
    uint8_t acthcalst;
    uint8_t acthcalsthr;
    uint8_t atchfrccalthr;
    uint8_t atchfrccalratio;
    uint8_t measallow;
    uint8_t reserved2[3];
    uint8_t cfg;
} __packed;

struct mxt_spt_cteconfig_t46 {
    uint8_t reserved[2];
    uint8_t idlesyncsperx;
    uint8_t activesyncsperx;
    uint8_t adcspersync;
    uint8_t piusesperadc;
    uint8_t xslew;
    uint16_t syncdelay;
    uint8_t xvoltage;
    uint8_t reserved2;
    uint8_t inrushcfg;
    uint8_t reserved3[6];
    uint8_t cfg;
} __packed;

struct mxt_touch_multiscreen_t100 {
    uint8_t ctrl;
    uint8_t cfg1;
    uint8_t scraux;
    uint8_t tchaux;
    uint8_t tcheventcfg;
    uint8_t akscfg;
    uint8_t numtch;
    uint8_t xycfg;
    uint8_t xorigin;
    uint8_t xsize;
    uint8_t xpitch;
    uint8_t xlocip;
    uint8_t xhiclip;
    uint16_t xrange;
    uint8_t xedgecfg;
    uint8_t xedgedist;
    uint8_t dxxedgecfg;
    uint8_t dxxedgedist;
    uint8_t yorigin;
    uint8_t ysize;
    uint8_t ypitch;
    uint8_t ylocip;
    uint8_t yhiclip;
    uint16_t yrange;
    uint8_t yedgecfg;
    uint8_t yedgedist;
    uint8_t gain;
    uint8_t dxgain;
    uint8_t tchthr;
    uint8_t tchhyst;
    uint8_t intthr;
    uint8_t noisesf;
    uint8_t cutoffthr;
    uint8_t mrgthr;
    uint8_t mrgthradjstr;
    uint8_t mrghyst;
    uint8_t dxthrsf;
    uint8_t tchdidown;
    uint8_t tchdiup;
    uint8_t nexttchdi;
    uint8_t calcfg;
    uint8_t jumplimit;
    uint8_t movfilter;
    uint8_t movsmooth;
    uint8_t movpred;
    uint16_t movhysti;
    uint16_t movhystn;
    uint8_t amplhyst;
    uint8_t scrareahyst;
    uint8_t intthryst;
    uint8_t xedgecfghi;
    uint8_t xedgedisthi;
    uint8_t dxxedgecfghi;
    uint8_t dxxedgedisthi;
    uint8_t yedgecfghi;
    uint8_t yedgedisthi;
    uint8_t cfg2;
    uint8_t movhystcfg;
    uint8_t amplcoeff;
    uint8_t amploffset;
    uint8_t jumplimitmov;
    uint16_t jlmmovthr;
    uint8_t jlmmovintthr;
} __packed;

#define MXT_T100_CTRL_SCANEN      BIT(7)
#define MXT_T100_CTRL_DISSCRMSG0  BIT(2)
#define MXT_T100_CTRL_RPTEN       BIT(1)
#define MXT_T100_CTRL_ENABLE      BIT(0)
 
#define MXT_T100_CFG_INVERTX       BIT(7)
#define MXT_T100_CFG_INVERTY       BIT(6)
#define MXT_T100_CFG_SWITCHXY      BIT(5)
#define MXT_T100_CFG_DISLOCK       BIT(4)
#define MXT_T100_CFG_ATCHTHRSEL    BIT(3)
#define MXT_T100_CFG_RPTEACHCYCLE  BIT(0)

// Touch events reported in the t100 messages
enum t100_touch_event {
    NO_EVENT,
    MOVE,
    UNSUP,
    SUP,
    DOWN,
    UP,
    UNSUPSUP,
    UNSUPUP,
    DOWNSUP,
    DOWNUP
};
