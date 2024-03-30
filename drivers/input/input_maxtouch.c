#define DT_DRV_COMPAT microchip_maxtouch

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>

#include "input_maxtouch.h"

LOG_MODULE_REGISTER(maxtouch, CONFIG_INPUT_LOG_LEVEL);

// TODO UGH
#define DIVIDE_UNSIGNED_ROUND(numerator, denominator)                                              \
    (((numerator) + ((denominator) / 2)) / (denominator))
#define CPI_TO_SAMPLES(cpi, dist_in_mm) (DIVIDE_UNSIGNED_ROUND((cpi) * (dist_in_mm) * 10, 254))
#define SAMPLES_TO_CPI(samples, dist_in_mm) (DIVIDE_UNSIGNED_ROUND((samples) * 254, (dist_in_mm) * 10)
#define MXT_DEFAULT_DPI 600
#define MXT_TOUCH_THRESHOLD 18
#define MXT_GAIN 4
#define MXT_DX_GAIN 255

static int mxt_seq_read(const struct device *dev, const uint16_t addr, void *buf,
                        const uint8_t len) {
    const struct mxt_config *config = dev->config;

    const uint16_t addr_lsb = sys_cpu_to_le16(addr);

    return i2c_write_read_dt(&config->bus, &addr_lsb, sizeof(addr_lsb), buf, len);
}

static int mxt_seq_write(const struct device *dev, const uint16_t addr, void *buf,
                         const uint8_t len) {
    const struct mxt_config *config = dev->config;
    struct i2c_msg msg[2];

    const uint16_t addr_lsb = sys_cpu_to_le16(addr);
    msg[0].buf = (uint8_t *)&addr_lsb;
    msg[0].len = 2U;
    msg[0].flags = I2C_MSG_WRITE;

    msg[1].buf = (uint8_t *)buf;
    msg[1].len = len;
    msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    return i2c_transfer_dt(&config->bus, msg, 2);
}

static inline bool is_t100_report(const struct device *dev, int report_id) {
    const struct mxt_config *config = dev->config;
    struct mxt_data *data = dev->data;

    return (report_id >= data->t100_first_report_id + 2 &&
            report_id < data->t100_first_report_id + 2 + config->max_touch_points);
}

static void mxt_report_data(const struct device *dev) {
    const struct mxt_config *config = dev->config;
    struct mxt_data *data = dev->data;
    int ret;

    if (!data->t44_message_count_address) {
        return;
    }

    uint8_t msg_count;
    ret = mxt_seq_read(dev, data->t44_message_count_address, &msg_count, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read message count: %d", ret);
        return;
    }

    uint16_t pending_fingers = 0;
    bool last_touch_status = false;
    for (int i = 0; i < msg_count; i++) {
        struct mxt_message msg;

        ret = mxt_seq_read(dev, data->t5_message_processor_address, &msg, sizeof(msg));
        if (ret < 0) {
            LOG_ERR("Failed to read message: %d", ret);
            return;
        }

        if (is_t100_report(dev, msg.report_id)) {
            uint8_t finger_idx = msg.report_id - data->t100_first_report_id - 2;
            bool pending_for_finger = (pending_fingers & BIT(finger_idx)) != 0;

            enum t100_touch_event ev = msg.data[0] & 0xF;
            uint16_t x_pos = msg.data[1] + (msg.data[2] << 8);
            uint16_t y_pos = msg.data[3] + (msg.data[4] << 8);

            switch (ev) {
            case DOWN:
            case MOVE:
            case UP:
            case NO_EVENT:
                if (pending_for_finger) {
                    input_report_key(dev, INPUT_BTN_TOUCH, last_touch_status, true, K_FOREVER);
                    pending_fingers = 0;
                }
                WRITE_BIT(pending_fingers, finger_idx, 1);
                last_touch_status = (ev != UP);
                input_report_abs(dev, INPUT_ABS_MT_SLOT, finger_idx, false, K_FOREVER);
                input_report_abs(dev, INPUT_ABS_X, x_pos, false, K_FOREVER);
                input_report_abs(dev, INPUT_ABS_Y, y_pos, false, K_FOREVER);
                input_report_key(dev, INPUT_BTN_TOUCH, last_touch_status, false, K_FOREVER);
                break;
            default:
                // All other events are ignored
                break;
            }
        } else {
            LOG_HEXDUMP_DBG(msg.data, 5, "message data");
        }
    }

    if (pending_fingers != 0) {
        input_report_key(dev, INPUT_BTN_TOUCH, last_touch_status, true, K_FOREVER);
    }

    return;
}

static void mxt_work_cb(struct k_work *work) {
    struct mxt_data *data = CONTAINER_OF(work, struct mxt_data, work);
    mxt_report_data(data->dev);
}

static void mxt_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct mxt_data *data = CONTAINER_OF(cb, struct mxt_data, gpio_cb);
    k_work_submit(&data->work);
}

static int mxt_load_object_table(const struct device *dev, struct mxt_information_block *info) {
    struct mxt_data *data = dev->data;
    int ret = 0;

    ret = mxt_seq_read(dev, MXT_REG_INFORMATION_BLOCK, info, sizeof(struct mxt_information_block));

    if (ret < 0) {
        LOG_ERR("Failed to load the info block: %d", ret);
        return ret;
    }

    LOG_HEXDUMP_DBG(info, sizeof(struct mxt_information_block), "info block");
    LOG_DBG("Found a maXTouch: family %d, variant %d, version %d. Matrix size: "
            "%d/%d and num of objects %d",
            info->family_id, info->variant_id, info->version, info->matrix_x_size,
            info->matrix_y_size, info->num_objects);

    uint8_t report_id = 1;
    uint16_t object_addr =
        sizeof(struct mxt_information_block); // Object table starts after the info block
    for (int i = 0; i < info->num_objects; i++) {
        struct mxt_object_table_element obj_table;

        ret = mxt_seq_read(dev, object_addr, &obj_table, sizeof(obj_table));
        if (ret < 0) {
            LOG_ERR("Failed to load object table %d: %d", i, ret);
            return ret;
        }

        uint16_t addr = sys_le16_to_cpu(obj_table.position);

        switch (obj_table.type) {
        case 2:
            data->t2_encryption_status_address = addr;
            break;
        case 5:
            data->t5_message_processor_address = addr;
            // We won't request a checksum, so subtract one
            data->t5_max_message_size = obj_table.size_minus_one - 1;
            break;
        case 6:
            data->t6_command_processor_address = addr;
            break;
        case 7:
            data->t7_powerconfig_address = addr;
            break;
        case 8:
            data->t8_acquisitionconfig_address = addr;
            break;
        case 44:
            data->t44_message_count_address = addr;
            break;
        case 46:
            data->t46_cte_config_address = addr;
            break;
        case 100:
            data->t100_multiple_touch_touchscreen_address = addr;
            data->t100_first_report_id = report_id;
            break;
        }

        object_addr += sizeof(obj_table);
        report_id += obj_table.report_ids_per_instance * (obj_table.instances_minus_one + 1);
    }

    return 0;
};

static int mxt_load_config(const struct device *dev,
                           const struct mxt_information_block *information) {
    struct mxt_data *data = dev->data;
    const struct mxt_config *config = dev->config;
    int ret;

    if (data->t7_powerconfig_address) {
        struct mxt_gen_powerconfig_t7 t7_conf = {0};
        t7_conf.idleacqint = config->idle_acq_time;
        t7_conf.actacqint = config->active_acq_time;
        t7_conf.actv2idleto = config->active_to_idle_timeout;

        ret = mxt_seq_write(dev, data->t7_powerconfig_address, &t7_conf, sizeof(t7_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T7 config: %d", ret);
            return ret;
        }
    }

    if (data->t8_acquisitionconfig_address) {
        struct mxt_gen_acquisitionconfig_t8 t8_conf = {0};
        ret = mxt_seq_write(dev, data->t8_acquisitionconfig_address, &t8_conf, sizeof(t8_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T8 config: %d", ret);
            return ret;
        }
    }

    if (data->t100_multiple_touch_touchscreen_address) {
        struct mxt_touch_multiscreen_t100 t100_conf = {0};

        ret = mxt_seq_read(dev, data->t100_multiple_touch_touchscreen_address, &t100_conf,
                           sizeof(t100_conf));
        if (ret < 0) {
            LOG_ERR("Failed to load the initial T100 config: %d", ret);
            return ret;
        }

        t100_conf.ctrl =
            MXT_T100_CTRL_RPTEN | MXT_T100_CTRL_ENABLE; // Enable the t100 object, and enable
                                                        // message reporting for the t100 object.1`
        uint8_t cfg1 = 0;

        if (config->repeat_each_cycle) {
            cfg1 |= MXT_T100_CFG_RPTEACHCYCLE;
        }

        if (config->swap_xy) {
            cfg1 |= MXT_T100_CFG_SWITCHXY;
        }

        if (config->invert_x) {
            cfg1 |= MXT_T100_CFG_INVERTX;
        }

        if (config->invert_y) {
            cfg1 |= MXT_T100_CFG_INVERTY;
        }

        t100_conf.cfg1 = cfg1; // Could also handle rotation, and axis inversion in hardware here

        t100_conf.scraux = 0x1;                       // AUX data: Report the number of touch events
        t100_conf.numtch = config->max_touch_points;  // The number of touch reports
                                                      // we want to receive (upto 10)
        t100_conf.xsize = information->matrix_x_size; // Make configurable as this depends on the
                                                      // sensor design.
        t100_conf.ysize = information->matrix_y_size; // Make configurable as this depends on the
                                                      // sensor design.
                                                      //
        t100_conf.xpitch = (MXT_SENSOR_WIDTH_MM * 10 / information->matrix_x_size) -
                           50; // Pitch between X-Lines (5mm + 0.1mm * XPitch).
        t100_conf.ypitch = (MXT_SENSOR_HEIGHT_MM * 10 / information->matrix_y_size) -
                           50;          // Pitch between Y-Lines (5mm + 0.1mm * YPitch).
        t100_conf.gain = MXT_GAIN;      // Single transmit gain for mutual capacitance measurements
        t100_conf.dxgain = MXT_DX_GAIN; // Dual transmit gain for mutual capacitance
                                        // measurements (255 = auto calibrate)
        t100_conf.tchthr = MXT_TOUCH_THRESHOLD; // Touch threshold
        t100_conf.mrgthr = 5;                   // Merge threshold
        t100_conf.mrghyst = 5;                  // Merge threshold hysteresis
        t100_conf.movsmooth = 224;              // The amount of smoothing applied to movements,
                                                // this tails off at higher speeds
        t100_conf.movfilter = 4 & 0xF; // The lower 4 bits are the speed response value, higher
                                       // values reduce lag, but also smoothing

        // These two fields implement a simple filter for reducing jitter, but large
        // values cause the pointer to stick in place before moving.
        t100_conf.movhysti = 6; // Initial movement hysteresis
        t100_conf.movhystn = 4; // Next movement hysteresis

        t100_conf.xrange = sys_cpu_to_le16(
            CPI_TO_SAMPLES(MXT_DEFAULT_DPI,
                           MXT_SENSOR_HEIGHT_MM)); // CPI handling, adjust the reported resolution
        t100_conf.yrange = sys_cpu_to_le16(
            CPI_TO_SAMPLES(MXT_DEFAULT_DPI,
                           MXT_SENSOR_WIDTH_MM)); // CPI handling, adjust the reported resolution

        ret = mxt_seq_write(dev, data->t100_multiple_touch_touchscreen_address, &t100_conf,
                            sizeof(t100_conf));
        if (ret < 0) {
            LOG_ERR("Failed to set T100 config: %d", ret);
            return ret;
        }
    }

    return 0;
}

static int mxt_init(const struct device *dev) {
    struct mxt_data *data = dev->data;
    const struct mxt_config *config = dev->config;

    int ret;

    data->dev = dev;

    if (!i2c_is_ready_dt(&config->bus)) {
        LOG_ERR("i2c bus isn't ready!");
        return -EIO;
    };

    struct mxt_information_block info = {0};
    ret = mxt_load_object_table(dev, &info);
    if (ret < 0) {
        LOG_ERR("Failed to load the ojbect table: %d", ret);
        return -EIO;
    }

    gpio_pin_configure_dt(&config->chg, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, mxt_gpio_cb, BIT(config->chg.pin));
    ret = gpio_add_callback(config->chg.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, mxt_work_cb);

    ret = gpio_pin_interrupt_configure_dt(&config->chg, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt for CHG pin %d", ret);
        return -EIO;
    }

    ret = mxt_load_config(dev, &info);
    if (ret < 0) {
        LOG_ERR("Failed to load default config: %d", ret);
        return -EIO;
    }

    // Load any existing messages to clear them, ensure our edge interrupt will fire
    mxt_report_data(dev);

    return 0;
}

#define MXT_INST(n)                                                                                \
    static struct mxt_data mxt_data_##n;                                                           \
    static const struct mxt_config mxt_config_##n = {                                              \
        .bus = I2C_DT_SPEC_INST_GET(n),                                                            \
        .chg = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), chg_gpios, {}),                                 \
        .max_touch_points = DT_INST_PROP_OR(n, max_touch_points, 5),                               \
        .idle_acq_time = DT_INST_PROP_OR(n, idle_acq_time_ms, 32),                                 \
        .active_acq_time = DT_INST_PROP_OR(n, active_acq_time_ms, 10),                             \
        .active_to_idle_timeout = DT_INST_PROP_OR(n, active_to_idle_timeout_ms, 50),               \
        .repeat_each_cycle = DT_INST_PROP(n, repeat_each_cycle),                                   \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                                       \
        .invert_x = DT_INST_PROP(n, invert_x),                                                     \
        .invert_y = DT_INST_PROP(n, invert_y),                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, mxt_init, NULL, &mxt_data_##n, &mxt_config_##n, POST_KERNEL,          \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MXT_INST)
