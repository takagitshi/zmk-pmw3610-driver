/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // 150 us required, test shows too short,
                                       // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // 10 ms required in spec,
                                       // test shows too short,
                                       // especially when integrated with display,
                                       // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi, 
                           bool swap_xy, bool inv_x, bool inv_y) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value = 0x00;
    int err = 0;

    LOG_INF("Setting cpi: %d", cpi);
    // Convert CPI to register value
    // Set prefered RES_STEP
    //   BIT 4-0: CPI
    uint8_t cpi_val = cpi / 200;
    value = (value & 0xE0) | (cpi_val & 0x1F);

    // Convert axis to register value
    // Set prefered RES_STEP
    //   BIT 7: SWAP_XY
    //   BIT 6: INV_X
    //   BIT 5: INV_Y
    LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s", 
            swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif

    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value,                0x00};

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    /* Write data */
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (config->force_awake) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read ref-performance %d", err);
            return err;
        }
        LOG_INF("Get performance register (reg value 0x%x)", value);

        // Set prefered RUN RATE        
        //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;
        //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;
        //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved
        uint8_t perf;
        if (config->force_awake_4ms_mode) {
            perf = 0x0d; // RUN RATE @ 4ms
        } else {
            // reset bit[3..0] to 0x0 (normal operation)
            perf = value & 0x0F; // RUN RATE @ 8ms
        }

        if (enabled) {
            perf |= 0xF0; // set bit[3..0] to 0xF (force awake)
        }
        if (perf != value) {
            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance register %d", err);
                return err;
            }
            LOG_INF("Set performance register (reg value 0x%x)", perf);
        }
        LOG_INF("%s performance mode", enabled ? "enable" : "disable");
    }

    return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
	int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) {
        err = pmw3610_set_performance(dev, true);
    }

    if (!err) {
        err = pmw3610_set_cpi(dev, config->cpi, config->swap_xy, config->inv_x, config->inv_y);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                         CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                         CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                         CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                      CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                      CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                      CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
    }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed in step %d", data->async_init_step);
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("PMW3610 initialized");
            pmw3610_set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static bool pmw3610_acceleration_bypass_layer_active(const struct pixart_config *config) {
    return zmk_keymap_layer_active(config->acceleration_scroll_layer) ||
           zmk_keymap_layer_active(config->acceleration_gesture_layer) ||
           zmk_keymap_layer_active(config->acceleration_gesture_layer_2);
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

	int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }
    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

    const struct pmw3610_motion_delta delta =
        pmw3610_decode_motion_delta(buf[PMW3610_X_L_POS], buf[PMW3610_Y_L_POS],
                                    buf[PMW3610_XY_H_POS]);
    int16_t x = delta.x;
    int16_t y = delta.y;
    LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) 
                    + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }
    if (!data->sw_smart_flag && shutter > 45) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    const int64_t now_ms = k_uptime_get();
    pmw3610_report_accumulate(&data->report, now_ms, x, y);

    int64_t delay_ms;
    if (pmw3610_report_prepare_schedule(&data->report, now_ms,
                                        CONFIG_PMW3610_REPORT_INTERVAL_MIN, &delay_ms)) {
        int schedule_err = k_work_reschedule(&data->report_work, K_MSEC(delay_ms));
        if (schedule_err < 0) {
            data->report.report_scheduled = false;
            LOG_ERR("Failed to schedule PMW3610 report: %d", schedule_err);
            return schedule_err;
        }
    }
#else
    const struct pixart_config *config = dev->config;
    int32_t output_x = x;
    int32_t output_y = y;
    const int64_t now_ms = k_uptime_get();

    if (IS_ENABLED(CONFIG_PMW3610_POINTER_ACCELERATION) && config->acceleration_enabled &&
        !pmw3610_acceleration_bypass_layer_active(config)) {
        pmw3610_pointer_accel_apply_frame(&config->acceleration_curve, &data->acceleration,
                                          x, y, now_ms,
                                          config->acceleration_curve.reference_interval_ms,
                                          &output_x, &output_y);
    } else {
        pmw3610_pointer_accel_reset(&data->acceleration);
    }

    bool have_x = output_x != 0;
    bool have_y = output_y != 0;
    if (have_x) {
        input_report(dev, config->evt_type, config->x_input_code, output_x, !have_y, K_NO_WAIT);
    }
    if (have_y) {
        input_report(dev, config->evt_type, config->y_input_code, output_y, true, K_NO_WAIT);
    }
#endif

    return err;
}

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
static bool pmw3610_acceleration_bypassed(const struct pixart_config *config) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    return pmw3610_acceleration_bypass_layer_active(config);
#else
    ARG_UNUSED(config);
    return true;
#endif
}

static struct pmw3610_frame_retry pmw3610_send_frame(const struct device *dev, int16_t x,
                                                      int16_t y, bool force_sync,
                                                      int *x_err, int *y_err) {
    const struct pixart_config *config = dev->config;
    const bool have_x = x != 0;
    const bool have_y = y != 0;
    *x_err = 0;
    *y_err = 0;

    if (!have_x && !have_y && force_sync) {
        *x_err = input_report(dev, config->evt_type, config->x_input_code, 0, true, K_NO_WAIT);
        return (struct pmw3610_frame_retry){0};
    }

    if (have_x) {
        *x_err = input_report(dev, config->evt_type, config->x_input_code, x, !have_y, K_NO_WAIT);
    }

    struct pmw3610_frame_retry retry = pmw3610_frame_retry_result(x, y, *x_err, 0);
    if (retry.send_y) {
        *y_err = input_report(dev, config->evt_type, config->y_input_code, y, true, K_NO_WAIT);
        retry = pmw3610_frame_retry_result(x, y, *x_err, *y_err);
    }

    return retry;
}

static void pmw3610_report_work_callback(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, report_work);
    const struct device *dev = data->dev;
    const struct pixart_config *config = dev->config;
    const int64_t now_ms = k_uptime_get();
    const bool draining_pending = pmw3610_output_has_pending(&data->output);

    data->report.report_scheduled = false;

    if (!draining_pending) {
        int16_t raw_x;
        int16_t raw_y;
        int64_t collection_elapsed_ms;
        if (!pmw3610_report_take(&data->report, now_ms, &raw_x, &raw_y,
                                  &collection_elapsed_ms)) {
            return;
        }

        int32_t output_x = raw_x;
        int32_t output_y = raw_y;
        if (IS_ENABLED(CONFIG_PMW3610_POINTER_ACCELERATION) && config->acceleration_enabled &&
            !pmw3610_acceleration_bypassed(config)) {
            pmw3610_pointer_accel_apply_frame(&config->acceleration_curve, &data->acceleration,
                                              raw_x, raw_y, now_ms, collection_elapsed_ms,
                                              &output_x, &output_y);
        } else {
            pmw3610_pointer_accel_reset(&data->acceleration);
        }
        pmw3610_output_queue(&data->output, output_x, output_y,
                             raw_x != 0 || raw_y != 0);
    }

    const struct pmw3610_output_frame frame = pmw3610_output_take_next(&data->output);
    int x_err = 0;
    int y_err = 0;
    struct pmw3610_frame_retry retry =
        pmw3610_send_frame(dev, frame.x, frame.y, frame.force_sync, &x_err, &y_err);
    const bool zero_sync_failed =
        frame.force_sync && frame.x == 0 && frame.y == 0 && x_err < 0;
    pmw3610_output_complete(&data->output, retry, zero_sync_failed);

    const bool have_pending = pmw3610_output_has_pending(&data->output);
    if (retry.x != 0 || retry.y != 0 || zero_sync_failed) {
        LOG_WRN("PMW3610 input report failed; retaining delta (%d, %d)", x_err, y_err);
    }
    if (draining_pending && !have_pending) {
        data->report.last_report_time = now_ms;
        data->report.have_last_report = true;
    }

    int64_t delay_ms = CONFIG_PMW3610_REPORT_INTERVAL_MIN;
    bool schedule = have_pending;
    if (schedule) {
        data->report.report_scheduled = true;
    } else {
        schedule = pmw3610_report_prepare_schedule(&data->report, now_ms,
                                                   CONFIG_PMW3610_REPORT_INTERVAL_MIN,
                                                   &delay_ms);
    }
    if (schedule) {
        int schedule_err = k_work_reschedule(&data->report_work, K_MSEC(delay_ms));
        if (schedule_err < 0) {
            data->report.report_scheduled = false;
            LOG_ERR("Failed to reschedule PMW3610 report: %d", schedule_err);
        }
    }
}
#endif

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3610_set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
    pmw3610_set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

    // init device pointer
    data->dev = dev;
    pmw3610_report_accumulator_init(&data->report);
    pmw3610_pointer_accel_reset(&data->acceleration);
    pmw3610_output_init(&data->output);

    // init smart algorithm flag;
    data->sw_smart_flag = false;

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    k_work_init_delayable(&data->report_work, pmw3610_report_work_callback);
#endif

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

static int pmw3610_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case PMW3610_ATTR_CPI:
        err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val),
                              config->swap_xy, config->inv_x, config->inv_y);
        break;

    case PMW3610_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_attr_set,
};

// #if IS_ENABLED(CONFIG_PM_DEVICE)
// static int pmw3610_pm_action(const struct device *dev, enum pm_device_action action) {
//     switch (action) {
//     case PM_DEVICE_ACTION_SUSPEND:
//         return pmw3610_set_interrupt(dev, false);
//     case PM_DEVICE_ACTION_RESUME:
//         return pmw3610_set_interrupt(dev, true);
//     default:
//         return -ENOTSUP;
//     }
// }
// #endif // IS_ENABLED(CONFIG_PM_DEVICE)
// PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_GESTURE_LAYER_2(n)                                                             \
    DT_PROP_OR(DT_DRV_INST(n), pointer_acceleration_gesture_layer_2,                            \
               DT_PROP(DT_DRV_INST(n), pointer_acceleration_gesture_layer))

#define PMW3610_DEFINE(n)                                                                          \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_base_gain_milli) >= 500,             \
                 "Pointer acceleration base gain must be at least 0.5x");                         \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_base_gain_milli) <= 1000,            \
                 "Pointer acceleration base gain must not exceed 1.0x");                          \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_full_speed) >                        \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_takeoff_speed),                  \
                 "Pointer acceleration full speed must exceed takeoff speed");                   \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_max_gain_milli) >=                   \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_base_gain_milli),                \
                 "Pointer acceleration maximum gain must exceed the base gain");                 \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_max_gain_milli) <= 4000,             \
                 "Pointer acceleration maximum gain must not exceed 4.0x");                      \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_reference_interval_ms) > 0,         \
                 "Pointer acceleration reference interval must be positive");                   \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), pointer_acceleration_idle_reset_ms) >                    \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_reference_interval_ms),          \
                 "Pointer acceleration idle reset must exceed the reference interval");          \
    BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_mode) ||                 \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_gain_milli) >= 100,   \
                 "Pointer acceleration precision gain must be at least 0.1x");                  \
    BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_mode) ||                 \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_gain_milli) <=        \
                         DT_PROP(DT_DRV_INST(n), pointer_acceleration_base_gain_milli),            \
                 "Pointer acceleration precision gain must not exceed the base gain");          \
    BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_mode) ||                 \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_speed) <              \
                         DT_PROP(DT_DRV_INST(n), pointer_acceleration_takeoff_speed),              \
                 "Pointer acceleration precision speed must be below takeoff speed");           \
    BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), pointer_acceleration) ||                                \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_scroll_layer) <                  \
                         ZMK_KEYMAP_LAYERS_LEN,                                                    \
                 "Pointer acceleration Scroll layer must exist");                               \
    BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), pointer_acceleration) ||                                \
                     DT_PROP(DT_DRV_INST(n), pointer_acceleration_gesture_layer) <                 \
                         ZMK_KEYMAP_LAYERS_LEN,                                                    \
                 "Pointer acceleration Gesture layer must exist");                              \
    BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), pointer_acceleration) ||                                \
                     PMW3610_GESTURE_LAYER_2(n) < ZMK_KEYMAP_LAYERS_LEN,                          \
                 "Pointer acceleration Gesture layer 2 must exist");                            \
    static struct pixart_data data##n;                                                             \
    static const struct pixart_config config##n = {                                                \
		.spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),		                               \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \
        .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                                               \
        .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                                                \
        .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                                                \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                                     \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                                       \
        .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),                     \
        .acceleration_enabled = DT_PROP(DT_DRV_INST(n), pointer_acceleration),                     \
        .acceleration_scroll_layer =                                                              \
            DT_PROP(DT_DRV_INST(n), pointer_acceleration_scroll_layer),                            \
        .acceleration_gesture_layer =                                                             \
            DT_PROP(DT_DRV_INST(n), pointer_acceleration_gesture_layer),                           \
        .acceleration_gesture_layer_2 = PMW3610_GESTURE_LAYER_2(n),                               \
        .acceleration_curve =                                                                     \
            {                                                                                      \
                .base_gain_milli =                                                                \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_base_gain_milli),                 \
                .takeoff_speed =                                                                  \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_takeoff_speed),                   \
                .full_speed =                                                                     \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_full_speed),                      \
                .max_gain_milli =                                                                 \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_max_gain_milli),                  \
                .reference_interval_ms =                                                          \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_reference_interval_ms),           \
                .idle_reset_ms =                                                                  \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_idle_reset_ms),                   \
                .precision_enabled =                                                              \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_mode),                  \
                .precision_gain_milli =                                                          \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_gain_milli),            \
                .precision_speed =                                                               \
                    DT_PROP(DT_DRV_INST(n), pointer_acceleration_precision_speed),                 \
            },                                                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_INPUT_PMW3610_INIT_PRIORITY, &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)


#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610, GET_PMW3610_DEV)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        pmw3610_set_performance(pmw3610_devs[i], enable);
    }

    return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
