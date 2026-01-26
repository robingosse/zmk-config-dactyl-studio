/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2024 Integrated Project - PS/2 GPIO Driver
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_gpio_ps2

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ps2_gpio, CONFIG_PS2_LOG_LEVEL);

struct ps2_gpio_config {
    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;
};

struct ps2_gpio_data {
    ps2_callback_t callback_isr;
    struct k_sem tx_lock;
    struct gpio_callback scl_cb_data;
    struct gpio_callback sda_cb_data;
    uint8_t bits_count;
};

static void ps2_gpio_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
}

static int ps2_gpio_configure(const struct device *dev, ps2_callback_t callback_isr) {
    struct ps2_gpio_data *data = dev->data;
    data->callback_isr = callback_isr;
    return 0;
}

static int ps2_gpio_write(const struct device *dev, uint8_t value) {
    ARG_UNUSED(dev);
    ARG_UNUSED(value);
    return -ENOTSUP;
}

static int ps2_gpio_enable_callback(const struct device *dev) {
    const struct ps2_gpio_config *config = dev->config;
    
    gpio_pin_interrupt_configure_dt(&config->scl_gpio, GPIO_INT_EDGE_FALLING);
    
    return 0;
}

static int ps2_gpio_disable_callback(const struct device *dev) {
    const struct ps2_gpio_config *config = dev->config;
    
    gpio_pin_interrupt_configure_dt(&config->scl_gpio, GPIO_INT_DISABLE);
    
    return 0;
}

static const struct ps2_driver_api ps2_gpio_driver_api = {
    .config = ps2_gpio_configure,
    .write = ps2_gpio_write,
    .enable_callback = ps2_gpio_enable_callback,
    .disable_callback = ps2_gpio_disable_callback,
};

static int ps2_gpio_init(const struct device *dev) {
    const struct ps2_gpio_config *config = dev->config;
    struct ps2_gpio_data *data = dev->data;
    int ret;

    if (!device_is_ready(config->scl_gpio.port)) {
        LOG_ERR("SCL GPIO device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(config->sda_gpio.port)) {
        LOG_ERR("SDA GPIO device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->scl_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure SCL GPIO");
        return ret;
    }

    ret = gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure SDA GPIO");
        return ret;
    }

    gpio_init_callback(&data->scl_cb_data, ps2_gpio_isr, BIT(config->scl_gpio.pin));
    gpio_add_callback(config->scl_gpio.port, &data->scl_cb_data);

    k_sem_init(&data->tx_lock, 1, 1);

    return 0;
}

#define PS2_GPIO_INIT(inst)                                                                        \
    static const struct ps2_gpio_config ps2_gpio_config_##inst = {                                 \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(inst, clk_gpios),                                        \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(inst, data_gpios),                                       \
    };                                                                                             \
                                                                                                   \
    static struct ps2_gpio_data ps2_gpio_data_##inst;                                              \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, ps2_gpio_init, NULL, &ps2_gpio_data_##inst,                       \
                          &ps2_gpio_config_##inst, POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,          \
                          &ps2_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_GPIO_INIT)
