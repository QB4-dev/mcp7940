/*
 * Copyright (c) 2026 Jakub Turek <qb4.dev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mcp7940.h
 * @defgroup mcp7940 mcp7940
 * @{
 *
 * ESP-IDF driver for mcp7940 real-time clock
 *
 * Copyright (c) 2026 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP7940_H__
#define __MCP7940_H__

#include <stdbool.h>
#include <time.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP7940_ADDR 0x6F //!< I2C address

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_free_desc(i2c_dev_t *dev);

/**
 * @brief Start/stop clock
 *
 * @param dev Device descriptor
 * @param start Start clock if true
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_start(i2c_dev_t *dev, bool start);

/**
 * @brief Get current clock state
 *
 * @param dev Device descriptor
 * @param[out] running true if clock running
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_is_running(i2c_dev_t *dev, bool *running);

/**
 * @brief Get current time
 *
 * @param dev Device descriptor
 * @param[out] time Pointer to the time struct to fill
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_get_time(i2c_dev_t *dev, struct tm *time);

/**
 * @brief Set time to RTC
 *
 * @param dev Device descriptor
 * @param[in] time Pointer to the time struct
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_set_time(i2c_dev_t *dev, const struct tm *time);

/**
 * @brief Read RAM contents into the buffer
 *
 * @param dev Device descriptor
 * @param offset Start byte, 0..55
 * @param[out] buf Buffer to store data
 * @param len Bytes to read, 1..56
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_read_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len);

/**
 * @brief Write buffer to RTC RAM
 *
 * @param dev Device descriptor
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to write, 1..56
 * @return `ESP_OK` on success
 */
esp_err_t mcp7940_write_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MCP7940_H__ */
