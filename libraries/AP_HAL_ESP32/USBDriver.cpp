/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL_ESP32/USBDriver.h>
#include <AP_Math/AP_Math.h>

#include "esp_log.h"

extern const AP_HAL::HAL& hal;

static ByteBuffer* static_readbuf;

namespace ESP32
{

void USBDriver::vprintf(const char *fmt, va_list ap)
{

    // if (1 == 1) {
         esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    // } else {
    //     AP_HAL::UARTDriver::vprintf(fmt, ap);
    // }
}

static void IRAM_ATTR cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    uint8_t buff[64];

    esp_err_t ret = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, buff, sizeof(buff), &rx_size);
    if (ret == ESP_OK) {
        if (rx_size > 0) {
             static_readbuf->write(buff, rx_size);
            }
    } 
}


void USBDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
        if (!_initialized) {
            tinyusb_config_t tusb_cfg = {}; // the configuration using default values
            tinyusb_driver_install(&tusb_cfg);

            tinyusb_config_cdcacm_t acm_cfg = {
                .usb_dev = TINYUSB_USBDEV_0,
                .cdc_port = TINYUSB_CDC_ACM_0,
                .rx_unread_buf_sz = 512,
                .callback_rx = (tusb_cdcacm_callback_t)&cdc_rx_callback, // NULL, // the first way to register a callback
                .callback_rx_wanted_char = NULL,
                .callback_line_state_changed = NULL,
                .callback_line_coding_changed = NULL
            };

            tusb_cdc_acm_init(&acm_cfg);
            itf = (tinyusb_cdcacm_itf_t)0;

            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);

            static_readbuf = &_readbuf;
            
            _initialized = true;

            printf("USBDriver begin");
        } else {
            flush();
        }
    _baudrate = b;
}

void USBDriver::_end()
{
    if (_initialized) {
       // uart_driver_delete(uart_desc[uart_num].port);
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
     printf("USBDriver end");
}

void USBDriver::_flush()
{
    if (!_initialized) {
        return;
    }
        tinyusb_cdcacm_write_flush(itf, 0);
}

bool USBDriver::is_initialized()
{
    return _initialized;
}

bool USBDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


uint32_t USBDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t USBDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

ssize_t IRAM_ATTR USBDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }


    _receive_timestamp_update();

    return ret;
}

void IRAM_ATTR USBDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    //read_data();
    write_data();
}

void IRAM_ATTR USBDriver::read_data()
{
    size_t rx_size = 0;
    do {
        rx_size = 0;
        esp_err_t ret = tinyusb_cdcacm_read(itf, _buffer, sizeof(_buffer), &rx_size);
        if ((rx_size > 0) && (ret == ESP_OK)) {
                _readbuf.write(_buffer, rx_size);
                } else break;
    } while (rx_size > 0);
}

void IRAM_ATTR USBDriver::write_data()
{
     _write_mutex.take_blocking();
     size_t count = 0;
     do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            tinyusb_cdcacm_write_queue(itf, (const uint8_t*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR USBDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();

    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool USBDriver::_discard_input()
{
    //uart_port_t p = uart_desc[uart_num].port;
    //return uart_flush_input(p) == ESP_OK;
    return false;
}

// record timestamp of new incoming data
void IRAM_ATTR USBDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}


/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  A return value of zero means the HAL does not support this API
*/
uint64_t USBDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

}
