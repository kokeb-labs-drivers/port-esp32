/*
 * Host fake of the ESP-IDF v5.5 declarations the port's boundary files use: the same names, types
 * and signatures (a subset), so the boundary files compile on the host unchanged and link against
 * tests/fakes/esp-idf/fake_esp_idf.cpp (a link seam per SDK call). Every public SDK header the
 * boundary files include is a one-line file in this folder that includes this one.
 *
 * Variants (compile definitions): KB_FAKE_NO_TWAI (a chip without TWAI), KB_FAKE_LINE_FITTING (the
 * ESP32 calibration scheme instead of curve fitting, with CONFIG_IDF_TARGET_ESP32), KB_FAKE_IDF_V6
 * (ESP-IDF 6.0: a failed I2C transfer returns ESP_ERR_INVALID_RESPONSE), KB_FAKE_TICK_100 (a
 * 100 Hz RTOS tick, ESP-IDF's default).
 *
 * Test code. SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_FAKE_ESP_IDF_H
#define KB_FAKE_ESP_IDF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* --- soc_caps.h ------------------------------------------------------------------------------- */
#define SOC_HP_I2C_NUM (2U)
#define SOC_UART_HP_NUM (3)
#define SOC_UART_FIFO_LEN (128)
#define SOC_SPI_PERIPH_NUM 3
#define SOC_SPI_MAXIMUM_BUFFER_SIZE 64
#define SOC_LEDC_CHANNEL_NUM (8)
#define SOC_LEDC_TIMER_BIT_WIDTH (14)
#define SOC_ADC_RTC_MAX_BITWIDTH (12)
#define SOC_ADC_PERIPH_NUM (2)
#define SOC_GPIO_PIN_COUNT 49
#ifndef KB_FAKE_NO_TWAI
#define SOC_TWAI_SUPPORTED 1
#define SOC_TWAI_CONTROLLER_NUM 2
#endif
#ifdef KB_FAKE_LINE_FITTING
#define ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED 1
#define CONFIG_IDF_TARGET_ESP32 1
#else
#define ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED 1
#endif

/* --- esp_idf_version.h ------------------------------------------------------------------------ */
#define ESP_IDF_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#ifdef KB_FAKE_IDF_V6
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(6, 0, 0)
#else
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(5, 5, 6)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* --- esp_err.h -------------------------------------------------------------------------------- */
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
#define ESP_ERR_INVALID_CRC 0x109
#define ESP_ERR_NOT_FINISHED 0x10C
#define ESP_ERR_NOT_ALLOWED 0x10D
#define ESP_INTR_FLAG_LEVEL1 (1 << 1)

/* --- FreeRTOS --------------------------------------------------------------------------------- */
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
#ifdef KB_FAKE_TICK_100
#define configTICK_RATE_HZ 100U
#else
#define configTICK_RATE_HZ 1000U
#endif
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFU)
typedef struct fake_queue* QueueHandle_t;
BaseType_t xQueueReceive(QueueHandle_t queue, void* item, TickType_t ticks);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue);

/* --- hal/i2c_ll.h, esp_clk_tree.h ------------------------------------------------------------- */
#define I2C_LL_SCL_WAIT_US_VAL_DEFAULT (2000)
#define I2C_LL_MAX_TIMEOUT_PERIOD (1UL << 22) /* 104 ms at the fake's 40 MHz I2C clock */
typedef int soc_module_clk_t;
typedef enum { ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED = 0 } esp_clk_tree_src_freq_precision_t;
esp_err_t esp_clk_tree_src_get_freq_hz(soc_module_clk_t clk_src,
                                       esp_clk_tree_src_freq_precision_t precision, uint32_t* freq);

/* --- esp_memory_utils.h, esp_private/esp_cache_private.h -------------------------------------- */
#define MALLOC_CAP_DMA (1U << 3)
#define MALLOC_CAP_INTERNAL (1U << 11)
void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps);
void heap_caps_free(void* ptr);
/* soc/soc.h: the IRAM range (fake::IramAll(true) makes every address IRAM). */
uintptr_t fake_iram_low(void);
uintptr_t fake_iram_high(void);
#define SOC_IRAM_LOW (fake_iram_low())
#define SOC_IRAM_HIGH (fake_iram_high())
esp_err_t esp_cache_get_alignment(uint32_t heap_caps, size_t* out_alignment);
void vTaskDelay(TickType_t ticks);

/* --- gpio ------------------------------------------------------------------------------------- */
typedef enum { GPIO_NUM_NC = -1, GPIO_NUM_0 = 0, GPIO_NUM_MAX = SOC_GPIO_PIN_COUNT } gpio_num_t;
typedef enum {
  GPIO_MODE_DISABLE = 0,
  GPIO_MODE_INPUT = 1,
  GPIO_MODE_OUTPUT = 2,
  GPIO_MODE_OUTPUT_OD = 6,
  GPIO_MODE_INPUT_OUTPUT_OD = 7,
  GPIO_MODE_INPUT_OUTPUT = 3
} gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE = 1 } gpio_pulldown_t;
typedef enum {
  GPIO_INTR_DISABLE = 0,
  GPIO_INTR_POSEDGE = 1,
  GPIO_INTR_NEGEDGE = 2,
  GPIO_INTR_ANYEDGE = 3
} gpio_int_type_t;
typedef enum {
  GPIO_PULLUP_ONLY = 0,
  GPIO_PULLDOWN_ONLY = 1,
  GPIO_PULLUP_PULLDOWN = 2,
  GPIO_FLOATING = 3
} gpio_pull_mode_t;
typedef struct {
  uint64_t pin_bit_mask;
  gpio_mode_t mode;
  gpio_pullup_t pull_up_en;
  gpio_pulldown_t pull_down_en;
  gpio_int_type_t intr_type;
} gpio_config_t;
typedef void (*gpio_isr_t)(void* arg);
/* Pins 0 to 48 are GPIOs; 34 to 39 are input only (as on the ESP32). */
#define GPIO_IS_VALID_GPIO(n) (((n) >= 0) && ((n) < SOC_GPIO_PIN_COUNT))
#define GPIO_IS_VALID_OUTPUT_GPIO(n) (GPIO_IS_VALID_GPIO(n) && (((n) < 34) || ((n) > 39)))
esp_err_t gpio_config(const gpio_config_t* config);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t pull);
esp_err_t gpio_reset_pin(gpio_num_t pin);
esp_err_t gpio_install_isr_service(int flags);
esp_err_t gpio_isr_handler_add(gpio_num_t pin, gpio_isr_t handler, void* args);
esp_err_t gpio_isr_handler_remove(gpio_num_t pin);
esp_err_t gpio_set_intr_type(gpio_num_t pin, gpio_int_type_t type);
esp_err_t gpio_intr_enable(gpio_num_t pin);
esp_err_t gpio_intr_disable(gpio_num_t pin);

/* --- i2c_master ------------------------------------------------------------------------------- */
typedef int i2c_port_num_t;
typedef enum { I2C_CLK_SRC_DEFAULT = 4 } i2c_clock_source_t;
typedef enum { I2C_ADDR_BIT_LEN_7 = 0 } i2c_addr_bit_len_t;
typedef struct i2c_master_bus_t* i2c_master_bus_handle_t;
typedef struct i2c_master_dev_t* i2c_master_dev_handle_t;
typedef struct {
  i2c_port_num_t i2c_port;
  gpio_num_t sda_io_num;
  gpio_num_t scl_io_num;
  union {
    i2c_clock_source_t clk_source;
  };
  uint8_t glitch_ignore_cnt;
  int intr_priority;
  size_t trans_queue_depth;
  struct {
    uint32_t enable_internal_pullup : 1;
    uint32_t allow_pd : 1;
  } flags;
} i2c_master_bus_config_t;
typedef struct {
  i2c_addr_bit_len_t dev_addr_length;
  uint16_t device_address;
  uint32_t scl_speed_hz;
  uint32_t scl_wait_us;
  struct {
    uint32_t disable_ack_check : 1;
  } flags;
} i2c_device_config_t;
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t* config, i2c_master_bus_handle_t* bus);
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t* config,
                                    i2c_master_dev_handle_t* dev);
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t dev);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t* tx, size_t tx_size,
                              int timeout_ms);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev, const uint8_t* tx,
                                      size_t tx_size, uint8_t* rx, size_t rx_size, int timeout_ms);
esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev, uint8_t* rx, size_t rx_size,
                             int timeout_ms);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address, int timeout_ms);
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus);
esp_err_t i2c_master_device_change_address(i2c_master_dev_handle_t dev, uint16_t address,
                                           int timeout_ms);

/* --- spi_master ------------------------------------------------------------------------------- */
typedef enum { SPI1_HOST = 0, SPI2_HOST = 1, SPI3_HOST = 2 } spi_host_device_t;
typedef enum { SPI_DMA_DISABLED = 0, SPI_DMA_CH_AUTO = 3 } spi_dma_chan_t;
#define SPICOMMON_BUSFLAG_MASTER (1U << 0)
#define SPI_TRANS_CS_KEEP_ACTIVE (1 << 8)
typedef struct {
  int mosi_io_num;
  int miso_io_num;
  int sclk_io_num;
  int quadwp_io_num;
  int quadhd_io_num;
  int max_transfer_sz;
  uint32_t flags;
} spi_bus_config_t;
typedef struct {
  uint8_t mode;
  uint16_t cs_ena_pretrans;
  uint8_t cs_ena_posttrans;
  int clock_speed_hz;
  int spics_io_num;
  uint32_t flags;
  int queue_size;
} spi_device_interface_config_t;
typedef struct {
  uint32_t flags;
  size_t length;
  size_t rxlength;
  const void* tx_buffer;
  void* rx_buffer;
} spi_transaction_t;
typedef struct spi_device_t* spi_device_handle_t;
esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t* config,
                             spi_dma_chan_t dma);
esp_err_t spi_bus_free(spi_host_device_t host);
esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t* config,
                             spi_device_handle_t* handle);
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);
esp_err_t spi_device_polling_start(spi_device_handle_t handle, spi_transaction_t* t,
                                   TickType_t ticks);
esp_err_t spi_device_polling_end(spi_device_handle_t handle, TickType_t ticks);
esp_err_t spi_device_acquire_bus(spi_device_handle_t handle, TickType_t ticks);
esp_err_t spi_device_get_actual_freq(spi_device_handle_t handle, int* freq_khz);
void spi_device_release_bus(spi_device_handle_t handle);

/* --- uart ------------------------------------------------------------------------------------- */
typedef int uart_port_t;
typedef enum { UART_DATA_5_BITS = 0, UART_DATA_8_BITS = 3 } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE = 0, UART_PARITY_EVEN = 2, UART_PARITY_ODD = 3 } uart_parity_t;
typedef enum { UART_STOP_BITS_1 = 1, UART_STOP_BITS_2 = 3 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE = 0, UART_HW_FLOWCTRL_CTS = 2 } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT = 4 } uart_sclk_t;
typedef enum { UART_MODE_UART = 0, UART_MODE_RS485_HALF_DUPLEX = 1 } uart_mode_t;
typedef enum {
  UART_DATA,
  UART_BREAK,
  UART_BUFFER_FULL,
  UART_FIFO_OVF,
  UART_FRAME_ERR,
  UART_PARITY_ERR,
  UART_DATA_BREAK,
  UART_PATTERN_DET,
  UART_EVENT_MAX
} uart_event_type_t;
typedef struct {
  uart_event_type_t type;
  size_t size;
  int timeout_flag;
} uart_event_t;
typedef struct {
  int baud_rate;
  uart_word_length_t data_bits;
  uart_parity_t parity;
  uart_stop_bits_t stop_bits;
  uart_hw_flowcontrol_t flow_ctrl;
  uint8_t rx_flow_ctrl_thresh;
  uart_sclk_t source_clk;
} uart_config_t;
esp_err_t uart_driver_install(uart_port_t port, int rx_buffer_size, int tx_buffer_size,
                              int queue_size, QueueHandle_t* queue, int intr_alloc_flags);
esp_err_t uart_driver_delete(uart_port_t port);
esp_err_t uart_param_config(uart_port_t port, const uart_config_t* config);
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
esp_err_t uart_set_mode(uart_port_t port, uart_mode_t mode);
int uart_tx_chars(uart_port_t port, const char* buffer, uint32_t len);
int uart_read_bytes(uart_port_t port, void* buf, uint32_t length, TickType_t ticks);
esp_err_t uart_get_buffered_data_len(uart_port_t port, size_t* size);
esp_err_t uart_flush_input(uart_port_t port);
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud);
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t ticks);

/* --- ledc ------------------------------------------------------------------------------------- */
typedef enum { LEDC_LOW_SPEED_MODE = 0 } ledc_mode_t;
typedef enum { LEDC_CHANNEL_0 = 0, LEDC_CHANNEL_MAX = 8 } ledc_channel_t;
typedef enum { LEDC_TIMER_0 = 0, LEDC_TIMER_MAX = 4 } ledc_timer_t;
typedef enum { LEDC_TIMER_1_BIT = 1, LEDC_TIMER_14_BIT = 14 } ledc_timer_bit_t;
typedef enum { LEDC_AUTO_CLK = 0 } ledc_clk_cfg_t;
typedef enum { LEDC_INTR_DISABLE = 0 } ledc_intr_type_t;
typedef struct {
  ledc_mode_t speed_mode;
  ledc_timer_bit_t duty_resolution;
  ledc_timer_t timer_num;
  uint32_t freq_hz;
  ledc_clk_cfg_t clk_cfg;
  int deconfigure;
} ledc_timer_config_t;
typedef struct {
  int gpio_num;
  ledc_mode_t speed_mode;
  ledc_channel_t channel;
  ledc_intr_type_t intr_type;
  ledc_timer_t timer_sel;
  uint32_t duty;
  int hpoint;
  struct {
    unsigned int output_invert : 1;
  } flags;
} ledc_channel_config_t;
esp_err_t ledc_timer_config(const ledc_timer_config_t* config);
esp_err_t ledc_channel_config(const ledc_channel_config_t* config);
esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t channel);
esp_err_t ledc_stop(ledc_mode_t mode, ledc_channel_t channel, uint32_t idle_level);

/* --- esp_adc ---------------------------------------------------------------------------------- */
typedef enum { ADC_UNIT_1 = 0, ADC_UNIT_2 = 1 } adc_unit_t;
typedef enum {
  ADC_CHANNEL_0 = 0,
  ADC_CHANNEL_1,
  ADC_CHANNEL_2,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_5,
  ADC_CHANNEL_6,
  ADC_CHANNEL_7,
  ADC_CHANNEL_8,
  ADC_CHANNEL_9
} adc_channel_t;
typedef enum {
  ADC_ATTEN_DB_0 = 0,
  ADC_ATTEN_DB_2_5 = 1,
  ADC_ATTEN_DB_6 = 2,
  ADC_ATTEN_DB_12 = 3
} adc_atten_t;
typedef enum { ADC_BITWIDTH_DEFAULT = 0 } adc_bitwidth_t;
typedef enum { ADC_ULP_MODE_DISABLE = 0 } adc_ulp_mode_t;
typedef int adc_oneshot_clk_src_t;
typedef struct adc_oneshot_unit_ctx_t* adc_oneshot_unit_handle_t;
typedef struct adc_cali_scheme_t* adc_cali_handle_t;
typedef struct {
  adc_unit_t unit_id;
  adc_oneshot_clk_src_t clk_src;
  adc_ulp_mode_t ulp_mode;
} adc_oneshot_unit_init_cfg_t;
typedef struct {
  adc_atten_t atten;
  adc_bitwidth_t bitwidth;
} adc_oneshot_chan_cfg_t;
typedef struct {
  adc_unit_t unit_id;
  adc_channel_t chan;
  adc_atten_t atten;
  adc_bitwidth_t bitwidth;
} adc_cali_curve_fitting_config_t;
typedef struct {
  adc_unit_t unit_id;
  adc_atten_t atten;
  adc_bitwidth_t bitwidth;
  uint32_t default_vref;
} adc_cali_line_fitting_config_t;
esp_err_t adc_oneshot_io_to_channel(int io_num, adc_unit_t* unit, adc_channel_t* channel);
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* config,
                               adc_oneshot_unit_handle_t* handle);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel,
                                     const adc_oneshot_chan_cfg_t* config);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t channel, int* raw);
esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle);
esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t* config,
                                               adc_cali_handle_t* handle);
esp_err_t adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t handle);
esp_err_t adc_cali_create_scheme_line_fitting(const adc_cali_line_fitting_config_t* config,
                                              adc_cali_handle_t* handle);
esp_err_t adc_cali_delete_scheme_line_fitting(adc_cali_handle_t handle);
esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle, int raw, int* voltage);

/* --- twai ------------------------------------------------------------------------------------- */
typedef struct twai_obj_t* twai_handle_t;
typedef enum { TWAI_MODE_NORMAL = 0, TWAI_MODE_NO_ACK = 1, TWAI_MODE_LISTEN_ONLY = 2 } twai_mode_t;
typedef enum {
  TWAI_STATE_STOPPED = 0,
  TWAI_STATE_RUNNING = 1,
  TWAI_STATE_BUS_OFF = 2,
  TWAI_STATE_RECOVERING = 3
} twai_state_t;
typedef enum { TWAI_CLK_SRC_DEFAULT = 4 } twai_clock_source_t;
#define TWAI_IO_UNUSED ((gpio_num_t) - 1)
#define TWAI_ALERT_NONE 0x00000000U
#define TWAI_MSG_FLAG_EXTD 0x01U
#define TWAI_MSG_FLAG_RTR 0x02U
typedef struct {
  int controller_id;
  twai_mode_t mode;
  gpio_num_t tx_io;
  gpio_num_t rx_io;
  gpio_num_t clkout_io;
  gpio_num_t bus_off_io;
  uint32_t tx_queue_len;
  uint32_t rx_queue_len;
  uint32_t alerts_enabled;
  uint32_t clkout_divider;
  int intr_flags;
} twai_general_config_t;
typedef struct {
  twai_clock_source_t clk_src;
  uint32_t quanta_resolution_hz;
  uint32_t brp;
  uint8_t prop_seg;
  uint8_t tseg_1;
  uint8_t tseg_2;
  uint8_t sjw;
  int8_t ssp_offset;
  bool triple_sampling;
} twai_timing_config_t;
typedef struct {
  uint32_t acceptance_code;
  uint32_t acceptance_mask;
  bool single_filter;
} twai_filter_config_t;
typedef struct {
  uint32_t flags;
  uint32_t identifier;
  uint8_t data_length_code;
  uint8_t data[8];
} twai_message_t;
typedef struct {
  twai_state_t state;
  uint32_t msgs_to_tx;
  uint32_t msgs_to_rx;
  uint32_t tx_error_counter;
  uint32_t rx_error_counter;
  uint32_t tx_failed_count;
  uint32_t rx_missed_count;
  uint32_t rx_overrun_count;
  uint32_t arb_lost_count;
  uint32_t bus_error_count;
} twai_status_info_t;
#define KB_FAKE_TWAI_TIMING(res, t1, t2, j)                                                        \
  {.clk_src = TWAI_CLK_SRC_DEFAULT,                                                                \
   .quanta_resolution_hz = (res),                                                                  \
   .brp = 0,                                                                                       \
   .prop_seg = 0,                                                                                  \
   .tseg_1 = (t1),                                                                                 \
   .tseg_2 = (t2),                                                                                 \
   .sjw = (j),                                                                                     \
   .ssp_offset = 0,                                                                                \
   .triple_sampling = false}
#define TWAI_TIMING_CONFIG_25KBITS() KB_FAKE_TWAI_TIMING(500000, 15, 4, 3)
#define TWAI_TIMING_CONFIG_50KBITS() KB_FAKE_TWAI_TIMING(1000000, 15, 4, 3)
#define TWAI_TIMING_CONFIG_100KBITS() KB_FAKE_TWAI_TIMING(2000000, 15, 4, 3)
#define TWAI_TIMING_CONFIG_125KBITS() KB_FAKE_TWAI_TIMING(2000000, 11, 4, 3)
#define TWAI_TIMING_CONFIG_250KBITS() KB_FAKE_TWAI_TIMING(4000000, 11, 4, 2)
#define TWAI_TIMING_CONFIG_500KBITS() KB_FAKE_TWAI_TIMING(10000000, 15, 4, 3)
#define TWAI_TIMING_CONFIG_800KBITS() KB_FAKE_TWAI_TIMING(20000000, 16, 8, 3)
#define TWAI_TIMING_CONFIG_1MBITS() KB_FAKE_TWAI_TIMING(20000000, 15, 4, 3)
#define TWAI_FILTER_CONFIG_ACCEPT_ALL()                                                            \
  {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true}
esp_err_t twai_driver_install_v2(const twai_general_config_t* general,
                                 const twai_timing_config_t* timing,
                                 const twai_filter_config_t* filter, twai_handle_t* handle);
esp_err_t twai_driver_uninstall_v2(twai_handle_t handle);
esp_err_t twai_start_v2(twai_handle_t handle);
esp_err_t twai_stop_v2(twai_handle_t handle);
esp_err_t twai_transmit_v2(twai_handle_t handle, const twai_message_t* message, TickType_t ticks);
esp_err_t twai_receive_v2(twai_handle_t handle, twai_message_t* message, TickType_t ticks);
esp_err_t twai_get_status_info_v2(twai_handle_t handle, twai_status_info_t* info);
esp_err_t twai_initiate_recovery_v2(twai_handle_t handle);

/* --- esp_timer, esp_rom, esp_log -------------------------------------------------------------- */
int64_t esp_timer_get_time(void);
void esp_rom_delay_us(uint32_t us);
typedef enum {
  ESP_LOG_NONE = 0,
  ESP_LOG_ERROR = 1,
  ESP_LOG_WARN = 2,
  ESP_LOG_INFO = 3,
  ESP_LOG_DEBUG = 4
} esp_log_level_t;
uint32_t esp_log_timestamp(void);
void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif /* KB_FAKE_ESP_IDF_H */
