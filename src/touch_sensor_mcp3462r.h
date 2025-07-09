#define READ_CMD 0b01000011 // Read command for MCP3462R
#define BUFFER_SIZE 15

static uint_fast8_t periodic_read_event(struct timer *t);

struct mcp3462r_adc {
    uint8_t oid;
    struct spidev_s *spi;
    struct gpio_in adc_ready_pin;
    struct gpio_out trigger_out_pin;
    struct gpio_out PI_EN_pin;
    uint32_t rest_ticks, timeout_cycles;
    struct timer timer;
    uint8_t active_session_flag, configured_flag;
    uint8_t msg[3];
    uint16_t sensitivity; 
};

struct RollingAverage {
    uint32_t size;
    uint32_t index;
    float buffer[BUFFER_SIZE];
    uint32_t count;
    float sum;
    float lst_avg_value;
    struct timer timer;
    uint32_t rest_ticks;
    char running_flag;
};

