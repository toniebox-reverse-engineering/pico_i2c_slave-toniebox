#include <i2c_fifo.h>
#include <i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

/*
#include "pico/sleep.h"
#include "hardware/rtc.h"
*/

static const uint I2C_SLAVE_ADDRESS = 0x1D;

static const uint HEADSDOWN_PIN = 2;
static const uint IRQ_PIN = 3;
static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5
static const uint TAP_LEFT_PIN = 7;
static const uint TAP_RIGHT_PIN = 6;
static const uint TILT_LEFT_PIN = 8;
static const uint TILT_RIGHT_PIN = 9;
static const uint TAP_N_TILT_FWD_PIN = 10;
static const uint TAP_N_TILT_BWD_PIN = 11;
static const uint EAR_LEFT_PIN = 12;
static const uint EAR_RIGHT_PIN = 13;
static const uint LED_PIN = 25;
static const uint SLEEP_PIN = 14;

#define REGISTER_COUNT 0xff
#define MSG_COUNT      0xff

enum MMA8452Q_Register
{
    STATUS_MMA8452Q = 0x00,
    OUT_X_MSB = 0x01,
    OUT_X_LSB = 0x02,
    OUT_Y_MSB = 0x03,
    OUT_Y_LSB = 0x04,
    OUT_Z_MSB = 0x05,
    OUT_Z_LSB = 0x06,
    F_SETUP = 0x09,
    SYSMOD = 0x0B,
    INT_SOURCE = 0x0C,
    WHO_AM_I = 0x0D,
    XYZ_DATA_CFG = 0x0E,
    HP_FILTER_CUTOFF = 0x0F,
    PL_STATUS = 0x10,
    PL_CFG = 0x11,
    PL_COUNT = 0x12,
    PL_BF_ZCOMP = 0x13,
    P_L_THS_REG = 0x14,
    FF_MT_CFG = 0x15,
    FF_MT_SRC = 0x16,
    FF_MT_THS = 0x17,
    FF_MT_COUNT = 0x18,
    TRANSIENT_CFG = 0x1D,
    TRANSIENT_SRC = 0x1E,
    TRANSIENT_THS = 0x1F,
    TRANSIENT_COUNT = 0x20,
    PULSE_CFG = 0x21,
    PULSE_SRC = 0x22,
    PULSE_THSX = 0x23,
    PULSE_THSY = 0x24,
    PULSE_THSZ = 0x25,
    PULSE_TMLT = 0x26,
    PULSE_LTCY = 0x27,
    PULSE_WIND = 0x28,
    ASLP_COUNT = 0x29,
    CTRL_REG1 = 0x2A,
    CTRL_REG2 = 0x2B,
    CTRL_REG3 = 0x2C,
    CTRL_REG4 = 0x2D,
    CTRL_REG5 = 0x2E,
    OFF_X = 0x2F,
    OFF_Y = 0x30,
    OFF_Z = 0x31,

    FIFO_BUF = 0x81,
    FIFO_END = REGISTER_COUNT
};

typedef enum
{
    MSG_NONE,
    MSG_START,
    MSG_READ,
    MSG_WRITE,
    MSG_STOP,
    MSG_TIMEOUT
} ctx_msg_type;

typedef struct
{
    ctx_msg_type type;
    uint8_t val1;
    uint8_t val2;
} ctx_msg;

static struct
{
    uint8_t mem[REGISTER_COUNT];
    uint8_t mem_address;
    uint8_t mem_f_bit;
    bool mem_address_written;

    ctx_msg msg_buffer[MSG_COUNT];
    uint8_t msg_read;
    uint8_t msg_write;
} ctx;
#define f_read ((ctx.mem[CTRL_REG1] & 0x02) == 0x02)
#define fmode  ((ctx.mem[F_SETUP] & 0xC0) == 0xC0)

static uint32_t millis() {
    return (time_us_64()/1000);
}

static void push_msg(ctx_msg_type type, uint8_t val1, uint8_t val2) {
    if ((ctx.msg_write + 1) % MSG_COUNT != ctx.msg_read) {
        ctx.msg_buffer[ctx.msg_write].type = type;
        ctx.msg_buffer[ctx.msg_write].val1 = val1;
        ctx.msg_buffer[ctx.msg_write].val2 = val2;
        ctx.msg_write = (ctx.msg_write + 1) % MSG_COUNT;
    } else {
        // Buffer is full, handle error or discard the message
    }
}
static ctx_msg pull_msg() {
    ctx_msg msg;
    if (ctx.msg_read != ctx.msg_write) {
        msg = ctx.msg_buffer[ctx.msg_read];
        ctx.msg_read = (ctx.msg_read + 1) % MSG_COUNT;
        return msg;
    } else {
        // Buffer is empty
        msg.type = MSG_NONE;
        return msg;
    }
}

// This example blinks the Pico LED when the BOOTSEL button is pressed.
//
// Picoboard has a button attached to the flash CS pin, which the bootrom
// checks, and jumps straight to the USB bootcode if the button is pressed
// (pulling flash CS low). We can check this pin in by jumping to some code in
// SRAM (so that the XIP interface is not required), floating the flash CS
// pin, and observing whether it is pulled low.
//
// This doesn't work if others are trying to access flash at the same time,
// e.g. XIP streamer, or the other core.

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
        GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i)
        ;

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
        GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}
static uint8_t auto_increment(uint8_t reg) {
    switch (reg) {
    case OUT_X_MSB:
        if (fmode == 0) {
            if (f_read == 0) {
                reg++;
            } else {
                reg = 0x3;
            }
        }
        break;
    case OUT_X_LSB:
    case OUT_Y_LSB:
    case OUT_Z_MSB:
        if (f_read == 0) {
            reg++;
        } else {
            reg = 0x0;
        }
        break;
    case OUT_Y_MSB:
        if (f_read == 0) {
            reg++;
        } else {
            if (fmode == 0) {
                reg = 0x05;
            } else {
                reg = 0x0;
            }
        }
        break;
    case OUT_Z_LSB:
        reg = 0x00;
        break;
    case OFF_Z:
        reg = 0x0D;
        break;
    case FIFO_END:
        reg = FIFO_BUF;
    default:
        reg++;
    }
    return reg;
}

static void reset_irq() {
    gpio_put(IRQ_PIN, false);
}
static void set_irq() {
    gpio_put(IRQ_PIN, true);
}

bool led_state;
static void led(bool active) {
    gpio_put(LED_PIN, active);
    led_state = active;
}
static void led_switch() {
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
}

static void i2c_receive(i2c_inst_t *i2c) {
    if (!ctx.mem_address_written) {
        // writes always start with the memory address
        uint8_t addr = i2c_read_byte(i2c);
        ctx.mem_f_bit = ((addr & 0x80) == 0x80);
        ctx.mem_address = addr; // & 0x7F;
        push_msg(MSG_START, ctx.mem_address, 0x0);

        ctx.mem_address_written = true;
    } else {
        // save into memory
        ctx.mem[ctx.mem_address] = i2c_read_byte(i2c);
        push_msg(MSG_WRITE, ctx.mem_address, ctx.mem[ctx.mem_address]);

        ctx.mem_address = auto_increment(ctx.mem_address);
    }
}
static void i2c_request(i2c_inst_t *i2c) {
    // load from memory
    i2c_write_byte(i2c, ctx.mem[ctx.mem_address]);
    push_msg(MSG_READ, ctx.mem_address, ctx.mem[ctx.mem_address]);

    switch (ctx.mem_address) {
    case PULSE_SRC:
        reset_irq();
    case PL_STATUS:
    case STATUS_MMA8452Q: //Custom
        ctx.mem[ctx.mem_address] = 0x0; //Reset after read
        break;
    }

    ctx.mem_address = auto_increment(ctx.mem_address);
}
static void i2c_stop() {
    if (ctx.mem_address_written) {
        push_msg(MSG_STOP, 0x0, 0x0);
        ctx.mem_address_written = false;
        ctx.mem_f_bit = false;
    }
}

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        i2c_receive(i2c);
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        i2c_request(i2c);
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        i2c_stop();
        break;
    default:
        break;
    }
}

void init_registers() {
    //fill with 0x00

    //Stock
    ctx.mem[WHO_AM_I] = 0x1A;
    ctx.mem[PL_CFG] = 0x80;
    ctx.mem[PL_BF_ZCOMP] = 0x44;
    ctx.mem[P_L_THS_REG] = 0x84;
}

static void setup_slave() {
    init_registers();

    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    gpio_init(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, GPIO_OUT);
    reset_irq();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    led(true);

    //gpio_init(EAR_LEFT_PIN);
    //gpio_set_dir(EAR_LEFT_PIN, GPIO_IN);

    //gpio_init(EAR_RIGHT_PIN);
    //gpio_set_dir(EAR_RIGHT_PIN, GPIO_IN);

    gpio_init(TAP_LEFT_PIN);
    gpio_set_dir(TAP_LEFT_PIN, GPIO_IN);
    gpio_pull_up(TAP_LEFT_PIN);
    gpio_init(TAP_RIGHT_PIN);
    gpio_set_dir(TAP_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(TAP_RIGHT_PIN);

    gpio_init(TILT_LEFT_PIN);
    gpio_set_dir(TILT_LEFT_PIN, GPIO_IN);
    gpio_pull_up(TILT_LEFT_PIN);
    gpio_init(TILT_RIGHT_PIN);
    gpio_set_dir(TILT_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(TILT_RIGHT_PIN);

    gpio_init(TAP_N_TILT_FWD_PIN);
    gpio_set_dir(TAP_N_TILT_FWD_PIN, GPIO_IN);
    gpio_pull_up(TAP_N_TILT_FWD_PIN);
    gpio_init(TAP_N_TILT_BWD_PIN);
    gpio_set_dir(TAP_N_TILT_BWD_PIN, GPIO_IN);
    gpio_pull_up(TAP_N_TILT_BWD_PIN);
    
    gpio_init(HEADSDOWN_PIN);
    gpio_set_dir(HEADSDOWN_PIN, GPIO_IN);
    gpio_pull_up(HEADSDOWN_PIN);
    
    gpio_init(SLEEP_PIN);
    gpio_set_dir(SLEEP_PIN, GPIO_IN);
    gpio_pull_down(SLEEP_PIN);


    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

uint32_t last_fifo;
#define FIFO_TIMEOUT 50

void send_fifo(uint8_t* fifo, size_t len, uint8_t status, uint8_t pulse, bool wait, bool irq) {
    ctx.mem[STATUS_MMA8452Q] = status;
    memcpy(&ctx.mem[FIFO_BUF], fifo, len);
    ctx.mem[PULSE_SRC] = pulse;

    if (irq)
        set_irq();

    if (wait) {
        last_fifo = millis();
        while (ctx.mem[STATUS_MMA8452Q] != 0 || ctx.mem[PULSE_SRC] != 0 || ctx.mem_f_bit) {
            sleep_ms(1); //Wait for read
            if (last_fifo + FIFO_TIMEOUT < millis()) {
                push_msg(MSG_TIMEOUT, 0x0, 0x0);
                break;
            }
        }
    }
}

static void send_tap_right() {
    puts("tap right\r\n");
    uint8_t fifo[] = { 0xef, 0x0, 0x0, 0xef, 0x1, 0x1, 0xf0, 0x2, 0x1, 0xef, 0x2, 0x1, 0xef, 0x2, 0x1, 0xf1, 0x1, 0x1, 0xef, 0x2, 0x1, 0xef, 0x2, 0x1, 0xef, 0x2, 0x1, 0xef, 0x2, 0x1, 0xef, 0x1, 0x1, 0xef, 0x1, 0x1, 0xef, 0x1, 0x1, 0xef, 0xc3, 0xd8, 0x19, 0x80, 0x82, 0x21, 0x80, 0xb5, 0xde, 0xca, 0x14, 0xc9, 0x4, 0xfd, 0xe0, 0x1c, 0x1a, 0xd1, 0x1, 0x1c, 0xe3, 0x17, 0xd, 0xf2, 0x16, 0x1, 0xf6, 0x19, 0xf3, 0xf9, 0x1f, 0xf5, 0xef, 0x1f, 0x0, 0xea, 0x23, 0xd, 0xeb, 0x25, 0x12, 0xed, 0x22, 0x11, 0xed, 0x19, 0xf, 0xed, 0xe, 0xa, 0xf0, 0x9, 0x8, 0xf2, 0x9, 0xb };
    send_fifo(fifo, sizeof(fifo), 0xe0, 0xc4, true, true);
}
static void send_tap_left() {
    puts("tap left\r\n");
    uint8_t fifo[] = { 0xef, 0x1, 0xff, 0xf0, 0xfd, 0xfe, 0xef, 0xfc, 0xfe, 0xef, 0xfc, 0xff, 0xf0, 0xfc, 0xff, 0xef, 0xfc, 0xff, 0xef, 0xfc, 0xff, 0xef, 0xfc, 0x0, 0xef, 0xfc, 0x0, 0xf0, 0xfc, 0x0, 0xef, 0xfc, 0x0, 0xef, 0xfc, 0x0, 0xef, 0xfb, 0x0, 0xef, 0xfb, 0x0, 0xf0, 0x3e, 0x40, 0xf1, 0x77, 0x2a, 0xd, 0x6a, 0x68, 0xf5, 0x2a, 0x3b, 0xc, 0xf5, 0xeb, 0xe4, 0xf7, 0xe8, 0xd3, 0x1, 0xa, 0xe9, 0x19, 0xf1, 0xf2, 0x29, 0xee, 0xef, 0x1d, 0xee, 0xf3, 0x14, 0xf6, 0xfb, 0x8, 0xfa, 0xf7, 0xfe, 0xf9, 0xf6, 0xf9, 0xfc, 0xe0, 0xfb, 0x4, 0xe4, 0xfb, 0xa, 0xf2, 0x1, 0xb, 0x1a, 0x1, 0xff };
    send_fifo(fifo, sizeof(fifo), 0xe0, 0xc0, true, true);
}


static void send_headsup() {
    puts("headsup\r\n");
    uint8_t fifo[] = { 0xef, 0x0, 0xff, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xf0, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0xff, 0xef, 0x0, 0x0, 0xef, 0x0, 0x0 };
    send_fifo(fifo, sizeof(fifo), 0x20, 0x00, false, true);
}
static void send_headsdown() { // TODO
    puts("headsdown\r\n");
    uint8_t fifo[] = { 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0, 0xf, 0x2, 0x0 };
    send_fifo(fifo, sizeof(fifo), 0x20, 0x00, false, true);
}

static void send_tilt_right() {
    puts("tilt right\r\n");
    uint8_t fifo[] = { 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8, 0xf4, 0xf8, 0xf8 };
    send_fifo(fifo, sizeof(fifo), 0x20, 0x00, false, true);
}
static void send_tilt_right_wait_release() {
    send_tilt_right();
    sleep_ms(1100); //Worked till 1050, 1000 did not
    send_headsup();
}
static void send_tilt_left() {
    puts("tilt left\r\n");
    uint8_t fifo[] = { 0xf4, 0x9, 0x8, 0xf4, 0x8, 0x9, 0xf4, 0x8, 0x7, 0xf5, 0x9, 0x7, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x8, 0xf5, 0x8, 0x7, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x9, 0xf4, 0x8, 0x7, 0xf4, 0x9, 0x7, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x8, 0xf4, 0x9, 0x7, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x7, 0xf4, 0x9, 0x7, 0xf4, 0x8, 0x9, 0xf4, 0x8, 0x7, 0xf4, 0x9, 0x7, 0xf4, 0x9, 0x8, 0xf4, 0x8, 0x8, 0xf5, 0x8, 0x7, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x7, 0xf5, 0x9, 0x7, 0xf4, 0x9, 0x8, 0xf4, 0x8, 0x8, 0xf4, 0x8, 0x7, 0xf5, 0x8, 0x7 };
    send_fifo(fifo, sizeof(fifo), 0x20, 0x00, false, true);
}
static void send_tilt_left_wait_release() {
    send_tilt_left();
    sleep_ms(1100); //Worked till 1050, 1000 did not
    send_headsup();
}

static void press_ear(uint8_t pin) {
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, false);
    sleep_ms(100);
    gpio_put(pin, true);
    gpio_set_dir(pin, GPIO_IN);
}

typedef void (*ActionFunction)();

void onButton(uint8_t pin, ActionFunction action) {
    if (!gpio_get(pin)) {
        action();
        while (!gpio_get(pin)) { }
        send_headsup();
    }
}
void onButtonShortLong(uint8_t pin, uint8_t not_pin, ActionFunction shortAction, ActionFunction longAction) {
    if (!gpio_get(pin) && gpio_get(not_pin)) {
        sleep_ms(500);
        if (gpio_get(not_pin)) {
            if (!gpio_get(pin)) { // Long
                longAction();
                while (!gpio_get(pin)) {}
                send_headsup();
            } else { // Short
                shortAction();
                while (!gpio_get(pin)) {}
            }
        }
    }
}
void onButtonDoubleHold(uint8_t pinA, uint8_t pinB, ActionFunction action) {
    if (!gpio_get(pinA) && !gpio_get(pinB)) {
        action();
        while(!gpio_get(pinA) && !gpio_get(pinB)) {}
    }
}

uint32_t lastMessageTime;
#define SLEEP_TIMEOUT 1000 * 10
void loop() {
    if (get_bootsel_button()) {
        reset_irq();
        sleep_ms(500);
        if (get_bootsel_button()) { //Long
            send_tap_right();
        } else { //Short
            send_tap_left();
        }

        while (get_bootsel_button()) {}
    }

    onButtonShortLong(TAP_N_TILT_FWD_PIN, TAP_N_TILT_BWD_PIN, send_tap_right, send_tilt_left);
    onButtonShortLong(TAP_N_TILT_BWD_PIN, TAP_N_TILT_FWD_PIN, send_tap_left, send_tilt_right);
    onButtonDoubleHold(TAP_N_TILT_FWD_PIN, TAP_N_TILT_FWD_PIN, send_headsdown);

    onButton(TAP_LEFT_PIN, send_tap_left);
    onButton(TAP_RIGHT_PIN, send_tap_right);
    onButton(TILT_LEFT_PIN, send_tilt_left);
    onButton(TILT_RIGHT_PIN, send_tilt_right);
    onButton(HEADSDOWN_PIN, send_headsdown);

    while (ctx.msg_read != ctx.msg_write) {
        ctx_msg msg = pull_msg();

        if (msg.type == MSG_WRITE) {
            printf("write register: %02x=%02x\r\n", msg.val1, msg.val2);
        } else if (msg.type == MSG_READ) {
            printf("read register: %02x=%02x\r\n", msg.val1, msg.val2);
        } else if (msg.type == MSG_STOP) {
            //printf("\r\nSTOP"); //Too often
        } else if (msg.type == MSG_TIMEOUT) {
            //printf("\r\nFifo timeout"); //always?!
        } else if (msg.type == MSG_NONE) {
            printf("Empty msg\r\n");
        }
        led_switch();

        lastMessageTime = millis();
    }
    if (gpio_get(SLEEP_PIN)) {
        lastMessageTime = millis();
    }
    
    if (lastMessageTime + SLEEP_TIMEOUT < millis()) {
        printf("Nothing happened for %is\r\n", SLEEP_TIMEOUT/1000);
        for(uint8_t i=0; i<10; i++) {
            led_switch();
            sleep_ms(50);
        }
        lastMessageTime = millis();
    }

    /*
    if (lastMessageTime + SLEEP_TIMEOUT < millis()) {
        uart_default_tx_wait_blocking();
        uint _scb_orig;
        uint _en0_orig;
        uint _en1_orig;

        _scb_orig = scb_hw->scr;
        _en0_orig = clocks_hw->sleep_en0;
        _en1_orig = clocks_hw->sleep_en1;

        sleep_run_from_xosc(); //needed?
        sleep_goto_dormant_until_pin(SLEEP_PIN, true, true);

        clocks_init();
        // Re-enable Ring Oscillator control
        rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

        // restore clock registers
        scb_hw->scr             = _scb_orig;
        clocks_hw->sleep_en0    = _en0_orig;
        clocks_hw->sleep_en1    = _en1_orig;

        main();
    }
    */
}

int main() {
    setup_slave();
    stdio_init_all();

    printf("\r\nMMA8452Q Emulator\r\n");

    lastMessageTime = millis();
    while (true) {
        loop();
    }
}