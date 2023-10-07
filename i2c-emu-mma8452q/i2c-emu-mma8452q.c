#include <i2c_fifo.h>
#include <i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

static const uint I2C_SLAVE_ADDRESS = 0x1D;

static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5
static const uint IRQ_PIN = 6; // 6
static const uint LED_PIN = 25; // 25

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
    MSG_NONE = 0,
    MSG_READ = 1,
    MSG_WRITE = 2,
} ctx_msg_type;

typedef struct
{
    ctx_msg_type type;
    uint8_t reg;
    uint8_t val;
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

static void push_msg(ctx_msg_type type, uint8_t reg, uint8_t val) {
    if ((ctx.msg_write + 1) % MSG_COUNT != ctx.msg_read) {
        ctx.msg_buffer[ctx.msg_write].type = type;
        ctx.msg_buffer[ctx.msg_write].reg = reg;
        ctx.msg_buffer[ctx.msg_write].val = val;
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
        ctx.mem_address_written = false;
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

    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

bool lastTap = false;
bool lastOrient = false;
void loop() {
    if (get_bootsel_button()) {
        reset_irq();

        sleep_ms(500);
        if (get_bootsel_button()) { //Long
            puts("Press long");
            //TODO
            if (lastOrient) {
                ctx.mem[PL_STATUS] = 0b10000111; //Ears Up
                ctx.mem[PL_STATUS] = 0b10000110; //Ears Up
            } else {
                ctx.mem[PL_STATUS] = 0b10000101; //Ears Down
                ctx.mem[PL_STATUS] = 0b10000100; //Ears Down
            }
            //ctx.mem[STATUS_MMA8452Q] = 0b01000000;
            lastOrient = !lastOrient;

        } else { //Short
            //TODO
            puts("Press short");
            uint8_t fifo1[] = { 0xee, 0x0, 0x0, 0xef, 0xff, 0xff, 0xef, 0x0, 0xff, 0xef, 0x0, 0xfe, 0xef, 0x0, 0xff, 0xef, 0x0, 0xff, 0xef, 0x1, 0xff, 0xef, 0x1, 0xff, 0xf0, 0x1, 0xff, 0xf0, 0x1, 0x0, 0xf0, 0x1, 0xff, 0xef, 0x2, 0xff, 0xeb, 0x3, 0x0, 0xf2, 0x3, 0x0, 0xf4, 0x5e, 0x59, 0xfa, 0x7f, 0x12, 0x1, 0x5a, 0x10, 0xdd, 0x33, 0xf6, 0xd8, 0xfd, 0x12, 0xf0, 0xee, 0xf6, 0xeb, 0xe9, 0xfc, 0xec, 0xff, 0xeb, 0xf7, 0xef, 0xfc, 0xff, 0xe7, 0x0, 0xf8, 0xf8, 0xfc, 0xe9, 0xf0, 0x8, 0xe9, 0xeb, 0x7, 0xec, 0xee, 0x0, 0xeb, 0xed, 0xfe, 0xea, 0xf0, 0xfd, 0xee, 0xf2, 0xfd, 0xf0, 0xf4, 0xfd };
            uint8_t fifo2[] = { 0xf0, 0x0, 0x0, 0xf0, 0x1, 0xff, 0xef, 0x1, 0xff, 0xf0, 0x0, 0xff, 0xef, 0x0, 0xff, 0xef, 0x0, 0xff, 0xef, 0x0, 0xff, 0xf0, 0x0, 0xff, 0xf0, 0x0, 0xff, 0xf0, 0x0, 0xff, 0xef, 0x0, 0xff, 0xf1, 0xda, 0xda, 0xf4, 0xdb, 0xbe, 0xe4, 0xd6, 0x9b, 0x6, 0xd4, 0x92, 0xf9, 0x9, 0x5, 0xec, 0xe8, 0x5, 0xe2, 0x5, 0xfd, 0xe3, 0x46, 0x33, 0xe8, 0xf, 0x2f, 0xf2, 0xf5, 0x1d, 0xf3, 0xfd, 0xf, 0xf9, 0xfb, 0x19, 0xf7, 0xf3, 0x14, 0xf7, 0xf7, 0xd, 0xf5, 0xf6, 0x11, 0xef, 0xf8, 0xd, 0xe9, 0xfc, 0xb, 0xe7, 0x1, 0x8, 0xea, 0x2, 0x5, 0xeb, 0x5, 0x3, 0xec, 0x7, 0xff };

            if (lastTap) { // Left
                ctx.mem[STATUS_MMA8452Q] = 0x05;
                memcpy(&ctx.mem[FIFO_BUF], fifo2, sizeof(fifo2));
                ctx.mem[PULSE_SRC] = 0x00;
            
                set_irq();
                while (ctx.mem[STATUS_MMA8452Q] != 0 || ctx.mem[PULSE_SRC] != 0)
                {
                    sleep_ms(1); //Wait for read
                }
                
                ctx.mem[STATUS_MMA8452Q] = 0xe0;
                memcpy(&ctx.mem[FIFO_BUF], fifo1, sizeof(fifo1));
                ctx.mem[PULSE_SRC] = 0xc0;
            
                set_irq();
                while (ctx.mem[STATUS_MMA8452Q] != 0 || ctx.mem[PULSE_SRC] != 0)
                {
                    sleep_ms(1); //Wait for read
                }
            } else { // Right
                ctx.mem[STATUS_MMA8452Q] = 0xe0;
                memcpy(&ctx.mem[FIFO_BUF], fifo1, sizeof(fifo1));
                ctx.mem[PULSE_SRC] = 0xc0;
            
                set_irq();
                while (ctx.mem[STATUS_MMA8452Q] != 0 || ctx.mem[PULSE_SRC] != 0)
                {
                    sleep_ms(1); //Wait for read
                }

                ctx.mem[STATUS_MMA8452Q] = 0xe0;
                memcpy(&ctx.mem[FIFO_BUF], fifo2, sizeof(fifo2));
                ctx.mem[PULSE_SRC] = 0xc4;
            
                set_irq();
                while (ctx.mem[STATUS_MMA8452Q] != 0 || ctx.mem[PULSE_SRC] != 0)
                {
                    sleep_ms(1); //Wait for read
                }
            }
            lastTap = !lastTap;
        }
        //ctx.mem[STATUS_MMA8452Q] = 0x20;


        while (get_bootsel_button()) {}
    }

    while (ctx.msg_read != ctx.msg_write) {
        ctx_msg msg = pull_msg();

        if (msg.type == MSG_WRITE) {
            printf("write register %02x=%02x\r\n", msg.reg, msg.val);
            led_switch();
        } else if (msg.type == MSG_READ) {
            printf("read register %02x=%02x\r\n", msg.reg, msg.val);
            led_switch();
        } else if (msg.type == MSG_NONE) {
            printf("Empty msg %02x=%02x\r\n", msg.reg, msg.val);
            led_switch();
        }
    }
}

int main() {
    stdio_init_all();

    setup_slave();
    puts("\nMMA8452Q Emulator\n");

    while (true) {
        loop();
    }
}