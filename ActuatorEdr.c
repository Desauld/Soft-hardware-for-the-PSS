/*
 * ActuatorEdrHub_ATmega_test_v0_1.c
 * Target: ATmega328P, Microchip Studio / avr-gcc
 *
 * One AVR emulates two virtual chips for the Proteus ACU scaffold:
 *   - Squib driver
 *   - BlackBox / EDR register block
 *
 * Wiring model:
 *   - PB3/PB4/PB5 are a shared software SPI bus
 *   - PB2 is the real transaction gate derived from:
 *       PB2 = CS_SQUIB AND CS_EDR
 *     because both virtual CS lines are active low
 *   - PD0 receives CS_SQUIB
 *   - PD1 receives CS_EDR
 *   - exactly one virtual CS must be low during a transaction
 *
 * The bus is intentionally software-driven to avoid Proteus SPDR collisions.
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define SPI_SS_BIT              PB2
#define SPI_MOSI_BIT            PB3
#define SPI_MISO_BIT            PB4
#define SPI_SCK_BIT             PB5

#define VCS_SQUIB_BIT           PD0
#define VCS_EDR_BIT             PD1

#define OUT_FRONT_BIT           PD2
#define OUT_BELT_BIT            PD3
#define OUT_SIDE_L_BIT          PD4
#define OUT_SIDE_R_BIT          PD5
#define OUT_ROLLOVER_BIT        PD6
#define OUT_FAULT_BIT           PD7

#define CMD_ALL_OFF             0x00
#define CMD_FRONT_ON            0x01
#define CMD_BELT_ON             0x02
#define CMD_SIDE_L_ON           0x04
#define CMD_SIDE_R_ON           0x08
#define CMD_ROLLOVER_ON         0x10
#define CMD_ALL_ON              0x1F
#define CMD_PATTERN_A           0xA5
#define CMD_PATTERN_B           0x5A

#define DEV_NONE                0x00
#define DEV_SQUIB               0x01
#define DEV_EDR                 0x02
#define DEV_MULTIPLE            0xFE

#define EDR_WHO_AM_I            0xE4

#define EDR_REG_WHO_AM_I        0x0F
#define EDR_REG_STATUS          0x10
#define EDR_REG_STAGE           0x11
#define EDR_REG_SCENARIO_INPUTS 0x12
#define EDR_REG_SENSOR_FLAGS    0x13
#define EDR_REG_LAST_SQUIB_CMD  0x14
#define EDR_REG_SQUIB_OUTPUT    0x15
#define EDR_REG_HOST_EVENT_ID   0x16
#define EDR_REG_HOST_FLAGS      0x17
#define EDR_REG_HOST_DATA0      0x18
#define EDR_REG_HOST_DATA1      0x19
#define EDR_REG_TRANSACTION_H   0x1A
#define EDR_REG_TRANSACTION_L   0x1B
#define EDR_REG_CTRL            0x20
#define EDR_REG_FAULT           0x21

#define EDR_STATUS_INIT_DONE      (1U << 0)
#define EDR_STATUS_READ_SEEN      (1U << 1)
#define EDR_STATUS_WRITE_SEEN     (1U << 2)
#define EDR_STATUS_SQUIB_ACTIVITY (1U << 3)
#define EDR_STATUS_LOG_VALID      (1U << 4)
#define EDR_STATUS_FAULT          (1U << 5)

#define EDR_CTRL_CLEAR_STATUS     (1U << 0)
#define EDR_CTRL_CLEAR_LOG        (1U << 1)

#define EDR_FAULT_NONE            0x00
#define EDR_FAULT_BAD_SELECT      0xE1
#define EDR_FAULT_BAD_SQUIB_CMD   0xE2

static volatile uint8_t g_active_device = DEV_NONE;
static volatile uint8_t g_current_tx_byte = 0x00;
static volatile uint8_t g_next_tx_byte = 0x00;
static volatile uint8_t g_rx_shift = 0x00;
static volatile uint8_t g_bits_sampled = 0;
static volatile uint8_t g_last_sck = 0;
static volatile uint8_t g_transaction_byte_index = 0;
static volatile uint8_t g_edr_addr = 0x00;
static volatile uint8_t g_edr_is_read = 0;
static volatile uint8_t g_select_state = DEV_NONE;
static volatile uint8_t g_first_byte_pending = 0;
static volatile uint8_t g_first_byte_value = 0x00;
static volatile uint8_t g_squib_output_mask = 0x00;
static volatile uint16_t g_transaction_count = 0;
static volatile uint8_t g_edr_regs[0x80];

static uint8_t bus_is_selected(void)
{
    return (PINB & (1 << SPI_SS_BIT)) == 0;
}

static void spi_miso_release(void)
{
    DDRB &= ~(1 << SPI_MISO_BIT);
    PORTB &= ~(1 << SPI_MISO_BIT);
}

static void spi_miso_drive(uint8_t level)
{
    DDRB |= (1 << SPI_MISO_BIT);

    if (level) {
        PORTB |= (1 << SPI_MISO_BIT);
    } else {
        PORTB &= ~(1 << SPI_MISO_BIT);
    }
}

static uint8_t get_selected_virtual_device(void)
{
    uint8_t pind = PIND;
    uint8_t count = 0;
    uint8_t dev = DEV_NONE;

    if ((pind & (1 << VCS_SQUIB_BIT)) == 0) {
        count++;
        dev = DEV_SQUIB;
    }

    if ((pind & (1 << VCS_EDR_BIT)) == 0) {
        count++;
        dev = DEV_EDR;
    }

    if (count == 0) {
        return DEV_NONE;
    }
    if (count > 1) {
        return DEV_MULTIPLE;
    }

    return dev;
}

static void apply_outputs(uint8_t mask)
{
    if (mask & 0x01) PORTD |=  (1 << OUT_FRONT_BIT);    else PORTD &= ~(1 << OUT_FRONT_BIT);
    if (mask & 0x02) PORTD |=  (1 << OUT_BELT_BIT);     else PORTD &= ~(1 << OUT_BELT_BIT);
    if (mask & 0x04) PORTD |=  (1 << OUT_SIDE_L_BIT);   else PORTD &= ~(1 << OUT_SIDE_L_BIT);
    if (mask & 0x08) PORTD |=  (1 << OUT_SIDE_R_BIT);   else PORTD &= ~(1 << OUT_SIDE_R_BIT);
    if (mask & 0x10) PORTD |=  (1 << OUT_ROLLOVER_BIT); else PORTD &= ~(1 << OUT_ROLLOVER_BIT);
}

static void edr_update_transaction_counter(void)
{
    g_edr_regs[EDR_REG_TRANSACTION_H] = (uint8_t)(g_transaction_count >> 8);
    g_edr_regs[EDR_REG_TRANSACTION_L] = (uint8_t)(g_transaction_count & 0xFFU);
}

static void edr_set_fault(uint8_t fault_code)
{
    g_edr_regs[EDR_REG_FAULT] = fault_code;
    g_edr_regs[EDR_REG_STATUS] |= EDR_STATUS_FAULT;
}

static void edr_clear_fault(void)
{
    g_edr_regs[EDR_REG_FAULT] = EDR_FAULT_NONE;
    g_edr_regs[EDR_REG_STATUS] &= (uint8_t)~EDR_STATUS_FAULT;
}

static void edr_clear_log(void)
{
    g_edr_regs[EDR_REG_STAGE] = 0x00;
    g_edr_regs[EDR_REG_SCENARIO_INPUTS] = 0x00;
    g_edr_regs[EDR_REG_SENSOR_FLAGS] = 0x00;
    g_edr_regs[EDR_REG_LAST_SQUIB_CMD] = 0x00;
    g_edr_regs[EDR_REG_SQUIB_OUTPUT] = 0x00;
    g_edr_regs[EDR_REG_HOST_EVENT_ID] = 0x00;
    g_edr_regs[EDR_REG_HOST_FLAGS] = 0x00;
    g_edr_regs[EDR_REG_HOST_DATA0] = 0x00;
    g_edr_regs[EDR_REG_HOST_DATA1] = 0x00;
}

static void edr_apply_ctrl(uint8_t value)
{
    if (value & EDR_CTRL_CLEAR_STATUS) {
        g_edr_regs[EDR_REG_STATUS] = EDR_STATUS_INIT_DONE;
        edr_clear_fault();
    }

    if (value & EDR_CTRL_CLEAR_LOG) {
        edr_clear_log();
    }

    g_edr_regs[EDR_REG_CTRL] = 0x00;
}

static void edr_init_defaults(void)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)sizeof(g_edr_regs); i++) {
        g_edr_regs[i] = 0x00;
    }

    g_edr_regs[EDR_REG_WHO_AM_I] = EDR_WHO_AM_I;
    g_edr_regs[EDR_REG_STATUS] = EDR_STATUS_INIT_DONE;
    g_edr_regs[EDR_REG_CTRL] = 0x00;
    g_edr_regs[EDR_REG_FAULT] = EDR_FAULT_NONE;
    g_edr_regs[EDR_REG_SQUIB_OUTPUT] = 0x00;
    edr_update_transaction_counter();
}

static uint8_t edr_read_reg(uint8_t addr)
{
    g_edr_regs[EDR_REG_STATUS] |= EDR_STATUS_READ_SEEN;
    return g_edr_regs[addr & 0x7FU];
}

static void edr_write_reg(uint8_t addr, uint8_t value)
{
    addr &= 0x7FU;

    switch (addr) {
    case EDR_REG_WHO_AM_I:
    case EDR_REG_STATUS:
    case EDR_REG_TRANSACTION_H:
    case EDR_REG_TRANSACTION_L:
        /* read only */
        break;

    case EDR_REG_CTRL:
        g_edr_regs[EDR_REG_CTRL] = value;
        edr_apply_ctrl(value);
        break;

    default:
        g_edr_regs[addr] = value;
        g_edr_regs[EDR_REG_STATUS] |= (EDR_STATUS_WRITE_SEEN | EDR_STATUS_LOG_VALID);
        break;
    }
}

static void update_squib_regs(uint8_t cmd, uint8_t output_mask)
{
    g_edr_regs[EDR_REG_LAST_SQUIB_CMD] = cmd;
    g_edr_regs[EDR_REG_SQUIB_OUTPUT] = output_mask;
    g_edr_regs[EDR_REG_STATUS] |= (EDR_STATUS_SQUIB_ACTIVITY | EDR_STATUS_LOG_VALID);
}

static void handle_squib_command(uint8_t cmd)
{
    switch (cmd) {
    case CMD_ALL_OFF:
        g_squib_output_mask = 0x00;
        apply_outputs(g_squib_output_mask);
        PORTD &= ~(1 << OUT_FAULT_BIT);
        edr_clear_fault();
        break;

    case CMD_PATTERN_A:
        g_squib_output_mask = (uint8_t)(0x01U | 0x04U | 0x10U);
        apply_outputs(g_squib_output_mask);
        PORTD &= ~(1 << OUT_FAULT_BIT);
        edr_clear_fault();
        break;

    case CMD_PATTERN_B:
        g_squib_output_mask = (uint8_t)(0x02U | 0x08U);
        apply_outputs(g_squib_output_mask);
        PORTD &= ~(1 << OUT_FAULT_BIT);
        edr_clear_fault();
        break;

    default:
        if ((cmd & 0xE0U) == 0x00U) {
            g_squib_output_mask = (uint8_t)(cmd & 0x1FU);
            apply_outputs(g_squib_output_mask);
            PORTD &= ~(1 << OUT_FAULT_BIT);
            edr_clear_fault();
        } else {
            PORTD |= (1 << OUT_FAULT_BIT);
            edr_set_fault(EDR_FAULT_BAD_SQUIB_CMD);
        }
        break;
    }

    update_squib_regs(cmd, g_squib_output_mask);
}

static void prepare_next_tx_bit(void)
{
    spi_miso_drive((g_current_tx_byte & 0x80U) != 0U);
}

static void begin_virtual_transaction(void)
{
    g_select_state = get_selected_virtual_device();
    g_active_device = (g_select_state == DEV_SQUIB || g_select_state == DEV_EDR) ? g_select_state : DEV_NONE;
    g_current_tx_byte = 0x00;
    g_next_tx_byte = 0x00;
    g_rx_shift = 0x00;
    g_bits_sampled = 0;
    g_transaction_byte_index = 0;
    g_edr_addr = 0x00;
    g_edr_is_read = 0;
    g_first_byte_pending = 0;
    g_first_byte_value = 0x00;
    g_last_sck = (PINB & (1 << SPI_SCK_BIT)) ? 1U : 0U;

    g_transaction_count++;
    edr_update_transaction_counter();

    prepare_next_tx_bit();
}

static void end_virtual_transaction(void)
{
    spi_miso_release();
    g_active_device = DEV_NONE;
    g_current_tx_byte = 0x00;
    g_next_tx_byte = 0x00;
    g_rx_shift = 0x00;
    g_bits_sampled = 0;
    g_transaction_byte_index = 0;
    g_edr_addr = 0x00;
    g_edr_is_read = 0;
    if (g_first_byte_pending) {
        handle_squib_command(g_first_byte_value);
    }
    g_select_state = DEV_NONE;
    g_first_byte_pending = 0;
    g_first_byte_value = 0x00;
}

static void handle_edr_byte(uint8_t byte)
{
    if (g_transaction_byte_index == 0U) {
        g_edr_is_read = (byte & 0x80U) ? 1U : 0U;
        g_edr_addr = (uint8_t)(byte & 0x7FU);

        if (g_edr_is_read) {
            g_next_tx_byte = edr_read_reg(g_edr_addr);
        } else {
            g_next_tx_byte = 0x00;
        }
        return;
    }

    if (g_edr_is_read) {
        g_edr_addr++;
        g_next_tx_byte = edr_read_reg(g_edr_addr);
    } else {
        edr_write_reg(g_edr_addr, byte);
        g_edr_addr++;
        g_next_tx_byte = 0x00;
    }
}

static void handle_completed_byte(uint8_t byte)
{
    if (g_transaction_byte_index == 0U) {
        if (g_active_device == DEV_EDR) {
            edr_clear_fault();
            handle_edr_byte(byte);
        } else if (g_active_device == DEV_SQUIB) {
            g_first_byte_pending = 1U;
            g_first_byte_value = byte;
            g_next_tx_byte = 0x00;
        } else if (byte & 0x80U) {
            g_active_device = DEV_EDR;
            edr_clear_fault();
            handle_edr_byte(byte);
        } else {
            g_first_byte_pending = 1U;
            g_first_byte_value = byte;
            g_next_tx_byte = 0x00;
            if (g_select_state == DEV_MULTIPLE) {
                edr_set_fault(EDR_FAULT_BAD_SELECT);
            }
        }
    } else {
        if (g_first_byte_pending) {
            g_active_device = DEV_EDR;
            g_transaction_byte_index = 0U;
            edr_clear_fault();
            handle_edr_byte(g_first_byte_value);
            g_transaction_byte_index = 1U;
            g_first_byte_pending = 0U;
        }

        switch (g_active_device) {
        case DEV_SQUIB:
            g_next_tx_byte = 0x00;
            break;

        case DEV_EDR:
            handle_edr_byte(byte);
            break;

        case DEV_NONE:
        case DEV_MULTIPLE:
        default:
            g_next_tx_byte = 0xEE;
            break;
        }
    }

    g_transaction_byte_index++;
}

static void gpio_init(void)
{
    DDRB &= ~((1 << SPI_SS_BIT) | (1 << SPI_MOSI_BIT) | (1 << SPI_MISO_BIT) | (1 << SPI_SCK_BIT));
    PORTB |= (1 << SPI_SS_BIT);
    PORTB &= ~((1 << SPI_MOSI_BIT) | (1 << SPI_MISO_BIT) | (1 << SPI_SCK_BIT));

    DDRD &= ~((1 << VCS_SQUIB_BIT) | (1 << VCS_EDR_BIT));
    PORTD |= ((1 << VCS_SQUIB_BIT) | (1 << VCS_EDR_BIT));

    DDRD |= ((1 << OUT_FRONT_BIT) |
             (1 << OUT_BELT_BIT) |
             (1 << OUT_SIDE_L_BIT) |
             (1 << OUT_SIDE_R_BIT) |
             (1 << OUT_ROLLOVER_BIT) |
             (1 << OUT_FAULT_BIT));

    PORTD &= ~((1 << OUT_FRONT_BIT) |
               (1 << OUT_BELT_BIT) |
               (1 << OUT_SIDE_L_BIT) |
               (1 << OUT_SIDE_R_BIT) |
               (1 << OUT_ROLLOVER_BIT) |
               (1 << OUT_FAULT_BIT));

    spi_miso_release();
}

static void pcint_init(void)
{
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << SPI_SS_BIT) | (1 << SPI_SCK_BIT);
}

ISR(PCINT0_vect)
{
    uint8_t current_sck = (PINB & (1 << SPI_SCK_BIT)) ? 1U : 0U;

    if (!bus_is_selected()) {
        end_virtual_transaction();
        g_last_sck = current_sck;
        return;
    }

    if (g_active_device == DEV_NONE) {
        begin_virtual_transaction();
        return;
    }

    if ((g_last_sck == 0U) && (current_sck != 0U)) {
        g_rx_shift <<= 1;
        if (PINB & (1 << SPI_MOSI_BIT)) {
            g_rx_shift |= 0x01U;
        }

        g_bits_sampled++;
        if (g_bits_sampled >= 8U) {
            handle_completed_byte(g_rx_shift);
        }
    } else if ((g_last_sck != 0U) && (current_sck == 0U)) {
        if (g_bits_sampled == 8U) {
            g_current_tx_byte = g_next_tx_byte;
            g_rx_shift = 0x00;
            g_bits_sampled = 0U;
            prepare_next_tx_bit();
        } else if ((g_bits_sampled > 0U) && (g_bits_sampled < 8U)) {
            g_current_tx_byte <<= 1;
            prepare_next_tx_bit();
        }
    }

    g_last_sck = current_sck;
}

int main(void)
{
    gpio_init();
    edr_init_defaults();
    pcint_init();
    sei();

    while (1) {
        /* Keep transport stable; future ACU logic can extend register behavior. */
    }
}
