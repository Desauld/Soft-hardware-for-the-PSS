/*
 * SensorHub_ATmega_test_v0_6.c
 * Target: ATmega328P, Microchip Studio / avr-gcc
 *
 * One AVR emulates five virtual sensors for the Proteus ACU bench:
 *   - IMU           (ASM330-like subset)
 *   - HighG_L       (H3LIS-like subset)
 *   - HighG_R       (H3LIS-like subset)
 *   - KP200_L       (virtual pressure sensor)
 *   - KP200_R       (virtual pressure sensor)
 *
 * External wiring model:
 *   - PB2 is the real bus-select gate and must be driven by:
 *       PB2 = CS_IMU AND CS_HIGHG_L AND CS_HIGHG_R AND CS_PPS_L AND CS_PPS_R
 *   - PD2..PD6 are the virtual chip-select inputs
 *   - PB3/PB4/PB5 are MOSI/MISO/SCK on the shared software SPI link
 *
 * Scenario control inputs:
 *   - PC0..PC2 : scenario code
 *   - PC3      : start scenario
 *   - PC4      : reset / idle override
 *   - PC5      : fault injection
 *
 * SPI protocol for every virtual chip:
 *   - first byte: bit7 = 1 read, 0 write; bit6..0 = register address
 *   - read: second and next bytes clock out register values with auto-increment
 *   - write: second and next bytes are written with auto-increment
 *
 * The implementation is fully software-driven to avoid Proteus SPDR collisions.
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define SPI_SS_BIT               PB2
#define SPI_MOSI_BIT             PB3
#define SPI_MISO_BIT             PB4
#define SPI_SCK_BIT              PB5

#define CS_IMU_BIT               PD2
#define CS_HIGHG_L_BIT           PD3
#define CS_HIGHG_R_BIT           PD4
#define CS_KP200_L_BIT           PD5
#define CS_KP200_R_BIT           PD6

#define SCENARIO_SEL0_BIT        PC0
#define SCENARIO_SEL1_BIT        PC1
#define SCENARIO_SEL2_BIT        PC2
#define SCENARIO_START_BIT       PC3
#define SCENARIO_RESET_BIT       PC4
#define SCENARIO_FAULT_BIT       PC5

#define DEV_NONE                 0x00
#define DEV_IMU                  0x01
#define DEV_HIGHG_L              0x02
#define DEV_HIGHG_R              0x03
#define DEV_KP200_L              0x04
#define DEV_KP200_R              0x05
#define DEV_MULTIPLE             0xFE

#define SCENARIO_IDLE            0x00
#define SCENARIO_FRONT           0x01
#define SCENARIO_SIDE_LEFT       0x02
#define SCENARIO_SIDE_RIGHT      0x03
#define SCENARIO_REAR            0x04
#define SCENARIO_ROLLOVER        0x05
#define SCENARIO_MISUSE          0x06
#define SCENARIO_VISUAL          0x07

#define EMU_STATUS_DRDY          0x01
#define EMU_STATUS_FAULT         0x80

#define IMU_WHO_AM_I_REG         0x0F
#define IMU_WHO_AM_I_VAL         0x6B
#define IMU_CTRL1_XL_REG         0x10
#define IMU_CTRL2_G_REG          0x11
#define IMU_STATUS_REG           0x1E
#define IMU_OUTX_L_G_REG         0x22
#define IMU_OUTX_L_A_REG         0x28

#define HIGHG_WHO_AM_I_REG       0x0F
#define HIGHG_WHO_AM_I_VAL       0x32
#define HIGHG_CTRL1_REG          0x20
#define HIGHG_STATUS_REG         0x27
#define HIGHG_OUT_X_L_REG        0x28

#define KP200_L_WHO_AM_I_VAL     0xA1
#define KP200_R_WHO_AM_I_VAL     0xA2
#define KP200_WHO_AM_I_REG       0x0F
#define KP200_STATUS_REG         0x10
#define KP200_PRESSURE_H_REG     0x11
#define KP200_PRESSURE_L_REG     0x12
#define KP200_CTRL_REG           0x20
#define KP200_FAULT_REG          0x21

#define KP200_FAULT_NONE         0x00
#define KP200_FAULT_INJECTED     0xF1

#define EMU_READ_BIT             0x80

#define IMU_RAW_PER_G            2048
#define HIGHG_RAW_PER_G          100

static volatile uint8_t g_active_device = DEV_NONE;
static volatile uint8_t g_transaction_active = 0;
static volatile uint8_t g_transaction_byte_index = 0;
static volatile uint8_t g_reg_addr = 0x00;
static volatile uint8_t g_is_read = 0;
static volatile uint8_t g_profile_active = 0;
static volatile uint8_t g_profile_scenario = SCENARIO_IDLE;
static volatile uint8_t g_prev_start = 0;
static volatile uint16_t g_profile_tick = 0;

static volatile uint8_t g_imu_regs[0x80];
static volatile uint8_t g_highg_l_regs[0x80];
static volatile uint8_t g_highg_r_regs[0x80];
static volatile uint8_t g_kp200_l_regs[0x80];
static volatile uint8_t g_kp200_r_regs[0x80];

#define PROFILE_TICK_HZ             2000U
#define PROFILE_PREDELAY_TICKS      200U
#define PROFILE_MAX_TICKS           4000U

static uint8_t sensorhub_is_selected(void)
{
    return (PINB & (1 << SPI_SS_BIT)) == 0;
}

static void spi_miso_release(void)
{
    PORTB &= ~(1 << SPI_MISO_BIT);
    DDRB &= ~(1 << SPI_MISO_BIT);
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

static uint8_t get_selected_device(void)
{
    uint8_t pind = PIND;
    uint8_t count = 0;
    uint8_t dev = DEV_NONE;

    if ((pind & (1 << CS_IMU_BIT)) == 0) {
        count++;
        dev = DEV_IMU;
    }
    if ((pind & (1 << CS_HIGHG_L_BIT)) == 0) {
        count++;
        dev = DEV_HIGHG_L;
    }
    if ((pind & (1 << CS_HIGHG_R_BIT)) == 0) {
        count++;
        dev = DEV_HIGHG_R;
    }
    if ((pind & (1 << CS_KP200_L_BIT)) == 0) {
        count++;
        dev = DEV_KP200_L;
    }
    if ((pind & (1 << CS_KP200_R_BIT)) == 0) {
        count++;
        dev = DEV_KP200_R;
    }

    if (count == 0) {
        return DEV_NONE;
    }
    if (count > 1) {
        return DEV_MULTIPLE;
    }

    return dev;
}

static volatile uint8_t* regs_for_device(uint8_t dev)
{
    switch (dev) {
    case DEV_IMU:
        return g_imu_regs;
    case DEV_HIGHG_L:
        return g_highg_l_regs;
    case DEV_HIGHG_R:
        return g_highg_r_regs;
    case DEV_KP200_L:
        return g_kp200_l_regs;
    case DEV_KP200_R:
        return g_kp200_r_regs;
    default:
        return 0;
    }
}

static void set_reg16(volatile uint8_t *regs, uint8_t low_addr, int16_t value)
{
    regs[low_addr] = (uint8_t)(value & 0xFF);
    regs[(uint8_t)(low_addr + 1U)] = (uint8_t)((uint16_t)value >> 8);
}

static int16_t imu_raw_from_g(int16_t g_times_10)
{
    if (g_times_10 > 159) {
        g_times_10 = 159;
    } else if (g_times_10 < -159) {
        g_times_10 = -159;
    }

    return (int16_t)((g_times_10 * IMU_RAW_PER_G) / 10);
}

static int16_t imu_raw_from_dps(int16_t dps)
{
    return (int16_t)((dps * 1000) / 140);
}

static int16_t highg_raw_from_g(int16_t g_times_10)
{
    return (int16_t)((g_times_10 * HIGHG_RAW_PER_G) / 10);
}

static uint8_t scenario_code(void)
{
    return (uint8_t)(PINC & 0x07U);
}

static uint8_t scenario_started(void)
{
    return (PINC & (1 << SCENARIO_START_BIT)) != 0;
}

static uint8_t scenario_reset_active(void)
{
    return (PINC & (1 << SCENARIO_RESET_BIT)) != 0;
}

static uint8_t scenario_fault_active(void)
{
    return (PINC & (1 << SCENARIO_FAULT_BIT)) != 0;
}

static int16_t shaped_pulse(uint16_t tick,
                            uint16_t start_tick,
                            uint16_t rise_ticks,
                            uint16_t hold_ticks,
                            uint16_t fall_ticks,
                            int16_t peak)
{
    uint16_t local_tick;
    uint32_t magnitude;
    uint32_t scaled_peak;

    if (tick < start_tick) {
        return 0;
    }

    local_tick = (uint16_t)(tick - start_tick);
    scaled_peak = (peak < 0) ? (uint32_t)(-peak) : (uint32_t)peak;

    if (local_tick < rise_ticks) {
        magnitude = (rise_ticks == 0U) ? scaled_peak :
                    (scaled_peak * local_tick) / rise_ticks;
    } else if (local_tick < (uint16_t)(rise_ticks + hold_ticks)) {
        magnitude = scaled_peak;
    } else if (local_tick < (uint16_t)(rise_ticks + hold_ticks + fall_ticks)) {
        uint16_t down_tick = (uint16_t)(local_tick - rise_ticks - hold_ticks);
        magnitude = (fall_ticks == 0U) ? 0U :
                    (scaled_peak * (fall_ticks - down_tick)) / fall_ticks;
    } else {
        magnitude = 0U;
    }

    return (peak < 0) ? -(int16_t)magnitude : (int16_t)magnitude;
}

static void init_register_defaults(void)
{
    uint8_t i;

    for (i = 0; i < 0x80U; i++) {
        g_imu_regs[i] = 0x00;
        g_highg_l_regs[i] = 0x00;
        g_highg_r_regs[i] = 0x00;
        g_kp200_l_regs[i] = 0x00;
        g_kp200_r_regs[i] = 0x00;
    }

    g_imu_regs[IMU_WHO_AM_I_REG] = IMU_WHO_AM_I_VAL;
    g_imu_regs[IMU_CTRL1_XL_REG] = 0x94;
    g_imu_regs[IMU_CTRL2_G_REG] = 0x9D;

    g_highg_l_regs[HIGHG_WHO_AM_I_REG] = HIGHG_WHO_AM_I_VAL;
    g_highg_r_regs[HIGHG_WHO_AM_I_REG] = HIGHG_WHO_AM_I_VAL;
    g_highg_l_regs[HIGHG_CTRL1_REG] = 0x27;
    g_highg_r_regs[HIGHG_CTRL1_REG] = 0x27;

    g_kp200_l_regs[KP200_WHO_AM_I_REG] = KP200_L_WHO_AM_I_VAL;
    g_kp200_r_regs[KP200_WHO_AM_I_REG] = KP200_R_WHO_AM_I_VAL;
    g_kp200_l_regs[KP200_CTRL_REG] = 0x01;
    g_kp200_r_regs[KP200_CTRL_REG] = 0x01;
}

static void refresh_live_registers(void)
{
    uint8_t scenario = g_profile_active ? g_profile_scenario : SCENARIO_IDLE;
    uint8_t fault = scenario_fault_active();
    uint16_t tick = g_profile_tick;
    int16_t imu_ax = 0;
    int16_t imu_ay = 0;
    int16_t imu_az = imu_raw_from_g(10);
    int16_t imu_gx = 0;
    int16_t imu_gy = 0;
    int16_t imu_gz = 0;
    int16_t highg_l_ax = 0;
    int16_t highg_l_ay = 0;
    int16_t highg_l_az = 0;
    int16_t highg_r_ax = 0;
    int16_t highg_r_ay = 0;
    int16_t highg_r_az = 0;
    int16_t kp200_l_pa = 0;
    int16_t kp200_r_pa = 0;

    switch (scenario) {
    case SCENARIO_FRONT:
        imu_ax = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 18U, 12U, 80U, -140));
        highg_l_ax = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 10U, 14U, 70U, -210));
        highg_r_ax = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 10U, 14U, 70U, -210));
        break;

    case SCENARIO_SIDE_LEFT:
        imu_ay = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 8U, 8U, 30U, 150));
        highg_l_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 6U, 8U, 25U, 1500));
        highg_r_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 6U, 8U, 25U, 120));
        kp200_l_pa = shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 5U, 12U, 30U, 14000);
        kp200_r_pa = shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 5U, 10U, 20U, 800);
        break;

    case SCENARIO_SIDE_RIGHT:
        imu_ay = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 8U, 8U, 30U, -150));
        highg_l_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 6U, 8U, 25U, -120));
        highg_r_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 6U, 8U, 25U, -1500));
        kp200_l_pa = shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 5U, 10U, 20U, 800);
        kp200_r_pa = shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 5U, 12U, 30U, 14000);
        break;

    case SCENARIO_REAR:
        imu_ax = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 30U, 60U, 100U, 55));
        highg_l_ax = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 24U, 60U, 90U, 110));
        highg_r_ax = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 24U, 60U, 90U, 110));
        break;

    case SCENARIO_ROLLOVER:
        imu_gx = imu_raw_from_dps(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 200U, 300U, 300U, 120));
        imu_ay = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 40U, 120U, 240U, 220U, 30));
        imu_az = imu_raw_from_g(10 - shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 80U, 200U, 180U, 200U, 7));
        highg_l_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 40U, 120U, 240U, 220U, 30));
        highg_r_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 40U, 120U, 240U, 220U, 30));
        break;

    case SCENARIO_MISUSE:
        imu_ax = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 3U, 1U, 4U, -110) +
                                shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 12U, 3U, 1U, 4U, 80));
        imu_ay = imu_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 6U, 3U, 1U, 4U, 120) +
                                shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 18U, 3U, 1U, 4U, -110));
        imu_gx = imu_raw_from_dps(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 4U, 6U, 2U, 10U, 18));
        highg_l_ax = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 2U, 1U, 3U, -18));
        highg_r_ax = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS, 2U, 1U, 3U, -18));
        highg_l_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 6U, 2U, 1U, 3U, 35));
        highg_r_ay = highg_raw_from_g(shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 6U, 2U, 1U, 3U, 35));
        kp200_l_pa = shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 8U, 3U, 1U, 5U, 1800);
        kp200_r_pa = shaped_pulse(tick, PROFILE_PREDELAY_TICKS + 8U, 3U, 1U, 5U, 1800);
        break;

    case SCENARIO_VISUAL:
        /* Sensor values stay benign; STM32 uses this code to run actuator visual test. */
        break;

    case SCENARIO_IDLE:
    default:
        break;
    }

    g_imu_regs[IMU_STATUS_REG] = fault ? 0x00 : EMU_STATUS_DRDY;
    g_highg_l_regs[HIGHG_STATUS_REG] = fault ? 0x00 : EMU_STATUS_DRDY;
    g_highg_r_regs[HIGHG_STATUS_REG] = fault ? 0x00 : EMU_STATUS_DRDY;
    g_kp200_l_regs[KP200_STATUS_REG] = fault ? EMU_STATUS_FAULT : EMU_STATUS_DRDY;
    g_kp200_r_regs[KP200_STATUS_REG] = fault ? EMU_STATUS_FAULT : EMU_STATUS_DRDY;
    g_kp200_l_regs[KP200_FAULT_REG] = fault ? KP200_FAULT_INJECTED : KP200_FAULT_NONE;
    g_kp200_r_regs[KP200_FAULT_REG] = fault ? KP200_FAULT_INJECTED : KP200_FAULT_NONE;

    set_reg16(g_imu_regs, IMU_OUTX_L_G_REG + 0U, imu_gx);
    set_reg16(g_imu_regs, IMU_OUTX_L_G_REG + 2U, imu_gy);
    set_reg16(g_imu_regs, IMU_OUTX_L_G_REG + 4U, imu_gz);
    set_reg16(g_imu_regs, IMU_OUTX_L_A_REG + 0U, imu_ax);
    set_reg16(g_imu_regs, IMU_OUTX_L_A_REG + 2U, imu_ay);
    set_reg16(g_imu_regs, IMU_OUTX_L_A_REG + 4U, imu_az);

    set_reg16(g_highg_l_regs, HIGHG_OUT_X_L_REG + 0U, highg_l_ax);
    set_reg16(g_highg_l_regs, HIGHG_OUT_X_L_REG + 2U, highg_l_ay);
    set_reg16(g_highg_l_regs, HIGHG_OUT_X_L_REG + 4U, highg_l_az);

    set_reg16(g_highg_r_regs, HIGHG_OUT_X_L_REG + 0U, highg_r_ax);
    set_reg16(g_highg_r_regs, HIGHG_OUT_X_L_REG + 2U, highg_r_ay);
    set_reg16(g_highg_r_regs, HIGHG_OUT_X_L_REG + 4U, highg_r_az);

    g_kp200_l_regs[KP200_PRESSURE_H_REG] = (uint8_t)(((uint16_t)kp200_l_pa >> 8) & 0xFFU);
    g_kp200_l_regs[KP200_PRESSURE_L_REG] = (uint8_t)(kp200_l_pa & 0xFFU);
    g_kp200_r_regs[KP200_PRESSURE_H_REG] = (uint8_t)(((uint16_t)kp200_r_pa >> 8) & 0xFFU);
    g_kp200_r_regs[KP200_PRESSURE_L_REG] = (uint8_t)(kp200_r_pa & 0xFFU);
}

static uint8_t read_reg(uint8_t dev, uint8_t addr)
{
    volatile uint8_t *regs = regs_for_device(dev);

    if (!regs) {
        return 0xEE;
    }

    return regs[addr & 0x7FU];
}

static void write_reg(uint8_t dev, uint8_t addr, uint8_t value)
{
    volatile uint8_t *regs = regs_for_device(dev);

    if (!regs) {
        return;
    }

    addr &= 0x7FU;

    switch (dev) {
    case DEV_IMU:
        if ((addr == IMU_CTRL1_XL_REG) || (addr == IMU_CTRL2_G_REG)) {
            regs[addr] = value;
        }
        break;

    case DEV_HIGHG_L:
    case DEV_HIGHG_R:
        if (addr == HIGHG_CTRL1_REG) {
            regs[addr] = value;
        }
        break;

    case DEV_KP200_L:
    case DEV_KP200_R:
        if (addr == KP200_CTRL_REG) {
            regs[addr] = value;
        } else if (addr == KP200_FAULT_REG) {
            regs[addr] = value;
        }
        break;

    default:
        break;
    }
}

static void begin_virtual_transaction(void)
{
    g_active_device = DEV_NONE;
    g_transaction_active = 1U;
    g_transaction_byte_index = 0U;
    g_reg_addr = 0x00;
    g_is_read = 0U;
    spi_miso_drive(0U);
    SPDR = 0x00;
}

static void end_virtual_transaction(void)
{
    spi_miso_release();
    g_active_device = DEV_NONE;
    g_transaction_active = 0U;
    g_transaction_byte_index = 0U;
    g_reg_addr = 0x00;
    g_is_read = 0U;
    SPDR = 0x00;
}

static uint8_t handle_device_byte(uint8_t byte)
{
    volatile uint8_t *regs;

    if (g_transaction_byte_index == 0U) {
        g_active_device = get_selected_device();
        g_is_read = (byte & EMU_READ_BIT) ? 1U : 0U;
        g_reg_addr = (uint8_t)(byte & 0x7FU);
        regs = regs_for_device(g_active_device);

        if (regs == 0) {
            return 0xEE;
        }

        if (g_is_read) {
            return read_reg(g_active_device, g_reg_addr);
        } else {
            return 0x00;
        }
    }

    regs = regs_for_device(g_active_device);
    if (regs == 0) {
        return 0xEE;
    }

    if (g_is_read) {
        g_reg_addr++;
        return read_reg(g_active_device, g_reg_addr);
    } else {
        write_reg(g_active_device, g_reg_addr, byte);
        g_reg_addr++;
        return 0x00;
    }
}

static void gpio_init(void)
{
    DDRB &= ~((1 << SPI_SS_BIT) | (1 << SPI_MOSI_BIT) | (1 << SPI_MISO_BIT) | (1 << SPI_SCK_BIT));
    PORTB |= (1 << SPI_SS_BIT);
    PORTB &= ~((1 << SPI_MOSI_BIT) | (1 << SPI_MISO_BIT) | (1 << SPI_SCK_BIT));

    DDRD &= ~((1 << CS_IMU_BIT) |
              (1 << CS_HIGHG_L_BIT) |
              (1 << CS_HIGHG_R_BIT) |
              (1 << CS_KP200_L_BIT) |
              (1 << CS_KP200_R_BIT));
    PORTD |= ((1 << CS_IMU_BIT) |
              (1 << CS_HIGHG_L_BIT) |
              (1 << CS_HIGHG_R_BIT) |
              (1 << CS_KP200_L_BIT) |
              (1 << CS_KP200_R_BIT));

    DDRC &= ~0x3FU;
    PORTC &= ~0x3FU;

    spi_miso_release();
}

static void spi_slave_init(void)
{
    SPCR = (1 << SPE) | (1 << SPIE);
    SPSR = 0x00;
    SPDR = 0x00;
}

static void pcint_init(void)
{
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << SPI_SS_BIT);
}

static void timer1_init_2khz(void)
{
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    OCR1A = 499U;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
}

static void scenario_engine_tick(void)
{
    uint8_t start = scenario_started();
    uint8_t reset = scenario_reset_active();

    if (reset || !start) {
        g_profile_active = 0U;
        g_profile_scenario = SCENARIO_IDLE;
        g_profile_tick = 0U;
    } else {
        if (!g_prev_start) {
            g_profile_active = 1U;
            g_profile_scenario = scenario_code();
            g_profile_tick = 0U;
        } else if (g_profile_active) {
            if (g_profile_tick < PROFILE_MAX_TICKS) {
                g_profile_tick++;
            }
        }
    }

    g_prev_start = start;
    refresh_live_registers();
}

ISR(PCINT0_vect)
{
    if (!sensorhub_is_selected()) {
        end_virtual_transaction();
        return;
    }

    if (!g_transaction_active) {
        begin_virtual_transaction();
    }
}

ISR(SPI_STC_vect)
{
    uint8_t byte;
    uint8_t tx = 0x00;

    (void)SPSR;
    byte = SPDR;

    if (!g_transaction_active) {
        SPDR = 0x00;
        return;
    }

    tx = handle_device_byte(byte);
    g_transaction_byte_index++;
    SPDR = tx;
}

ISR(TIMER1_COMPA_vect)
{
    scenario_engine_tick();
}

int main(void)
{
    gpio_init();
    spi_slave_init();
    init_register_defaults();
    refresh_live_registers();
    pcint_init();
    timer1_init_2khz();
    sei();

    while (1) {
        /* Transport is interrupt-driven; all live sensor values refresh at transaction start. */
    }
}
