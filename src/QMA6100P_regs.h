// QMA6100P_regs.h
//
// This file holds the bit fields for the QMA6100P registers.

#define SFE_QMA6100P_CHIP_ID  0x00 // Returns "KION" in ASCII

/**
 * @brief Structure to hold the chip ID.
 */
typedef struct
{
  uint8_t chip_id : 8;
} sfe_qma6100p_chip_id_t;

#define SFE_QMA6100P_DX_L 0x01

/**
 * @brief Structure to hold the lower 6 bits of the X-axis acceleration data.
 * @details
 * - DX: 14-bit acceleration data of the X-channel in two's complement.
 * - NEWDATA_X: 1 if X-channel data has been updated since last reading, 0 otherwise.
 */
typedef struct
{
  uint8_t newdata_x : 1;
  uint8_t blank : 1;
  uint8_t dx_l : 6;
} sfe_qma6100p_dx_l_t;

#define SFE_QMA6100P_DX_H 0x02

/**
 * @brief Structure to hold the upper 8 bits of the X-axis acceleration data.
 */
typedef struct
{
  uint8_t dx_h : 8;
} sfe_qma6100p_dx_h_t;

#define SFE_QMA6100P_DY_L 0x03

/**
 * @brief Structure to hold the lower 6 bits of the Y-axis acceleration data.
 * @details
 * - DY: 14-bit acceleration data of the Y-channel in two's complement.
 * - NEWDATA_Y: 1 if Y-channel data has been updated since last reading, 0 otherwise.
 */
typedef struct
{
  uint8_t newdata_y : 1;
  uint8_t blank : 1;
  uint8_t dy_l : 6;
} sfe_qma6100p_dy_l_t;

#define SFE_QMA6100P_DY_H 0x04

/**
 * @brief Structure to hold the upper 8 bits of the Y-axis acceleration data.
 */
typedef struct
{
  uint8_t dy_h : 8;
} sfe_qma6100p_dy_h_t;

#define SFE_QMA6100P_DZ_L 0x05

/**
 * @brief Structure to hold the lower 6 bits of the Z-axis acceleration data.
 * @details
 * - DZ: 14-bit acceleration data of the Z-channel in two's complement.
 * - NEWDATA_Z: 1 if Z-channel data has been updated since last reading, 0 otherwise.
 */
typedef struct
{
  uint8_t newdata_z : 1;
  uint8_t blank : 1;
  uint8_t dz_l : 6;
} sfe_qma6100p_dz_l_t;

#define SFE_QMA6100P_DZ_H 0x06

/**
 * @brief Structure to hold the upper 8 bits of the Z-axis acceleration data.
 */
typedef struct
{
  uint8_t dz_h : 8;
} sfe_qma6100p_dz_h_t;

#define SFE_QMA6100P_INT_ST0  0x09

/**
 * @brief Structure to hold the interrupt status for various functions.
 * @details
 * - NO_MOT: 1 if no_motion interrupt is active, 0 otherwise.
 * - STEP_FLAG: 1 if STEP is detected, 0 otherwise.
 * - ANY_MOT_SIGN: 1 if the sign of any_motion triggering signal is negative, 0 if positive.
 * - ANY_MOT_FIRST_Z: 1 if any_motion interrupt is triggered by the Z axis, 0 otherwise.
 * - ANY_MOT_FIRST_Y: 1 if any_motion interrupt is triggered by the Y axis, 0 otherwise.
 * - ANY_MOT_FIRST_X: 1 if any_motion interrupt is triggered by the X axis, 0 otherwise.
 */
typedef struct
{
  uint8_t no_mot : 1;
  uint8_t step_flag : 1;
  uint8_t blank : 2;
  uint8_t any_mot_sign : 1;
  uint8_t any_mot_first_z : 1;
  uint8_t any_mot_first_y : 1;
  uint8_t any_mot_first_x : 1;
} sfe_qma6100p_int_st0_t;

#define SFE_QMA6100P_INT_ST1  0x0a

/**
 * @brief Structure to hold the interrupt status for various functions.
 * @details
 * - S_TAP_INT: 1 if single tap is active, 0 otherwise.
 * - SIG_STEP: 1 if significant step is active, 0 otherwise.
 * - D_TAP_INT: 1 if double tap is active, 0 otherwise.
 * - STEP_INT: 1 if step valid interrupt is active, 0 if step quit interrupt is inactive.
 * - T_TAP_INT: 1 if triple tap is active, 0 otherwise.
 * - HD_INT: 1 if hand down interrupt is active, 0 otherwise.
 * - RAISE_INT: 1 if raise hand interrupt is active, 0 otherwise.
 * - SIG_MOT_INT: 1 if significant interrupt is active, 0 otherwise.
 */
typedef struct
{
  uint8_t s_tap_int : 1;
  uint8_t sig_step : 1;
  uint8_t d_tap_int : 1;
  uint8_t t_tap_int : 1;
  uint8_t step_int : 1;
  uint8_t hd_int : 1;
  uint8_t raie_int : 1;
  uint8_t sig_mot_int : 1;
} sfe_qma6100p_int_st1_t;

/**
 * @brief Union to access the interrupt status bits or the entire byte.
 */
typedef union
{
  uint8_t all;
  sfe_qma6100p_int_st1_t bits;
} sfe_qma6100p_int_st1_bitfield_t;

#define SFE_QMA6100P_INT_ST2  0x0b

/**
 * @brief Structure to hold the interrupt status for various functions.
 * @details
 * - FIFO_OR: 1 if FIFO Over-Run occurred, 0 otherwise.
 * - FIFO_WM_INT: 1 if FIFO watermark interrupt is active, 0 otherwise.
 * - FIFO_FULL_INT: 1 if FIFO full interrupt is active, 0 otherwise.
 * - DATA_INT: 1 if data ready interrupt is active, 0 otherwise.
 * - EARIN_FLAG: 1 if ear-in interrupt is active, 0 otherwise.
 * - Q_TAP_INT: 1 if quad tap is active, 0 otherwise.
 */
typedef struct
{
  uint8_t fifo_or : 1;
  uint8_t fifo_wm_int : 1;
  uint8_t fifo_full_int : 1;
  uint8_t data_int : 1; // data ready int
  uint8_t blank : 2;
  uint8_t earin_flag : 1;
  uint8_t q_tap_int : 1;
} sfe_qma6100p_int_st2_t;

#define SFE_QMA6100P_INT_ST3  0x0c

#define SFE_QMA6100P_INT_ST4  0x0d

#define SFE_QMA6100P_FIFO_ST  0x0e

/**
 * @brief Structure to hold the FIFO frame counter.
 */
typedef struct
{
  uint8_t fifo_frame_counter : 8;
} sfe_qma6100p_fifo_st_t;

#define SFE_QMA6100P_FSR 0x0f

/**
 * @brief Structure to hold the full scale range and filter settings.
 * @details
 * - RANGE: 4-bit field to set the full scale range of the accelerometer.
 * - LPF_HPF: 1-bit field to set the low-pass/high-pass filter.
 */
typedef struct
{
  uint8_t range : 4;
  uint8_t blank : 3;
  uint8_t lpf_hpf : 1;
} sfe_qma6100p_fsr_t;

/**
 * @brief Union to access the full scale range and filter settings or the entire byte.
 */
typedef union
{
  uint8_t all;
  sfe_qma6100p_fsr_t bits;
} sfe_qma6100p_fsr_bitfield_t;

#define SFE_QMA6100P_BW 0x10

#define SFE_QMA6100P_PM 0x11

/**
 * @brief Structure to hold the power mode and clock settings.
 * @details
 * - MODE_BIT: 1 to set the device into active mode, 0 to set it into standby mode.
 * - T_RSTB_SINC_SEL: 2-bit field to set the reset clock setting.
 * - MCLK_SEL: 4-bit field to set the master clock.
 */
typedef struct
{
  uint8_t mclk_sel : 4;
  uint8_t t_rstb_sinc_sel : 2;
  uint8_t blank : 1;
  uint8_t mode_bit : 1;
} sfe_qma6100p_pm_t;

/**
 * @brief Union to access the power mode and clock settings or the entire byte.
 */
typedef union
{
  uint8_t all;
  sfe_qma6100p_pm_t bits;
} sfe_qma6100p_pm_bitfield_t;

#define SFE_QMA6100P_STEP_CONF0 0x12
#define SFE_QMA6100P_STEP_CONF1 0x13
#define SFE_QMA6100P_STEP_CONF2 0x14
#define SFE_QMA6100P_STEP_CONF3 0x15

#define SFE_QMA6100P_INT_EN0  0x16

#define SFE_QMA6100P_INT_EN1  0x17

/**
 * @brief Structure to hold the interrupt enable settings.
 * @details
 * - INT_FWM_EN: 1 to enable FIFO watermark interrupt, 0 to disable.
 * - INT_FFULL_EN: 1 to enable FIFO full interrupt, 0 to disable.
 * - INT_DATA_EN: 1 to enable data ready interrupt, 0 to disable.
 */
typedef struct
{
  uint8_t int_fwm_en : 1;
  uint8_t int_ffull_en : 1;
  uint8_t int_data_en : 1;
  uint8_t blank : 4;
} sfe_qma6100p_int_en1_t;

/**
 * @brief Union to access the interrupt enable settings or the entire byte.
 */
typedef union
{
  uint8_t all;
  sfe_qma6100p_int_en1_t bits;
} sfe_qma6100p_int_en1_bitfield_t;

#define SFE_QMA6100P_INT_EN2  0x18

#define SFE_QMA6100P_INT_MAP0 0x19

#define SFE_QMA6100P_INT_MAP1 0x1a

/**
 * @brief Structure to hold the interrupt mapping settings for INT1 pin.
 * @details
 * - INT1_NO_MOT: 1 to map no_motion interrupt to INT1 pin, 0 otherwise.
 * - INT1_FWM: 1 to map FIFO watermark interrupt to INT1 pin, 0 otherwise.
 * - INT1_FFULL: 1 to map FIFO full interrupt to INT1 pin, 0 otherwise.
 * - INT1_DATA: 1 to map data ready interrupt to INT1 pin, 0 otherwise.
 * - INT1_Q_TAP: 1 to map quad tap interrupt to INT1 pin, 0 otherwise.
 * - INT1_ANY_MOT: 1 to map any motion interrupt to INT1 pin, 0 otherwise.
 */
typedef struct
{
  uint8_t int1_no_mot : 1;
  uint8_t int1_fwm : 1;
  uint8_t int1_ffull : 1;
  uint8_t int1_data : 1;
  uint8_t blank : 2;
  uint8_t int1_q_tap : 1;
  uint8_t int1_any_mot : 1;
} sfe_qma6100p_int_map1_t;

/**
 * @brief Union to access the interrupt mapping settings for INT1 pin or the entire byte.
 */
typedef union
{
  uint8_t all;
  sfe_qma6100p_int_map1_t bits;
} sfe_qma6100p_int_map1_bitfield_t;

#define SFE_QMA6100P_INT_MAP2 0x1b
#define SFE_QMA6100P_INT_MAP3 0x1c

/**
 * @brief Structure to hold the interrupt mapping settings for INT2 pin.
 * @details
 * - INT2_NO_MOT: 1 to map no_motion interrupt to INT2 pin, 0 otherwise.
 * - INT2_FWM: 1 to map FIFO watermark interrupt to INT2 pin, 0 otherwise.
 * - INT2_FFULL: 1 to map FIFO full interrupt to INT2 pin, 0 otherwise.
 * - INT2_DATA: 1 to map register data ready interrupt to INT2 pin, 0 otherwise.
 * - INT2_Q_TAP: 1 to map quad tap interrupt to INT2 pin, 0 otherwise.
 * - INT2_ANY_MOT: 1 to map any motion interrupt to INT2 pin, 0 otherwise.
 */
typedef struct
{
  uint8_t int2_no_mot : 1;
  uint8_t int2_fwm : 1;
  uint8_t int2_ffull : 1;
  uint8_t int2_data : 1;
  uint8_t blank : 2;
  uint8_t int2_q_tap : 1;
  uint8_t int2_any_mot : 1;
} sfe_qma6100p_int_map3_t;

/**
 * @brief Union to access the interrupt mapping settings for INT2 pin or the entire byte.
 */
typedef union
{
  uint8_t all;
  sfe_qma6100p_int_map3_t bits;
} sfe_qma6100p_int_map3_bitfield_t;

#define SFE_QMA6100P_STEP_CFG0  0x1d
#define SFE_QMA6100P_STEP_CFG1  0x1e

#define SFE_QMA6100P_STEP_COUNTER 0x1f

#define SFE_QMA6100P_INTPINT_CONF 0x20

#define SFE_QMA6100P_INT_CFG  0x21

#define SFE_QMA6100P_REG_22 0x22
#define SFE_QMA6100P_REG_23 0x23
#define SFE_QMA6100P_REG_24 0x24
#define SFE_QMA6100P_REG_25 0x25
#define SFE_QMA6100P_REG_26 0x26

#define SFE_QMA6100P_OS_CUST_X  0x27
#define SFE_QMA6100P_OS_CUST_Y  0x28
#define SFE_QMA6100P_OS_CUST_X  0x27

#define SFE_QMA6100P_REG_2A 0x2a
#define SFE_QMA6100P_REG_2B 0x2B

#define SFE_QMA6100P_MOT_CONF0  0x2c
#define SFE_QMA6100P_MOT_CONF1  0x2d
#define SFE_QMA6100P_MOT_CONF2  0x2e
#define SFE_QMA6100P_MOT_CONF3  0x2f

#define SFE_QMA6100P_REG_30 0x30
#define SFE_QMA6100P_REG_31 0x31

#define SFE_QMA6100P_ST 0x32

#define SFE_QMA6100P_REG_34 0x34
#define SFE_QMA6100P_REG_35 0x35

#define SFE_QMA6100P_SR 0x36  // writing 0xB6, soft reset all of the registers. 
                              // After soft-reset, user should write 0x00 back
typedef struct 
{
  uint8_t soft_reset : 8;
} sfe_qma6100p_sr_t;

typedef union
{
  uint8_t all;
} sfe_qma6100p_sr_bitfeild_t;

#define SFE_QMA6100P_FIFO_CFG0 0x3e
/*
FIFO_MODE<1:0>: FIFO_MODE defines FIFO mode of the device. Settings as following
0b11 FIFO
0b10 STREAM
0b01 FIFO
0b00 BYPASS
RAISE_XYZ_SW<2:0>
0x3E[2:0]: User can select the acceleration data of which axis to be stored in the FIFO. 
This configuration can be done by setting FIFO_CH, where ‘111b’ for x-, y-, and 
z-axis, ‘001b’ for x-axis only, ‘010b’ for y-axis only, ‘100b’ for z-axis only
*/
typedef struct
{
  uint8_t fifo_mode : 2;
  uint8_t raise_xyz_sw  : 3;
  uint8_t fifo_en_xyz : 3;
} sfe_qma6100p_fifo_cfg0_t;

typedef union
{
  uint8_t all;
  sfe_qma6100p_fifo_cfg0_t bits;
} sfe_qma6100p_fifo_cfg0_bitfield_t;

#define SFE_QMA6100P_FIFO_DATA 0x3f
/*
FIFO_DATA<7:0>: FIFO read out data. User can read out FIFO data through this register. Data format depends on the setting of FIFO_CH (0x3e<2:0>).
When the FIFO data is the LSB part of acceleration data, and if FIFO is empty, then FIFO_DATA<0> is 0. Otherwise if FIFO is not empty and the data is effective, 
FIFO_DATA<0> is 1 when reading LSB of acceleration.
*/
typedef struct 
{
  uint8_t fifo_data : 8;
} sfe_qma6100p_fifo_cfg0;