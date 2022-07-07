#ifndef DRV8711_H
#define DRV8711_H

extern void drv8711_init();
extern void enableSteppers();
extern void disableSteppers();

typedef enum {
    MODE_FULL_STEP = 0b0000,
    MODE_HALF_STEP = 0b0001,
    MODE_QUARTER_STEP = 0b0010,
    MODE_EIGHTH_STEP = 0b0011,
    MODE_SIXTEENTH_STEP = 0b0100,
    MODE_THIRTY_SECOND_STEP = 0b0101,
    MODE_SIXTY_FOURTH_STEP = 0b0110,
    MODE_ONE_HUNDRED_TWENTY_EIGHTH_STEP = 0b0111,
    MODE_TWO_HUNDRED_FIFTY_SIXTH_STEP = 0b1000
} DRV_MODE_t;

typedef enum {
    ISGAIN_5 = 0b00,
    ISGAIN_10 = 0b01,
    ISGAIN_20 = 0b10,
    ISGAIN_40 = 0b11
} DRV_ISGAIN_t;

typedef enum {
    DTIME_400_ns = 0b00,
    DTIME_450_ns = 0b01,
    DTIME_650_ns = 0b10,
    DTIME_850_ns = 0b11
} DRV_DTIME_t;

typedef enum {
    SMPLTH_50_us = 0b000,
    SMPLTH_100_us = 0b001,
    SMPLTH_200_us = 0b010,
    SMPLTH_300_us = 0b011,
    SMPLTH_400_us = 0b100,
    SMPLTH_600_us = 0b101,
    SMPLTH_800_us = 0b110,
    SMPLTH_1000_us = 0b111
} DRV_SMPLTH_t;

typedef enum {
    ABT_OFF = 0,
    ABT_ON = 1
} DRV_ABT_t;

typedef enum {
    DECMOD_SLOW_ALL = 0b000,
    DECMOD_SLOW_INC_MIXED_DEC = 0b001,
    DECMOD_FAST_ALL = 0b010,
    DECMOD_MIXED_ALL = 0b011,
    DECMOD_SLOW_INC_AUTO_MIXED_DEC = 0b100,
    DECMOD_AUTO_MIXED_ALL = 0b101
} DRV_DECMOD_t;

typedef enum {
    SDCNT_1_STEP = 0b00,
    SDCNT_2_STEPS = 0b01,
    SDCNT_4_STEPS = 0b10,
    SDCNT_8_STEPS = 0b11
} DRV_SDCNT_t;

typedef enum {
    VDIV_32 = 0b00,
    VDIV_16 = 0b01,
    VDIV_8 = 0b10,
    VDIV_4 = 0b11
} DRV_VDIV_t;

typedef enum {
    OCPTH_250_mV = 0b00,
    OCPTH_500_mV = 0b01,
    OCPTH_750_mV = 0b10,
    OCPTH_1000_mV = 0b11
} DRV_OCPTH_t;

typedef enum {
    OCPDEG_1_us = 0b00,
    OCPDEG_2_us = 0b01,
    OCPDEG_4_us = 0b10,
    OCPDEG_8_us = 0b11
} DRV_OCPDEG_t;

typedef enum {
    TDRIVEN_250_ns = 0b00,
    TDRIVEN_500_ns = 0b01,
    TDRIVEN_1_us = 0b10,
    TDRIVEN_2_us = 0b11
} DRV_TDRIVEN_t;

typedef enum {
    TDRIVEP_250_ns = 0b00,
    TDRIVEP_500_ns = 0b01,
    TDRIVEP_1_us = 0b10,
    TDRIVEP_2_us = 0b11
} DRV_TDRIVEP_t;

typedef enum {
    IDRIVEN_100_mA = 0b00,
    IDRIVEN_200_mA = 0b01,
    IDRIVEN_300_mA = 0b10,
    IDRIVEN_400_mA = 0b11
} DRV_IDRIVEN_t;

typedef enum {
    IDRIVEP_50_mA = 0b00,
    IDRIVEP_100_mA = 0b01,
    IDRIVEP_150_mA = 0b10,
    IDRIVEP_200_mA = 0b11
} DRV_IDRIVEP_t;

#endif
