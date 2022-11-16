#include <avr/io.h>
#include "spi.h"
#include "grbl.h"
#include "config.h"

uint16_t setMicrostep_Mask( uint16_t desiredRate );
uint8_t calculate_ISGAIN();
uint8_t calculate_ISGAIN_variable_I_FS( float I_FS_val );
uint16_t setISGAIN_Mask( uint8_t gain_val );
uint16_t setTORQUE_Mask( uint8_t gain_val );
uint16_t setTORQUE_Mask_variable_I_FS( uint8_t gain_val, float I_FS_val );

uint16_t setDRV8711( uint16_t address, uint16_t Data );
uint16_t readDRV8711( uint16_t address );

#define CTRL    0x0000
#define TORQUE  0x1000
#define OFF     0x2000
#define BLANK   0x3000
#define DECAY   0x4000
#define STALL   0x5000
#define DRIVE   0x6000
#define STATUS  0x7000

#define ENABLE_CTRL   0x1U
#define DISABLE_CTRL  0xFFE

#define R_SENSE               0.02f     // This is the value of the feedback resistor to sense stepper current
#define I_FS                  3.0f      // This is the desired current setpoint for the motor
#define TORQUE_REG_BIT_WIDTH  8         // Bit-width of the TORQUE value to be written to register

// DRV8711 register values that will be altered before writing values to chips
#define CTRL_DEFAULT    0b110000000000    // Initial value: disabled motor, dirction set with DIR pin, full-step, internal stall detect, ISGAIN of 5
#define TORQUE_DEFAULT  0x100             // Initial value: TORQUE value of 0 (TORQUE and ISGAIN values will be calculated), 100 us back-EMF sample threshold

// DRV8711 register values that are ready to be written to chips
#define OFF_VAL         0x32              // OFF time of 25.5 us, use internal indexer
#define BLANK_VAL       0x100             // Enable adaptive blanking time, current trip blanking time of 1 us
#define DECAY_VAL       0x510             // Mixed decay transition time of 8 us, use auto-mixed decay at all times
#define STALL_VAL       0xA02             // Stall asserted after 4 steps, back-EMF divided by 8
#define DRIVE_VAL       0x000             // OCP threshold of 250 mV, OCP deglitch time of 1 us, low-side gate drive time of 250 ns, high-side gate drive time of 250 ns,\
                                              low-side gate drive peak current of 100 mA, high-side gate drive peak current of 50 mA

uint16_t x_ctrl_val = CTRL_DEFAULT;
uint16_t y_ctrl_val = CTRL_DEFAULT;
uint16_t dual_ctrl_val = CTRL_DEFAULT;
uint16_t z_ctrl_val = CTRL_DEFAULT;

#ifdef BALL_SCREW_TESTING
// Create a structure type to hold axis-specific DRV configuration information
typedef struct xcp_axis {
  uint16_t axis_ctrl_reg_value;
  uint16_t axis_torque_reg_value;
} xcp_axis;

// Define the number of motor driver channels with unique control inputs from the micro.
// For XCP, this is 4 because while the Y-axes are always in sync, they do not share STEP/DIR lines and instead have unique control pins that mirror each other
#ifdef ENABLE_DUAL_AXIS
  #define NUM_AXES  4
#else
  #define NUM_AXES  3
#endif

// Initialize a list of xcp_axis variables, one for each motor driver channel
xcp_axis axis_list[NUM_AXES];

// Create a list of the control select bits for SPI bus corresponding to the axes order in the axis_list
uint8_t csBit_list[NUM_AXES] = {CS_X_BIT, CS_Y_BIT, CS_DUAL_BIT, CS_Z_BIT};

// Create a list of the microstep values corresponding to the axes order in the axis_list
unsigned int microstep_list[NUM_AXES] = {MICROSTEPS_X, MICROSTEPS_Y, MICROSTEPS_Y, MICROSTEPS_Z};

// Create a list of the motor current setpoints corresponding to the axes order in the axis_list
float i_fs_list[NUM_AXES] = {CONFIG_CURRENT_X, CONFIG_CURRENT_Y, CONFIG_CURRENT_Y, CONFIG_CURRENT_Z};
#endif


void drv8711_init()
{
  uint8_t csBit;
  
  CS_DDR |= CS_MASK;            // Set the chip select pins as outputs
  CS_PORT &= ~(CS_MASK);        // Set the pin states to low (chips not selected)

  RESET_DDR |= RESET_MASK;      // Set the reset pins as outputs
  RESET_PORT &= ~(RESET_MASK);  // Set the pin states to low (active high reset)

  FAULT_DDR &= ~(FAULT_MASK);   // Set the fault pins as inputs
  FAULT_PORT |= (FAULT_MASK);   // Enable internal pull-up resistors (active low signals)

  STALL_DDR &= ~(STALL_MASK);   // Set the stall pins as inputs
  STALL_PORT |= (STALL_MASK);   // Enable internal pull-up resistors (active low signals)
  

  spi_init();  // initialize the SPI port


  #ifdef BALL_SCREW_TESTING
    // Iterate over the number of axes to initialize the xcp_axis list
    for( uint8_t i=0; i<NUM_AXES; i++ )
    {
      axis_list[i].axis_ctrl_reg_value = CTRL_DEFAULT;
      axis_list[i].axis_torque_reg_value = TORQUE_DEFAULT;
    }

    // For each individual hardware channel configure the register contents to be written to the DRV chip
    for( uint8_t i=0; i<NUM_AXES; i++ )
    {
      // Mask axis control register value with desired microstepping rate, as defined by default.h
      axis_list[i].axis_ctrl_reg_value |= setMicrostep_Mask( microstep_list[i] );

      // Calculate the value of IS_GAIN to set in CTRL register, based on I_FS for the axis and R_SENSE populated on PCB
      uint8_t is_gain_val = calculate_ISGAIN_variable_I_FS( i_fs_list[i] );

      // Mask the axis control register value with the desired IS_GAIN value
      axis_list[i].axis_ctrl_reg_value |= setISGAIN_Mask( is_gain_val );

      // Calculate the TORQUE value and set the mask to be applied to the TORQUE register contents
      axis_list[i].axis_torque_reg_value |= setTORQUE_Mask_variable_I_FS( is_gain_val, i_fs_list[i] );
    }

    // Write the register contents to each DRV chip
    for( uint8_t i=0; i<NUM_AXES; i++ )
    {
      delay_ms(20);   // give a little time before SPI starts
      spi_send16bit( setDRV8711( CTRL, axis_list[i].axis_ctrl_reg_value ), &CS_PORT, csBit_list[i] );
      spi_send16bit( setDRV8711( TORQUE, axis_list[i].axis_torque_reg_value ), &CS_PORT, csBit_list[i] );
      spi_send16bit( setDRV8711( OFF, OFF_VAL ), &CS_PORT, csBit_list[i] );
      spi_send16bit( setDRV8711( BLANK, BLANK_VAL ), &CS_PORT, csBit_list[i] );
      spi_send16bit( setDRV8711( DECAY, DECAY_VAL ), &CS_PORT, csBit_list[i] );
      spi_send16bit( setDRV8711( STALL, STALL_VAL ), &CS_PORT, csBit_list[i] );
      spi_send16bit( setDRV8711( DRIVE, DRIVE_VAL ), &CS_PORT, csBit_list[i] );
    }

  #else
  // Mask axis control register values with desired microstepping rates, as defined by default.h
  x_ctrl_val |= setMicrostep_Mask( MICROSTEPS_XY );
  y_ctrl_val |= setMicrostep_Mask( MICROSTEPS_XY );
  dual_ctrl_val |= setMicrostep_Mask( MICROSTEPS_XY );
  z_ctrl_val |= setMicrostep_Mask( MICROSTEPS_Z );

  // Calculate the value of IS_GAIN to set in CTRL register, based on I_FS and R_SENSE
  uint8_t is_gain_val = calculate_ISGAIN();
  // Mask axis control register values with desired IS_GAIN value
  uint16_t ISGAIN_mask = setISGAIN_Mask( is_gain_val );
  x_ctrl_val |= ISGAIN_mask;
  y_ctrl_val |= ISGAIN_mask;
  dual_ctrl_val |= ISGAIN_mask;
  z_ctrl_val |= ISGAIN_mask;

  // Calculate the TORQUE value and set the mask to be applied to TORQUE register contents
  uint16_t torque_val = TORQUE_DEFAULT;
  torque_val |= setTORQUE_Mask( is_gain_val );

  // setup x axis driver
  csBit = CS_X_BIT;
  delay_ms(20);  // give a little time before SPI starts
  spi_send16bit( setDRV8711( CTRL, x_ctrl_val ), &CS_PORT, csBit );     // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  spi_send16bit( setDRV8711( TORQUE, torque_val ), &CS_PORT, csBit );   // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  spi_send16bit( setDRV8711( OFF, OFF_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( BLANK, BLANK_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DECAY, DECAY_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( STALL, STALL_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DRIVE, DRIVE_VAL ), &CS_PORT, csBit );
  
  // setup y axis drivers
  csBit = CS_Y_BIT;
  delay_ms(20); // give a little time before SPI starts
  spi_send16bit( setDRV8711( CTRL, y_ctrl_val ), &CS_PORT, csBit );     // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  spi_send16bit( setDRV8711( TORQUE, torque_val ), &CS_PORT, csBit );   // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  spi_send16bit( setDRV8711( OFF, OFF_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( BLANK, BLANK_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DECAY, DECAY_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( STALL, STALL_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DRIVE, DRIVE_VAL ), &CS_PORT, csBit );

  csBit = CS_DUAL_BIT;
  delay_ms(20); // give a little time before SPI starts
  spi_send16bit( setDRV8711( CTRL, dual_ctrl_val ), &CS_PORT, csBit );  // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  spi_send16bit( setDRV8711( TORQUE, torque_val ), &CS_PORT, csBit );   // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  spi_send16bit( setDRV8711( OFF, OFF_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( BLANK, BLANK_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DECAY, DECAY_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( STALL, STALL_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DRIVE, DRIVE_VAL ), &CS_PORT, csBit );
  
  // setup z axis driver
  csBit = CS_Z_BIT;
  delay_ms(20); // give a little time before SPI starts
  spi_send16bit( setDRV8711( CTRL, z_ctrl_val ), &CS_PORT, csBit );     // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  spi_send16bit( setDRV8711( TORQUE, torque_val ), &CS_PORT, csBit );   // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  spi_send16bit( setDRV8711( OFF, OFF_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( BLANK, BLANK_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DECAY, DECAY_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( STALL, STALL_VAL ), &CS_PORT, csBit );
  spi_send16bit( setDRV8711( DRIVE, DRIVE_VAL ), &CS_PORT, csBit );
  #endif
  
}

void enableSteppers()
{
  #ifdef BALL_SCREW_TESTING
    for( uint8_t i=0; i<NUM_AXES; i++ )
    {
      spi_send16bit( setDRV8711( CTRL, axis_list[i].axis_ctrl_reg_value | ENABLE_CTRL ), &CS_PORT, csBit_list[i] );
    }
  #else
  spi_send16bit( setDRV8711( CTRL, x_ctrl_val | ENABLE_CTRL ), &CS_PORT, CS_X_BIT );
  spi_send16bit( setDRV8711( CTRL, y_ctrl_val | ENABLE_CTRL ), &CS_PORT, CS_Y_BIT );
  spi_send16bit( setDRV8711( CTRL, dual_ctrl_val | ENABLE_CTRL ), &CS_PORT, CS_DUAL_BIT );
  spi_send16bit( setDRV8711( CTRL, z_ctrl_val | ENABLE_CTRL ), &CS_PORT, CS_Z_BIT );
  #endif
}

void disableSteppers()
{
  #ifdef BALL_SCREW_TESTING
    for( uint8_t i=0; i<NUM_AXES; i++ )
    {
      spi_send16bit( setDRV8711( CTRL, axis_list[i].axis_ctrl_reg_value & DISABLE_CTRL ), &CS_PORT, csBit_list[i] );
    }
  #else
  spi_send16bit( setDRV8711( CTRL, x_ctrl_val & DISABLE_CTRL ), &CS_PORT, CS_X_BIT );
  spi_send16bit( setDRV8711( CTRL, y_ctrl_val & DISABLE_CTRL ), &CS_PORT, CS_Y_BIT );
  spi_send16bit( setDRV8711( CTRL, dual_ctrl_val & DISABLE_CTRL ), &CS_PORT, CS_DUAL_BIT );
  spi_send16bit( setDRV8711( CTRL, z_ctrl_val & DISABLE_CTRL ), &CS_PORT, CS_Z_BIT );
  #endif
}

uint16_t setMicrostep_Mask( uint16_t desiredRate )
{
  uint16_t microstepMask;

  switch( desiredRate )
  {
    case 1: microstepMask = ((0b0000)<<3); break;
    case 2: microstepMask = ((0b0001)<<3); break;
    case 4: microstepMask = ((0b0010)<<3); break;
    case 8: microstepMask = ((0b0011)<<3); break;
    case 16: microstepMask = ((0b0100)<<3); break;
    case 32: microstepMask = ((0b0101)<<3); break;
    case 64: microstepMask = ((0b0110)<<3); break;
    case 128: microstepMask = ((0b0111)<<3); break;
    case 256: microstepMask = ((0b1000)<<3); break;
    default: microstepMask = ((0b1000)<<3); break;    // Set default to highest microstep rate (256) to avoid accidental large-scale movements if microstep incorrectly selected
  }

  return microstepMask;
}

uint8_t calculate_ISGAIN()
{
  uint8_t IS_GAIN_val = 5;  // Default value for safety

  float is_gain_max = 2.75 * ((1<<TORQUE_REG_BIT_WIDTH)-1) / (256 * I_FS * R_SENSE);

  if( is_gain_max > 40 ) { IS_GAIN_val = 40; }
  else if( is_gain_max > 20 ) { IS_GAIN_val = 20; }
  else if( is_gain_max > 10 ) { IS_GAIN_val = 10; }
  else { IS_GAIN_val = 5; }

  return IS_GAIN_val;
}

uint8_t calculate_ISGAIN_variable_I_FS( float I_FS_val )
{
  uint8_t IS_GAIN_val = 5;  // Default value for safety

  float is_gain_max = 2.75 * ((1<<TORQUE_REG_BIT_WIDTH)-1) / (256 * I_FS_val * R_SENSE);

  if( is_gain_max > 40 ) { IS_GAIN_val = 40; }
  else if( is_gain_max > 20 ) { IS_GAIN_val = 20; }
  else if( is_gain_max > 10 ) { IS_GAIN_val = 10; }
  else { IS_GAIN_val = 5; }

  return IS_GAIN_val;
}

uint16_t setISGAIN_Mask( uint8_t gain_val )
{
  uint16_t isgainMask;

  switch( gain_val )
  {
    case 40: isgainMask = (0b11)<<8; break;
    case 20: isgainMask = (0b10)<<8; break;
    case 10: isgainMask = (0b01)<<8; break;
    case 5:  isgainMask = (0b00)<<8; break;
  }

  return isgainMask;
}

uint16_t setTORQUE_Mask( uint8_t gain_val )
{
  float torque = I_FS * 256 * gain_val * R_SENSE / 2.75;

  return (uint16_t)torque & (0xff);
}

uint16_t setTORQUE_Mask_variable_I_FS( uint8_t gain_val, float I_FS_val )
{
  float torque = I_FS_val * 256 * gain_val * R_SENSE / 2.75;

  return (uint16_t)torque & (0xff);
}

uint16_t setDRV8711( uint16_t address, uint16_t Data )
{
  uint16_t register_value = address;
  uint16_t returnVal = register_value | (Data & 0xfff);

  return returnVal;
}

uint16_t readDRV8711( uint16_t address )
{
  uint16_t register_value =  address;
  uint16_t returnVal = register_value | 0x8000;

  return returnVal;
}
