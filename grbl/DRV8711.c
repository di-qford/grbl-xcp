#include <avr/io.h>
#include "spi.h"
#include "grbl.h"

uint16_t setMicrostep_Mask( uint16_t desiredRate );
uint8_t calculate_ISGAIN();
uint16_t setISGAIN_Mask( uint8_t gain_val );
uint16_t setTORQUE_Mask( uint8_t gain_val );

void setDRV8711( uint8_t axis_cs, uint16_t register_address, uint16_t Data );
uint16_t readDRV8711( uint8_t axis_cs, uint16_t register_address );

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
#define CLEAR_FAULTS    0x00              // Writes zeroes to the status register, clearing any pending faults on startup

#ifdef X_CARVE_PRO
  #define NUM_DRIVERS 4
#else
  /*  Uncomment the line corresponding to the 
      number of unique, independent stepper motor drive
      channels that exist on your set of hardware.
  */
  // #define NUM_DRIVERS 3
  #define NUM_DRIVERS 4
#endif

// static uint8_t fault_pin_array[] = {FAULT_X_BIT, FAULT_Y_BIT, FAULT_DUAL_BIT, FAULT_Z_BIT};
static uint8_t cs_pin_array[] = {CS_X_BIT, CS_Y_BIT, CS_DUAL_BIT, CS_Z_BIT};

uint16_t x_ctrl_val = CTRL_DEFAULT;
uint16_t y_ctrl_val = CTRL_DEFAULT;
uint16_t dual_ctrl_val = CTRL_DEFAULT;
uint16_t z_ctrl_val = CTRL_DEFAULT;


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

  // Initialize fault pins for pin change interrupts
  // Set the specific interrupts to be enabled due to fault pins in the corresponding mask register
  FAULT_PCMASK_REG |= FAULT_PCMASK;
  // Enable pin change interrupts to process
  FAULT_INT_REG |= (1<<FAULT_INT);

  spi_init();  // initialize the SPI port


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
  setDRV8711( csBit, CTRL, x_ctrl_val );      // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  setDRV8711( csBit, TORQUE, torque_val );    // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  setDRV8711( csBit, OFF, OFF_VAL );
  setDRV8711( csBit, BLANK, BLANK_VAL );
  setDRV8711( csBit, DECAY, DECAY_VAL );
  setDRV8711( csBit, STALL, STALL_VAL );
  setDRV8711( csBit, DRIVE, DRIVE_VAL );
  setDRV8711( csBit, STATUS, CLEAR_FAULTS );
  
  // setup y axis drivers
  csBit = CS_Y_BIT;
  delay_ms(20); // give a little time before SPI starts
  setDRV8711( csBit, CTRL, y_ctrl_val );      // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  setDRV8711( csBit, TORQUE, torque_val );    // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  setDRV8711( csBit, OFF, OFF_VAL );
  setDRV8711( csBit, BLANK, BLANK_VAL );
  setDRV8711( csBit, DECAY, DECAY_VAL );
  setDRV8711( csBit, STALL, STALL_VAL );
  setDRV8711( csBit, DRIVE, DRIVE_VAL );
  setDRV8711( csBit, STATUS, CLEAR_FAULTS );

  csBit = CS_DUAL_BIT;
  delay_ms(20); // give a little time before SPI starts
  setDRV8711( csBit, CTRL, dual_ctrl_val );      // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  setDRV8711( csBit, TORQUE, torque_val );    // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  setDRV8711( csBit, OFF, OFF_VAL );
  setDRV8711( csBit, BLANK, BLANK_VAL );
  setDRV8711( csBit, DECAY, DECAY_VAL );
  setDRV8711( csBit, STALL, STALL_VAL );
  setDRV8711( csBit, DRIVE, DRIVE_VAL );
  setDRV8711( csBit, STATUS, CLEAR_FAULTS );
  
  // setup z axis driver
  csBit = CS_Z_BIT;
  delay_ms(20); // give a little time before SPI starts
  setDRV8711( csBit, CTRL, z_ctrl_val );      // Bits 9:8 are calculated in setISGAIN_Mask, bits 6:3 are calculated in setMicrostep_Mask, and masked with CTRL_DEFAULT
  setDRV8711( csBit, TORQUE, torque_val );    // Bits 8:0 are calculated in setTORQUE_Mask, and masked with TORQUE_VAL_DEFAULT
  setDRV8711( csBit, OFF, OFF_VAL );
  setDRV8711( csBit, BLANK, BLANK_VAL );
  setDRV8711( csBit, DECAY, DECAY_VAL );
  setDRV8711( csBit, STALL, STALL_VAL );
  setDRV8711( csBit, DRIVE, DRIVE_VAL );
  setDRV8711( csBit, STATUS, CLEAR_FAULTS );
  
}

void enableSteppers()
{
  setDRV8711( CS_X_BIT, CTRL, x_ctrl_val | ENABLE_CTRL );
  setDRV8711( CS_Y_BIT, CTRL, y_ctrl_val | ENABLE_CTRL );
  setDRV8711( CS_DUAL_BIT, CTRL, dual_ctrl_val | ENABLE_CTRL );
  setDRV8711( CS_Z_BIT, CTRL, z_ctrl_val | ENABLE_CTRL );
}

void disableSteppers()
{
  setDRV8711( CS_X_BIT, CTRL, x_ctrl_val & DISABLE_CTRL );
  setDRV8711( CS_Y_BIT, CTRL, y_ctrl_val & DISABLE_CTRL );
  setDRV8711( CS_DUAL_BIT, CTRL, dual_ctrl_val & DISABLE_CTRL );
  setDRV8711( CS_Z_BIT, CTRL, z_ctrl_val & DISABLE_CTRL );
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

void setDRV8711( uint8_t axis_cs, uint16_t register_address, uint16_t Data )
{
  uint16_t datagram = register_address | (Data & 0xfff);  // construct the data to be sent by the spi_send16bit function

  spi_send16bit( datagram, &CS_PORT, axis_cs );           // write the data to the SPI bus, activating respective chip select line
}

uint16_t readDRV8711( uint8_t axis_cs, uint16_t register_address )
{
  unsigned long returnValue;
  uint16_t datagram = register_address | 0x8000;                // construct the data to be sent by the spi_send16bit function

  returnValue = spi_send16bit( datagram, &CS_PORT, axis_cs );   // write the data to the SPI bus, activating the respective chip select line,
                                                                // and store the response back from the selected DRV8711
  
  return (uint16_t)returnValue;
}

ISR(FAULT_INT_vect)
{
  // Read the fault pins
  uint8_t register_value = (FAULT_PIN & FAULT_MASK);

  // If any of the fault pins is low (fault is active low)
  if( register_value < FAULT_MASK )
  {
    uint8_t i;  // Create an iterator
    for(i=0; i<NUM_DRIVERS; i++)  // For all motor driver fault pins
    {
      if( (register_value & (1<<i)) == 0 )  // If the current index's bit value is low
      {
        // The current index value is asserting a fault, so read the status register
        // Read the motor driver's status register
        uint16_t status_reg_contents = readDRV8711_test( cs_pin_array[i], STATUS );
        // Mask only the bits that should contain data from the DRV response [7:0] and cast down to uint8_t
        uint8_t status_fault_values = (uint8_t)(status_reg_contents 0xff);

        // Print a message to the terminal
        // report_motor_driver_fault( fault_pin_array[i], status_fault_values );
      }
    }
  }
}
