
**CAN-L4-3100 Magnetometer Module** 

This firmware makes a Mateksys CAN-L4-3100 useful for a Larus system. 

The Magnetometer can be placed in a magnetically undisturbed place in the glider in a arbitrary but fixed position.

It measures magnetic induction with very high resolution and accuracy.

It outputs its induction measurement at 100 Hz with 3 * 16bit resolution in a single CAN frame. 

The Larus sensor automatically calibrates the induction measurement and uses it for navigation as long as it receives the frames over the Larus CAN network. 

The firmware installation to the magnetometer module can be done with either SWD or USART (STM32 boot protocol).

