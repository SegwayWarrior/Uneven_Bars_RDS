The code runs an interrupt at a fixed frequency and switches the light on and off.
We can choose which color light to turn on and off.

To indicate that the main loop is running, a different color LED is blinking at a different rate(here I set it to green color)

When the interrupt fires at 1Hz and 50% duty cycle, the LED goes red.

Things that need to be done:
1. Configure the interrupt to run from a Pin level ie, configure it to a pin (say b4; configure the pin as an input pin) which will be always high (3.3V from the Tiva itself). When we pull the wire and connect it to ground pin of the board, the interrupt should get triggered.
2. in the while loop, have a poll reading for the interrupt and when read, it will do the actions which Bill wanted it to do, mentioned in the assignment.