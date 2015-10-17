MainBoard Firmware

##MainBoard Xcode command:
```
X1 + O + num (0 ~ 2)                                                           : set laser on (0: all , 1:laser1 , 2:laser2)
X1 + F + num (0 ~ 2)                                                           : set laser off (0: all , 1:laser1 , 2:laser2)
X2 + O + num (0 ~ 255)                                                         : set IO2 PWM duty on
X2 + F                                                                         : set IO2 PWM duty off
X3 + O + num (0 ~ 4)                                                           : set REFC on (0: all , 1:REFC1 , 2:REFC2 , 3:REFC3 , 4:REFC4)
X3 + F + num (0 ~ 4)                                                           : set REFC off (0: all , 1:REFC1 , 2:REFC2 , 3:REFC3 , 4:REFC4)
```

##### X4 Manage LED

* Syntax: `X4L[id]C[mode]A[cycle]`

* `[id]`: `0`=White, `1`=Red, `2`=Green (Required)
* `[mode]`: `0`=Off, `1`=Wave, `2`=Blink, `3`=On
* `[cycle]`: Wave/Blink mode cycle, use `0.0005` first if you don't know which value is better

* Example:

> `X3L0C1A0.005`: White LED wave with cycle 0.005
> `X3L1C3`: Red LED on

```
X6                                                                             : delta Homing Distance Calculation
X7 + R + num (1 ~ 6) + C + num (step)                                          : motor control 
X8 + O,F                                                                       : filament detect 
X111                                                                           : read FW version
```


