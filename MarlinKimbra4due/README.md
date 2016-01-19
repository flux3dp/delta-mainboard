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
X6                                                                             : delta Homing Distance Calculation
X7 + R + num (1 ~ 6) + C + num (step)                                          : motor control 
X8 + O,F                                                                       : filament detect 
X111                                                                           : read FW version
```


