# Robot Controller
This package contains the differential drive base controller and hardware interface.

The python script starts a Serial communication with the microcontroller and sends custom commands to set the wheel velocity and direction. In adition, it also reads a feedback message that has the counts of the encoder per wheel.

## Write message
The structure of the message that is needed to set the motor speed is the following:

```Python
msg = ("X"+str(dir_r)+";"+str(round(rpmR))+";" +
               str(dir_l)+";"+str(round(rpmL))+"Y").encode("utf-8")
```

The message starts with an `X` and after that it includes the right wheel direction (clock/couterclockwise), right wheel speed, left wheel direction, left wheel speed. The message ends with a `Y` character.

## Read message
The structure of the feedback message is:
```Python
# 'X'+ dirR ; stepsR ; dirL ; stepsL 'Y'
feedback = msg.split(";")
```
So it can be obtained spleeting the string and removeing the  starting (`X`) and ending (`Y`) characters.