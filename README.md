# mikRobot

Erweiterung für den Roboter mik:robot von S. Kummer / Extension for mik:robot by S. Kummer
Beispiele und Dokumentationen unter www.mikrobot.at / For code examples and documentation see www.mikrobot.at

Author: S.Kummer  
Date:   2021.Feb

## Basic usage
```
input.onButtonPressed(Button.A, function () {
    mikRobot.GyroReset()
    omega = 0
    speed = 40
})
input.onButtonPressed(Button.B, function () {
    speed = 0
})
let angle = 0
let speed = 0
let omega = 0
basic.forever(function () {
    angle = omega * (10 / 32767)
    mikRobot.MotorRun(Motors.Left, speed + angle)
    mikRobot.MotorRun(Motors.Right, speed - angle)
    basic.pause(500)
})
basic.forever(function () {
    omega = mikRobot.Gyro() + omega
    basic.pause(10)
})
```

## License

MIT

## Supported targets

* for PXT/micro:bit

```package
mikRobot=github.com/KS-Bulme/pxt-mikRobot
```
