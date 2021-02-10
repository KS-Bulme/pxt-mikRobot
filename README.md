# mikRobot

Erweiterung für den Roboter mik:robot von S. Kummer / Extension for mik:robot by S. Kummer

Beispiele und Dokumentationen unter www.mikrobot.at / For code examples and documentation see www.mikrobot.at

## Beispiel 1: Hindernissen ausweichen / Example 1: avoiding the obstacles
```
basic.forever(function () {
    while (!(mikRobot.Infrared(Sensor.Left) && mikRobot.Infrared(Sensor.Right))) {
        mikRobot.Run(Dir.forward, 80)
    }
    // move backwards with speed 50 for 2 sec...
    mikRobot.RunDelay(Dir.backward, 50, 2)
    // ... then turn left with speed 50 for a random number of time between 0.1 and 1 sec
    mikRobot.RunDelay(Dir.turnLeft, 50, randint(0, 10) / 10)
})
```
## Beispiel 2: Folge der Linie! / Example 2: follow the line!
```
let sensor: number[] = []
basic.forever(function () {
    sensor = mikRobot.AnalogRead()
    if (sensor[2] < 100) {
        // middle sensor detects black line, steer right
        mikRobot.MotorRun(Motors.Left, 50)
        mikRobot.MotorRun(Motors.Right, 20)
    } else {
        // we're right off the line, steer left
        mikRobot.MotorRun(Motors.Left, 20)
        mikRobot.MotorRun(Motors.Right, 50)
    }
})
```

## Beispiel 3: immer geradeaus mit dem Gyro-Sensor / Example 3: steer straight using gyro sensor
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
    // calculate angle dividing by time passed
    angle = omega * (10 / 32767)
    // run left motor faster by value of angle to compensate gyro rotation
    mikRobot.MotorRun(Motors.Left, speed + angle)
    mikRobot.MotorRun(Motors.Right, speed - angle)
    basic.pause(500)
})
basic.forever(function () {
    // adding up angular velocity for numerical integration
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
