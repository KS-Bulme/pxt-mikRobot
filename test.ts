input.onButtonPressed(Button.A, function () {
    mikRobot.Servo(Servos.S1, 0)  // PWM test A
    for (let index = 0; index < 4; index++) {
        mikRobot.RunDelay(Dir.forward, 50, 2)  // motor test
        mikRobot.RunDelay(Dir.turnLeft, 50, 0.5)  // turn left
    }
})
input.onButtonPressed(Button.B, function () {
    mikRobot.Servo(Servos.S1, 180)  // PWM test B
})
mikRobot.SensorCalibrated()  // calibrate line sensors
basic.forever(function () {
    led.plotBarGraph(
    mikRobot.Ultrasonic(),  // ultrasonic sensor test
    40
    )
    basic.pause(100)
})
