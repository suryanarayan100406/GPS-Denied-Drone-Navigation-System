from controller import Robot


def main() -> None:
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motor = robot.getDevice('slider_motor')
    sensor = robot.getDevice('slider_sensor')

    sensor.enable(timestep)
    motor.setVelocity(0.45)

    target = 0.8
    motor.setPosition(target)

    while robot.step(timestep) != -1:
        pos = sensor.getValue()
        if pos >= 0.78:
            target = -0.8
            motor.setPosition(target)
        elif pos <= -0.78:
            target = 0.8
            motor.setPosition(target)


if __name__ == '__main__':
    main()
