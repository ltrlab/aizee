from feetech import FeetechMotorsBus


def main():

    motors = {
        # name: (index, model)
        "shoulder_pan": [1, "sts3215"],
        "shoulder_lift": [2, "sts3215"],
        "elbow_flex": [3, "sts3215"],
        "wrist_flex": [4, "sts3215"],
        "wrist_roll": [5, "sts3215"],
        "gripper": [6, "sts3215"],
    }

    motors_bus = FeetechMotorsBus(
        port="/dev/tty.usbmodem58CD1766831",
        motors=motors,
    )
    motors_bus.connect()

    position = motors_bus.read("Present_Position")
    print(position)

    # move from a few motor steps as an example
    few_steps = +100
    motors_bus.write(
        "Goal_Position", [position[0] + few_steps], motor_names=["shoulder_pan"]
    )


if __name__ == "__main__":
    main()
