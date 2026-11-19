"""Create and couple the RMFS controller and physical robot models."""

from argparse import ArgumentParser

from .model import Controller, RMFS, Robot
from .OrderGenerator import OrderGenerator


def create_model(*, robot_count, order_count=100):
    if robot_count < 1:
        raise ValueError('robot_count must be at least 1')

    model = RMFS()
    controller = Controller(robot_count, order_count)
    robots = tuple(Robot(number) for number in range(1, robot_count + 1))
    generator = OrderGenerator(order_count)
    model.add_components(controller, generator, *robots)
    model.add_couples(((generator, "!orders"), (controller, "?orders")))
    for robot in robots:
        model.add_couples(
            ((controller, f"!command_{robot.number}"), (robot, "?command")),
            ((controller, f"!grant_{robot.number}"), (robot, "?grant")),
            ((robot, "!report"), (controller, "?report")),
        )
    return model


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, required=True)
    parser.add_argument('--orders', type=int, default=100, help='initial orders (default: 100)')
    args = parser.parse_args()
    try:
        model = create_model(robot_count=args.robots, order_count=args.orders)
    except ValueError as error:
        parser.error(str(error))
    print(f"{model.name}: {args.robots} robots, 1 controller, 1 order generator")


if __name__ == "__main__":
    main()
