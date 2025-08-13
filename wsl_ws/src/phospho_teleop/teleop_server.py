#!/usr/bin/env python3
"""Simple entrypoint launching the Phosphobot teleoperation manager."""
import asyncio

from phospho_teleop.teleoperation import TeleopManager
from phospho_teleop.robot import RobotConnectionManager


async def _async_main() -> None:
    rcm = RobotConnectionManager()
    manager = TeleopManager(rcm)
    await manager.move_init()
    while True:
        await asyncio.sleep(1)


def main() -> None:
    asyncio.run(_async_main())


if __name__ == "__main__":
    main()
