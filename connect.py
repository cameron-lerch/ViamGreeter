import asyncio
import os

from dotenv import load_dotenv

from viam.robot.client import RobotClient

load_dotenv()

async def connect() -> RobotClient:
    opts = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"],
        api_key_id=os.environ["VIAM_API_KEY_ID"],
    )
    return await RobotClient.at_address(os.environ["VIAM_MACHINE_ADDRESS"], opts)

async def main():
    ''' Connection check: list every resource on the machine, grouped by type. '''
    async with await connect() as machine:
        print(f"Connected to {os.environ['VIAM_MACHINE_ADDRESS']}")
        for rn in sorted(machine.resource_names, key=lambda r: (r.subtype, r.name)):
            print(f"  {rn.subtype:<10} {rn.name}")

if __name__ == '__main__':
    asyncio.run(main())
