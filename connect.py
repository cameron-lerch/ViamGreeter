import asyncio
import os

from dotenv import load_dotenv

from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.components.camera import Camera
from viam.services.mlmodel import MLModelClient
from viam.services.vision import VisionClient
from viam.services.generic import Generic as GenericService

load_dotenv()

async def connect() -> RobotClient:
    opts = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"],
        api_key_id=os.environ["VIAM_API_KEY_ID"],
    )
    return await RobotClient.at_address(os.environ["VIAM_MACHINE_ADDRESS"], opts)

async def main():
    async with await connect() as machine:
        print('Resources:')
        print(machine.resource_names)
        
        # arm (name from GREETER_ARM in .env)
        arm_name = os.environ.get("GREETER_ARM", "arm-1")
        arm: Arm = Arm.from_robot(machine, arm_name)
        arm_return_value = await arm.get_end_position()
        print(f"{arm_name} get_end_position return value: {arm_return_value}")

        # gripper
        gripper: Gripper = Gripper.from_robot(machine, "gripper-SO101-real")
        gripper_return_value = await gripper.is_moving()
        print(f"gripper-SO101-real is_moving return value: {gripper_return_value}")

        # cam
        cam: Camera = Camera.from_robot(machine, "cam")
        cam_return_value = await cam.get_images()
        print(f"cam get_images return value: {cam_return_value}")

        # people_model
        people_model: MLModelClient = MLModelClient.from_robot(machine, "people_model")
        people_model_return_value = await people_model.metadata()
        print(f"people_model metadata return value: {people_model_return_value}")

        # people_detector
        people_detector: VisionClient = VisionClient.from_robot(machine, "people_detector")
        people_detector_return_value = await people_detector.get_properties()
        print(f"people_detector get_properties return value: {people_detector_return_value}")

if __name__ == '__main__':
    asyncio.run(main())
