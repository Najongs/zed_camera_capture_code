import json
import numpy as np

import pyzed.sl as sl
from PIL import Image

from fairino import Robot
robot = Robot.RPC('192.168.58.2')

def initialize_camera():
    zed = sl.Camera()
    # RIGHT SN 30779426
    # LEFT SN 38007749
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 15  # FPS 설정
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.set_from_serial_number(30779426)
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.3
    init_params.depth_maximum_distance = 5
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED 카메라를 열 수 없습니다.")
        exit()

    return zed

def capture_and_save(zed, name):
    image = sl.Mat()
    depth = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed_file_name = f"/home/intertek/DGIST/dataset/image/image{name}.jpg"
        depth_file_name = f"/home/intertek/DGIST/dataset/depth/depth{name}.jpg"

        zed.retrieve_image(image, sl.VIEW.LEFT)
        print("Left Image captured!")

        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        print("Depth Image captured!")

        left_image = image.get_data()  
        depth_image = depth.get_data()

        # numpy 배열로 변환 (BGRA -> BGR)
        left_image_np = np.array(left_image)  # 슬라이드 이미지를 numpy 배열로 변환
        left_image_np = left_image_np[:, :, :3]  # 4채널에서 3채널로 자르기 (BGRA -> BGR)
        left_image_np = left_image_np[..., ::-1]  # BGR -> RGB로 변환

        # Pillow를 사용하여 이미지를 저장
        left_image_pil = Image.fromarray(left_image_np)  # BGR 배열을 Pillow 이미지로 변환
        left_image_pil = left_image_pil.convert('RGB')  # RGB로 변환하여 저장
        left_image_pil.save(zed_file_name)
        print("Left 이미지가 저장되었습니다.")

        # Depth 이미지를 저장 (Depth 값에 색상 맵 적용)
        # NaN 또는 inf 값 처리
        depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)  # NaN과 inf 값을 0으로 처리

        # 정규화: Depth 값을 0에서 255 사이로 변환
        depth_min = np.min(depth_image)
        depth_max = np.max(depth_image)
        depth_normalized = (depth_image - depth_min) / (depth_max - depth_min) * 255.0  # 정규화
        depth_normalized = np.clip(depth_normalized, 0, 255).astype(np.uint8)  # 0~255 범위로 클리핑하고 uint8로 변환

        # Depth 이미지에 색상 맵 적용 (간단한 색상 맵을 PIL로 적용)
        depth_colored = np.zeros((depth_normalized.shape[0], depth_normalized.shape[1], 3), dtype=np.uint8)

        # 색상 맵을 적용 (여기서는 깊이에 따라 색을 변화)
        depth_colored[:, :, 0] = depth_normalized  # Blue 채널에 깊이 값을 설정 (BGR 형식)
        depth_colored[:, :, 1] = depth_normalized // 2  # Green 채널에 약간 더 적은 깊이 값
        depth_colored[:, :, 2] = 255 - depth_normalized  # Red 채널에 반대로 깊이 값을 설정

        # Depth 색상 이미지를 Pillow로 저장
        depth_colored_pil = Image.fromarray(depth_colored)
        depth_colored_pil.save(depth_file_name)
        print("Depth 이미지가 저장되었습니다.")

        # print(f"Image saved as {zed_file_name}, {depth_file_name}")
    else:
        print("Failed to capture image.")

def GetActualJointPosDegree(flag=1):
    flag = int(flag)
    _error = robot.GetActualJointPosDegree(flag)
    error = _error[0]
    if error == 0:
        e_values = _error[1]
        return [e_values[0], e_values[1], e_values[2], e_values[3], e_values[4], e_values[5]]
    else:
        return error

def main():
    zed = initialize_camera()
    for i in range(1000):
        joint = GetActualJointPosDegree()
        output_file = f"/home/intertek/DGIST/dataset/angle/joint{i}.json"
        with open(output_file, "w") as file:
            json.dump(joint, file, indent=4)
        capture_and_save(zed, i)
    zed.close()

if __name__ == "__main__":
    main()
    