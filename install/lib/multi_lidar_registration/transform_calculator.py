#!/usr/bin/env python3

import numpy as np
import os
import ament_index_python.packages

def estimate_rt(src_points, dst_points):
    """
    src_points: 소스 군집의 포인트 쌍 (N, 3)
    dst_points: 대상 군집의 대응 포인트 쌍 (N, 3)
    """
    assert src_points.shape == dst_points.shape, "점 개수가 일치하지 않습니다."
    
    N = src_points.shape[0]
    
    # 평균 벡터 계산
    src_centroid = np.mean(src_points, axis=0)
    dst_centroid = np.mean(dst_points, axis=0)
    
    # 평균 벡터를 빼서 중심화
    src_points_centered = src_points - src_centroid
    dst_points_centered = dst_points - dst_centroid
    
    # 공분산 행렬 계산
    H = np.transpose(dst_points_centered) @ src_points_centered
    
    # SVD 분해
    U, S, Vt = np.linalg.svd(H)
    
    # 회전 행렬 계산
    R = U @ Vt
    
    # 평행 이동 벡터 계산
    t = dst_centroid - (R @ src_centroid)
    
    transform = np.eye(4)
    
    # R을 3x3 부분에 할당
    transform[:3, :3] = R
    
    # t를 3x1 부분에 할당
    transform[:3, 3] = t.flatten()
    
    return transform

def main():
    # 패키지 경로를 동적으로 가져옴
    package_share_directory = ament_index_python.packages.get_package_share_directory("multi_lidar_registration")

    # 파일 경로 동적 설정
    target_file = os.path.join(package_share_directory, "keypoints/source_keypoint.txt")
    reference_file = os.path.join(package_share_directory, "keypoints/target_keypoint.txt")

    with open(target_file, "r") as file:
        indata = file.read().split()
    indata = [float(x) for x in indata]

    # 12x3 행렬 생성
    points1 = np.array(indata).reshape(12, 3)
    
    with open(reference_file, "r") as file:
        outdata = file.read().split()
    outdata = [float(x) for x in outdata]

    # 12x3 행렬 생성
    points2 = np.array(outdata).reshape(12, 3)

    transform = estimate_rt(points1, points2)
    print("Rotation matrix:\n", transform)

    # 결과를 파일로 저장
    output_file = os.path.join(package_share_directory, "keypoints/RT.txt")
    with open(output_file, "w") as f:
        np.savetxt(f, transform, fmt='%.6f')

if __name__ == "__main__":
    main()
