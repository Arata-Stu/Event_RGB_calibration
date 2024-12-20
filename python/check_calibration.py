import cv2
import numpy as np

def undistort_image(image_path, calibration_file, camera="Camera1"):
    # キャリブレーションデータを読み込む
    fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode(f"{camera}Matrix").mat()
    dist_coeffs = fs.getNode(f"DistCoeffs{camera[-1]}").mat()
    fs.release()

    # 入力画像を読み込む
    img = cv2.imread(image_path)
    h, w = img.shape[:2]

    # 歪み補正用のマップを生成
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1)

    # 補正を適用
    undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    # 必要ならROIで切り抜き
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]

    return undistorted_img

if __name__ == "__main__":
    image_path = "path/to/image.jpg"
    calibration_file = "calibration.yaml"
    output_path = "undistorted_image.jpg"

    # Camera1で補正を実施
    corrected_image = undistort_image(image_path, calibration_file, camera="Camera1")

    # 結果を保存
    cv2.imwrite(output_path, corrected_image)
    print(f"補正画像を {output_path} に保存しました。")
