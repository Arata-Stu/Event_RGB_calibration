import cv2
import numpy as np
import glob
import argparse
from tqdm import tqdm

def save_calibration_file(output_file, mtx1, dist1, mtx2, dist2, R, T, E, F):
    try:
        # OpenCVのFileStorageを使用してYAML形式で保存
        fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
        fs.write("CameraMatrix1", mtx1)
        fs.write("DistCoeffs1", dist1)
        fs.write("CameraMatrix2", mtx2)
        fs.write("DistCoeffs2", dist2)
        fs.write("RotationMatrix", R)
        fs.write("TranslationVector", T)
        fs.write("EssentialMatrix", E)
        fs.write("FundamentalMatrix", F)
        fs.release()
        print(f"\nキャリブレーション結果を {output_file} に保存しました。")
    except Exception as e:
        print(f"キャリブレーション結果の保存中にエラーが発生しました: {e}")


def stereo_calibration(camera1_dir, camera2_dir, square_size, checkerboard_size, output_file):
    try:
        # 3D点の準備
        objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        objp *= square_size

        # 各カメラの画像リスト
        images_camera1 = sorted([f for f in glob.glob(f'{camera1_dir}/*') if f.lower().endswith(('.jpg', '.png'))])
        images_camera2 = sorted([f for f in glob.glob(f'{camera2_dir}/*') if f.lower().endswith(('.jpg', '.png'))])

        # 必要な変数
        objpoints = []  # 3D点
        imgpoints1 = []  # camera1の2D点
        imgpoints2 = []  # camera2の2D点
        failed_images = []  # 特徴抽出に失敗した画像リスト

        # 各カメラのコーナー検出
        for img1_path, img2_path in tqdm(zip(images_camera1, images_camera2), total=len(images_camera1), desc="Processing images"):
            try:
                img1 = cv2.imread(img1_path)
                img2 = cv2.imread(img2_path)
                gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

                # コーナーを検出
                ret1, corners1 = cv2.findChessboardCorners(gray1, checkerboard_size)
                ret2, corners2 = cv2.findChessboardCorners(gray2, checkerboard_size)

                if ret1 and ret2:
                    objpoints.append(objp)
                    imgpoints1.append(corners1)
                    imgpoints2.append(corners2)
                else:
                    failed_images.append((img1_path, img2_path))
            except Exception as e:
                print(f"画像処理中にエラーが発生しました: {e}")
                failed_images.append((img1_path, img2_path))

        # 失敗した画像を表示
        if failed_images:
            print("\n以下の画像で特徴抽出が失敗しました:")
            for img1_path, img2_path in failed_images:
                print(f"  Camera1: {img1_path}, Camera2: {img2_path}")
            print(f"\n特徴抽出に失敗した画像の数: {len(failed_images)}")
        else:
            print("\nすべての画像で特徴抽出に成功しました。")

        # キャリブレーション
        if objpoints:
            ret1, mtx1, dist1, _, _ = cv2.calibrateCamera(objpoints, imgpoints1, gray1.shape[::-1], None, None)
            ret2, mtx2, dist2, _, _ = cv2.calibrateCamera(objpoints, imgpoints2, gray2.shape[::-1], None, None)

            # ステレオキャリブレーション
            flags = cv2.CALIB_FIX_INTRINSIC
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

            _, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
                objpoints, imgpoints1, imgpoints2,
                mtx1, dist1, mtx2, dist2, gray1.shape[::-1],
                criteria=criteria, flags=flags
            )

            # キャリブレーション結果を保存
            save_calibration_file(output_file, mtx1, dist1, mtx2, dist2, R, T, E, F)

            # 結果出力
            print("回転行列 (R):\n", R)
            print("並進ベクトル (T):\n", T)
            print("本質行列 (E):\n", E)
            print("基本行列 (F):\n", F)
            return R, T, E, F
        else:
            print("\nキャリブレーションに必要な十分なデータがありませんでした。")
            return None, None, None, None

    except Exception as e:
        print(f"ステレオキャリブレーション中にエラーが発生しました: {e}")
        return None, None, None, None


if __name__ == "__main__":
    # 引数解析
    parser = argparse.ArgumentParser(description="Stereo Camera Calibration")
    parser.add_argument("--camera1", type=str, required=True, help="Camera 1 image directory")
    parser.add_argument("--camera2", type=str, required=True, help="Camera 2 image directory")
    parser.add_argument("--size", type=float, required=True, help="Size of one square in the checkerboard (in mm or cm)")
    parser.add_argument("--cols", type=int, required=True, help="Number of columns in the checkerboard")
    parser.add_argument("--rows", type=int, required=True, help="Number of rows in the checkerboard")
    parser.add_argument("--output", type=str, required=True, help="Output file for calibration results (e.g., calibration.yaml)")

    args = parser.parse_args()

    # チェッカーボードサイズを指定
    checkerboard_size = (args.cols, args.rows)

    # 実行
    stereo_calibration(args.camera1, args.camera2, args.size, checkerboard_size, args.output)
