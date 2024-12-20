import os
import re
import argparse
import matplotlib.pyplot as plt

def extract_last_six_digits(file_name):
    """
    ファイル名から末尾6桁を抽出する関数
    """
    match = re.search(r'_(\d{6})\.(jpg|png)$', file_name)
    return int(match.group(1)) if match else None

def correct_difference(diff, range_limit=1000000):
    """
    差分を6桁の範囲を考慮して補正する関数
    """
    corrected_diff = (diff + range_limit) % range_limit
    if corrected_diff > range_limit // 2:
        corrected_diff -= range_limit
    return corrected_diff

def main(directory, threshold):
    """
    指定されたディレクトリ内の画像ファイルの末尾6桁を抽出し、差分を計算してプロットする
    しきい値を超えた変化量があれば表示する
    """
    if not os.path.isdir(directory):
        print(f"Error: The directory '{directory}' does not exist.")
        return

    # ディレクトリ内のファイルを取得し、jpgおよびpngファイルのみ抽出
    file_names = [file for file in os.listdir(directory) if file.endswith(('.jpg', '.png'))]

    # ファイル名をソート
    file_names.sort()

    # 末尾6桁を抽出
    numbers = [extract_last_six_digits(file) for file in file_names if extract_last_six_digits(file) is not None]

    if len(numbers) < 2:
        print("Error: Not enough valid image files found for processing.")
        return

    # 差分を計算し補正
    differences = [correct_difference(j - i) for i, j in zip(numbers[:-1], numbers[1:])]

    # しきい値を超えた差分をチェックして表示
    for idx, diff in enumerate(differences):
        if abs(diff) > threshold:
            print(f"Threshold exceeded at index {idx}: Difference = {diff}")

    # 差分をプロット
    plt.figure(figsize=(10, 6))
    plt.plot(differences, marker='o', linestyle='-')
    plt.title("Differences Between Consecutive Numbers")
    plt.xlabel("Index")
    plt.ylabel("Difference")
    plt.grid(True)
    plt.savefig('output_plot.png')  # プロットをPNG形式で保存
    print("Plot saved as 'output_plot.png'")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process images and calculate differences of last 6 digits.")
    parser.add_argument("directory", type=str, help="Directory containing image files")
    parser.add_argument("--threshold", type=int, default=100000, help="Threshold for significant differences")
    args = parser.parse_args()

    main(args.directory, args.threshold)
