import os
import h5py
import numpy as np
import matplotlib.pyplot as plt

try:
    import hdf5plugin
except ImportError:
    pass

def read_h5_file(file_path):
    """
    h5ファイルからデータを読み込む関数
    CD/eventsのデータを読み込む
    """
    assert file_path.endswith(('.h5', '.hdf5')), "Error: The file must be in HDF5 format."
    assert os.path.exists(file_path), f"Error: The file '{file_path}' does not exist."
    with h5py.File(file_path, 'r') as f:
        # CD/eventsのデータを読み込む
        events = f['CD']['events'][:]
        return events   
    
def main(file_path):
    """
    指定されたh5ファイルからCD/eventsのデータを読み込む
    イベントはμ秒でタイムスタンプされている
    duration を指定し、イベントをduration_msミリ秒ごとにグループ化し、plotする
    """
    events = read_h5_file(file_path)
    duration_ms = 30  # ミリ秒
    duration = duration_ms * 1000  # マイクロ秒

    # データの準備
    timestamps = events[:, 0]
    polarities = events[:, 2]
    
    # タイムスタンプをソート
    sorted_indices = np.argsort(timestamps)
    timestamps = timestamps[sorted_indices]
    polarities = polarities[sorted_indices]
    
    max_timestamp = timestamps[-1]
    num_groups = max_timestamp // duration + 1
    group_indices = np.searchsorted(timestamps, np.arange(num_groups) * duration, side='right')
    group_indices = np.append(group_indices, len(timestamps))
    group_sizes = np.diff(group_indices)
    group_polarities = np.split(polarities, group_indices[1:-1])
    
    # プロット
    plt.figure(figsize=(10, 6))
    for idx, (size, polarities) in enumerate(zip(group_sizes, group_polarities)):
        if size > 0:  # 空のグループをスキップ
            x = np.full(size, idx)
            plt.scatter(x, polarities, s=1, c='black')
    plt.title("Events Grouped by Time")
    plt.xlabel("Time Group Index")
    plt.ylabel("Polarity")
    plt.grid(True)
    plt.savefig('output_plot.png')  # プロットをPNG形式で保存
    print("Plot saved as 'output_plot.png'")

# テスト実行
main('/Users/at/dataset/calibration/sample_record/events/20241222_044954_157158.hdf5')
