import os
import shutil
import argparse
import json
from datetime import datetime

def strip_directory_name(name):
    """
    サブディレクトリ名から末尾8文字を削除する。
    例:
      '20241229_181002_925622' -> '20241229_18100'
    """
    # nameの長さが8より短い場合はそのまま返す
    return name[:-8] if len(name) > 8 else name

def parse_timestamp(name):
    """
    サブディレクトリ名を datetime オブジェクトに変換（UTCマイクロ秒まで）
    例: '20241229_181002_925622'
    """
    try:
        return datetime.strptime(name, '%Y%m%d_%H%M%S_%f')
    except ValueError:
        return None

def parse_timestamp_from_filename(filename):
    """
    新しいフォーマットの画像ファイル名からタイムスタンプを抽出
    例: frame_20241219_205444_232943.jpg -> 2024-12-19 20:54:44.232943
    """
    try:
        base_name = filename.split('.')[0]  # 拡張子を除去
        # "frame_YYYYMMDD_HHMMSS_microseconds" を想定
        # 例: frame_20241219_205444_232943
        _, date, time, microseconds = base_name.split('_')
        full_datetime = f"{date}_{time}_{microseconds}"
        timestamp = datetime.strptime(full_datetime, "%Y%m%d_%H%M%S_%f")
        return timestamp
    except (ValueError, IndexError):
        return None

def calculate_offsets(image_dir, start_time):
    """
    画像ファイルのタイムスタンプから、指定した開始時間を引いてオフセットを計算。
    オフセットはマイクロ秒単位でテキストに出力する。
    """
    offsets = []
    for filename in sorted(os.listdir(image_dir)):
        # JPGファイルだけを対象
        if filename.lower().endswith('.jpg'):
            image_timestamp = parse_timestamp_from_filename(filename)
            if image_timestamp and start_time:
                offset = (image_timestamp - start_time).total_seconds() * 1_000_000  # マイクロ秒単位
                offsets.append(str(int(offset)))  # オフセット値（整数）のみをリストへ追加
    return offsets

def find_and_process_matching_directories(base_directory, output_directory):
    events_dir = os.path.join(base_directory, 'events')
    images_dir = os.path.join(base_directory, 'images')
    
    if not os.path.exists(events_dir) or not os.path.exists(images_dir):
        print("指定されたディレクトリに 'events' または 'images' が存在しません。")
        return
    
    # 比較用キーを生成
    #   ex: '20241229_181002_925622' -> '20241229_18100'
    events_subdirs = {strip_directory_name(name): name for name in os.listdir(events_dir)}
    images_subdirs = {strip_directory_name(name): name for name in os.listdir(images_dir)}
    
    # 両者でキーが一致するディレクトリを処理
    for stripped_name in events_subdirs.keys() & images_subdirs.keys():
        events_full_name = events_subdirs[stripped_name]
        images_full_name = images_subdirs[stripped_name]
        
        events_path = os.path.join(events_dir, events_full_name)
        images_path = os.path.join(images_dir, images_full_name)
        
        # タイムスタンプを解析（UTCマイクロ秒まで）
        # ※events側はイベント開始時刻だが、今回のオフセットはimages側の時刻を基準とする
        event_start_time = parse_timestamp(events_full_name)
        rgb_image_start_time = parse_timestamp(images_full_name)  # 画像ディレクトリ名から記録開始時刻を取得
        
        # 出力先ディレクトリを作成
        target_dir = os.path.join(output_directory, stripped_name)
        events_target = os.path.join(target_dir, 'events')
        images_target = os.path.join(target_dir, 'images')
        
        os.makedirs(events_target, exist_ok=True)
        os.makedirs(images_target, exist_ok=True)
        
        # ファイルを移動
        print(f"移動中: {events_path}/* -> {events_target}")
        for file_name in os.listdir(events_path):
            shutil.move(os.path.join(events_path, file_name), events_target)
        
        print(f"移動中: {images_path}/* -> {images_target}")
        for file_name in os.listdir(images_path):
            shutil.move(os.path.join(images_path, file_name), images_target)
        
        # オフセット計算（images ディレクトリの時刻を基準に）
        offsets = calculate_offsets(images_target, rgb_image_start_time)
        
        # オフセット情報を保存
        offsets_file = os.path.join(target_dir, "image_offsets.txt")
        with open(offsets_file, "w", encoding="utf-8") as file:
            for offset in offsets:
                file.write(offset + "\n")
        print(f"オフセット情報を {offsets_file} に保存しました。")
        
        # メタ情報を保存（イベント開始時刻なども念のため含める）
        metadata = {
            "events_path": events_target,
            "images_path": images_target,
            "event_start_time": event_start_time.isoformat() + "Z" if event_start_time else None,
            "rgb_image_start_time": rgb_image_start_time.isoformat() + "Z" if rgb_image_start_time else None,
            "offsets_file": offsets_file
        }
        metadata_file = os.path.join(target_dir, "metadata.json")
        with open(metadata_file, "w", encoding="utf-8") as meta_file:
            json.dump(metadata, meta_file, indent=4, ensure_ascii=False)
        print(f"メタ情報を {metadata_file} に保存しました。")
        
        # 空になった元のディレクトリを削除
        os.rmdir(events_path)
        os.rmdir(images_path)

def main():
    parser = argparse.ArgumentParser(
        description="Find, move, and process matching directory pairs in 'events' and 'images'."
    )
    parser.add_argument(
        "-i", "--input_directory", 
        required=True, 
        help="Base directory containing 'events' and 'images'."
    )
    parser.add_argument(
        "-o", "--output_directory", 
        required=True, 
        help="Output directory to move and process matching pairs."
    )
    
    args = parser.parse_args()
    base_directory = args.input_directory
    output_directory = args.output_directory
    
    find_and_process_matching_directories(base_directory, output_directory)

if __name__ == "__main__":
    main()
