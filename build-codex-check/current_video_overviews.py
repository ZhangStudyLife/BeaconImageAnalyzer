from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont


OUT_DIR = Path(
    r"C:/Users/lx/.codex/visualizations/2026/07/14/019f61e5-48cb-7330-9603-f69cb6d6d7ab"
)

VIDEOS = [
    ("jul08", Path(r"E:/Desktop/前后摄45度结构/2026_07_08_02_32_37_Video.avi")),
    ("jul11", Path(r"E:/Desktop/前后摄45度结构/2026_07_11_07_58_58_Video.avi")),
    ("ceiling", Path(r"E:/Desktop/前后摄45度结构/前后摄手拿_防天花板灯2026_07_10_19_49_30_Video.avi")),
    ("front-car", Path(r"E:/Desktop/前后摄45度结构/前摄_车灯.avi")),
    ("front", Path(r"E:/Desktop/前后摄45度结构/前摄像头.avi")),
    ("front-flight", Path(r"E:/Desktop/前后摄45度结构/前摄像头_实际飞.avi")),
    ("windows", Path(r"E:/Desktop/前后摄45度结构/早上6点半，两个窗户都打开的前摄视频2026_07_12_06_12_57_Video.avi")),
    ("sunlight", Path(r"E:/Desktop/前后摄45度结构/有阳光开窗户2026_07_14_06_34_57_Video.avi")),
]


def font(size: int):
    path = Path(r"C:/Windows/Fonts/arial.ttf")
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default()


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for label, path in VIDEOS:
        capture = cv2.VideoCapture(str(path))
        if not capture.isOpened():
            print(f"open failed: {path}")
            continue
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        indices = [round(index * (count - 1) / 19) for index in range(20)]
        cells = []
        for frame_index in indices:
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame = capture.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            image = Image.fromarray(gray).resize((376, 240), Image.Resampling.NEAREST).convert("RGB")
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 170, 21), fill=(16, 16, 16))
            draw.text((5, 3), f"F{frame_index} mean={gray.mean():.1f}", fill=(255, 255, 255), font=font(13))
            cells.append(image)
        capture.release()

        sheet = Image.new("RGB", (376 * 4, 240 * 5), (25, 25, 25))
        for index, image in enumerate(cells):
            sheet.paste(image, ((index % 4) * 376, (index // 4) * 240))
        output = OUT_DIR / f"current-source-{label}.jpg"
        sheet.save(output, quality=88, optimize=True)
        print(f"{label}|{count}|{output}")


if __name__ == "__main__":
    main()
