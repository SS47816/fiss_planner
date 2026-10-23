"""Compose the 1280x640 GitHub social preview from the logo and two demo frames.

pixi run -e carla python examples/make_social_preview.py   # needs cairosvg, imageio
"""

import io
from pathlib import Path

import cairosvg
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont

MEDIA = Path("docs/media")
BG = (20, 23, 28)


def frame_from(path: Path, fraction: float) -> Image.Image:
    im = Image.open(path)
    im.seek(int(im.n_frames * fraction))
    return im.convert("RGB")


def main() -> None:
    card = Image.new("RGB", (1280, 640), BG)
    logo = Image.open(io.BytesIO(cairosvg.svg2png(url=str(MEDIA / "logo.svg"), output_width=760)))
    card.paste(logo, (40, 24), logo)
    d = ImageDraw.Draw(card)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 30)
    except OSError:
        font = ImageFont.load_default()
    d.text(
        (64, 196),
        "FOP · FISS · FISS+  —  C++17 · Python · ROS 2 · CARLA · Lanelet2",
        font=font,
        fill=(201, 209, 217),
    )
    cam = Image.fromarray(imageio.get_reader("results/videos/carla_cut_in_fiss_plus.mp4").get_data(300))
    cam = cam.resize((600, 338), Image.LANCZOS)
    cmp_ = frame_from(MEDIA / "compare_fop_fiss_fissplus.gif", 0.33)
    cmp_ = cmp_.resize((600, int(cmp_.height * 600 / cmp_.width)), Image.LANCZOS)
    card.paste(cam, (40, 262))
    card.paste(cmp_.crop((0, 0, 600, min(338, cmp_.height))), (640, 262 + max(0, (338 - cmp_.height) // 2)))
    card.save(MEDIA / "social_preview.png", optimize=True)


if __name__ == "__main__":
    main()
