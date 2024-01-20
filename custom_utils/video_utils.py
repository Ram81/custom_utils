import os
from typing import List, Optional

import imageio
import numpy as np
import tqdm
from PIL import Image
import glob
from tqdm import tqdm


def images_to_video(
    images: List[np.ndarray],
    output_dir: str,
    video_name: str,
    fps: int = 20,
    quality: Optional[float] = 10,
    **kwargs,
):
    r"""Calls imageio to run FFMPEG on a list of images. For more info on
    parameters, see https://imageio.readthedocs.io/en/stable/format_ffmpeg.html
    Args:
        images: The list of images. Images should be HxWx3 in RGB order.
        output_dir: The folder to put the video in.
        video_name: The name for the video.
        fps: Frames per second for the video. Not all values work with FFMPEG,
            use at your own risk.
        quality: Default is 5. Uses variable bit rate. Highest quality is 10,
            lowest is 0.  Set to None to prevent variable bitrate flags to
            FFMPEG so you can manually specify them using output_params
            instead. Specifying a fixed bitrate using ‘bitrate’ disables
            this parameter.
    """
    assert 0 <= quality <= 10
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    video_name = video_name.replace(" ", "_").replace("\n", "_") + ".mp4"
    writer = imageio.get_writer(
        os.path.join(output_dir, video_name),
        fps=fps,
        quality=quality,
        **kwargs,
    )
    for im in tqdm(images):
        writer.append_data(im)
    writer.close()


# Usage example:
def main(path):
    files = glob.glob(path + "/*tp*.png")
    #files = [f for f in files if "tp" not in f]
    files = sorted(files)
    print(files[:10])
    print("Total frames: {}".format(len(files)))


    images = [np.array(Image.open(f)) for f in tqdm(files)]
    print(images[0].shape)
    images_to_video(images, "demos", "put_pear_tp")
