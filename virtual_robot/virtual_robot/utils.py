import cv2
import numpy as np

def resize(old_size, frame):
    proportion = max(old_size[0] / frame[0], old_size[1] / frame[1])
    return (int(old_size[0] / proportion), int(old_size[1] / proportion))

def modified_image(image, new_height, new_width):
    old_height, old_width = image.shape[:2]

    if old_height >= new_height and old_width >= new_width:
        new_size = resize((old_height, old_width), (new_height, new_width))
        resized_image = cv2.resize(image, (new_size[1], new_size[0]), interpolation=cv2.INTER_AREA)
    else:
        # Если изображение меньше, то просто растягиваем его
        resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

    new_image = np.zeros((new_height, new_width, 3), dtype="uint8")
    start_y = (new_height - resized_image.shape[0]) // 2
    start_x = (new_width - resized_image.shape[1]) // 2
    new_image[start_y:start_y + resized_image.shape[0], start_x:start_x + resized_image.shape[1]] = resized_image

    return new_image
