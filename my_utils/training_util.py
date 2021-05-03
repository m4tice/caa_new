import cv2
import os

import numpy as np
import matplotlib.image as mpimg

from tensorflow.keras.preprocessing.image import img_to_array
from PIL import Image


canny_threshold1 = 50
canny_threshold2 = 150

IMG_H = 66
IMG_W = 200
IMG_C = 3
INPUT_SHAPE = (IMG_H, IMG_W, IMG_C)


# - @private: Load image
def load_image(data_dir, image_file):
    return mpimg.imread(os.path.join(data_dir, image_file))


# - @private: Load RGB image
def load_carla_image(image, path=None):
    if path is not None:
        image = Image.open(os.path.join(path, image))
    else:
        image = Image.open(image)
    case1 = image.mode == 'RGBA'
    case2 = image.mode == 'CMYK'

    if case1 or case2:
        image = image.convert('RGB')

    image = img_to_array(image)
    image = image.astype(np.uint8)

    return image


# @private: Crop Image
def crop_img(image):
    # return image[60:-25, :, :]  # udacity
    return image[100:, :, :]  # carla


# @private: Resize Image
def resize_img(image):
    return cv2.resize(image, (IMG_W, IMG_H), cv2.INTER_AREA)


# @private: Change RGB to YUV
def rgb2yuv(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2YUV)


# @private: Choose a random image
def random(center, left, right, steering_angle, offsets):
    index = np.random.choice(3)
    if index == 0:
        return left, steering_angle + offsets

    elif index == 1:
        return right, steering_angle - offsets

    else:
        return center, steering_angle


# @private: Flip image randomly
def flip(image, steering_angle):
    if np.random.rand() < 0.5:
        image = cv2.flip(image, 1)
        steering_angle = -steering_angle
    else:
        image = image
        steering_angle = steering_angle

    return image, steering_angle


# @private: Translate image randomly
def translate(image, steering_angle, range_x, range_y):
    trans_x = range_x * (np.random.rand() - 0.5)
    trans_y = range_y * (np.random.rand() - 0.5)
    steering_angle += trans_x * 0.002
    trans_m = np.float32([[1, 0, trans_x], [0, 1, trans_y]])
    height, width = image.shape[:2]
    image = cv2.warpAffine(image, trans_m, (width, height))

    return image, steering_angle


# @private: Create shadow
def shadow(image):
    x1, y1 = IMG_W * np.random.rand(), 0
    x2, y2 = IMG_W * np.random.rand(), IMG_H
    xm, ym = np.mgrid[0:IMG_H, 0:IMG_W]

    mask = np.zeros_like(image[:, :, 1])
    mask[np.where((ym - y1) * (x2 - x1) - (y2 - y1) * (xm - x1) > 0)] = 1

    cond = mask == np.random.randint(2)
    s_ratio = np.random.uniform(low=0.2, high=0.5)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    hls[:, :, 1][cond] = hls[:, :, 1][cond] * s_ratio
    image = cv2.cvtColor(hls, cv2.COLOR_HLS2RGB)

    return image


# @private: Adjust brightness randomly
def brightness(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    ratio = 1.0 + 0.4 * (np.random.rand() - 0.5)
    hsv[:, :, 2] = hsv[:, :, 2] * ratio
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

    return image


# @private: Transform grayscale image from 1 channel to 3 channels
def transform_image(image):
    image = np.stack((image,)*3, axis=-1)

    return image


# @public: Detect edges in image
def lane_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    image = cv2.GaussianBlur(image, (5, 5), 0)
    image = cv2.Canny(image, canny_threshold1, canny_threshold2)
    image = transform_image(image)

    return image


# @public: [NEW]
def preprocess1(image):
    image = crop_img(image)
    image = resize_img(image)
    image = rgb2yuv(image)

    return image


# @public: [OLD] Process normal image
# def preprocess2(center, left, right, steering_angle, angle_offsets=0.2, range_x=100, range_y=10):
#     image, steering_angle = random(center, left, right, steering_angle, angle_offsets)
#     image, steering_angle = flip(image, steering_angle)
#     image, steering_angle = translate(image, steering_angle, range_x, range_y)
#     image = shadow(image)
#     image = brightness(image)
#
#     return image, steering_angle


# - @public: [NEW] Process normal image
def preprocess2(image, path, steering_angle, range_x=100, range_y=10):
    image = load_carla_image(image, path=path)
    # image, steering_angle = flip(image, steering_angle)
    image, steering_angle = translate(image, steering_angle, range_x, range_y)

    return image, steering_angle


def data_generator(feature, label, path, batch_size, is_training, lucky_number=0.6):
    images = np.empty([batch_size, IMG_H, IMG_W, IMG_C])
    angles = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(feature.shape[0]):
            image = feature[index]
            steering = label[index]

            if is_training and np.random.rand() < lucky_number:
                image, steering = preprocess2(image, path, steering)
            else:
                image = load_carla_image(image, path)

            images[i] = preprocess1(image)
            angles[i] = steering

            i += 1
            if i == batch_size:
                break

        yield images, angles
