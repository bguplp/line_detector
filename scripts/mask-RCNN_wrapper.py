#!/usr/bin/env python3

import rospy
import json
import numpy as np
from numpy import ndarray
import skimage.io
from MaskRcnnDetector import MaskrcnnObjectDetector as Detector
from pathlib import Path
import colorsys
import random
from skimage.measure import find_contours
from matplotlib.patches import Polygon
from matplotlib import patches,  lines
from sensor_msgs.msg import CompressedImage
import cv2
from sensor_msgs.msg import PointCloud2
import pandas as pd

global verbose, detector, class_names, pub_res
pointcloud_topic = '/kinect2/qhd/points'

class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
            'bus', 'train', 'truck', 'boat', 'traffic light',
            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
            'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
            'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
            'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
            'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
            'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
            'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
            'teddy bear', 'hair drier', 'toothbrush']

def pcl2_callback(msg):
    try:
        detect(msg, class_names)
    except:
        pass

rospy.init_node('mask_RCNN_wrapper', anonymous=True)
pub_res = rospy.Publisher('/mask_RCNN/results/compressed', CompressedImage, queue_size=10)
pc2_msg = rospy.wait_for_message(pointcloud_topic, PointCloud2)
rospy.Subscriber(pointcloud_topic, PointCloud2, pcl2_callback)
rospy.loginfo("start publish mask-RCNN results")

path_to_maskrcnn = Path(__file__).parent.parent.parent.joinpath("depthCamera/Mask_RCNN-master")
verbose = False
detector = Detector(str(path_to_maskrcnn),
                    str(path_to_maskrcnn.joinpath('samples/coco')),
                    str(path_to_maskrcnn.joinpath('samples/mask_rcnn_coco.h5')),
                    str(path_to_maskrcnn.joinpath('logs'))
                        )

def apply_mask(image, mask, color, alpha=0.5):
    """Apply the given mask to the image.
    """
    for c in range(3):
        image[:, :, c] = np.where(mask == 1,
                                  image[:, :, c] *
                                  (1 - alpha) + alpha * color[c] * 255,
                                  image[:, :, c])
    return image

def random_colors(N, bright=True):
    """
    Generate random colors.
    To get visually distinct colors, generate them in HSV space then
    convert to RGB.
    """
    brightness = 1.0 if bright else 0.7
    hsv = [(i / N, 1, brightness) for i in range(N)]
    colors = list(map(lambda c: colorsys.hsv_to_rgb(*c), hsv))
    random.shuffle(colors)
    return colors

def display_instances(image, boxes, masks, class_ids, class_names,
                      scores=None, title="",
                      figsize=(16, 16), ax=None,
                      show_mask=True, show_bbox=True,
                      colors=None, captions=None):
    global pub_res
    """
    boxes: [num_instance, (y1, x1, y2, x2, class_id)] in image coordinates.
    masks: [height, width, num_instances]
    class_ids: [num_instances]
    class_names: list of class names of the dataset
    scores: (optional) confidence scores for each box
    title: (optional) Figure title
    show_mask, show_bbox: To show masks and bounding boxes or not
    figsize: (optional) the size of the image
    colors: (optional) An array or colors to use with each object
    captions: (optional) A list of strings to use as captions for each object
    """
    # Number of instances
    N = boxes.shape[0]
    if not N:
        print("\n*** No instances to display *** \n")
    else:
        assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]

    # If no axis is passed, create one and automatically call show()
    #auto_show = False
    #if not ax:
        # _, ax = plt.subplots(1, figsize=figsize)
        #auto_show = True

    # Generate random colors
    colors = colors or random_colors(N)

    # Show area outside image boundaries.
    #height, width = image.shape[:2]
    #ax.set_ylim(height + 10, -10)
    #ax.set_xlim(-10, width + 10)
    #ax.axis('off')
    #ax.set_title(title)

    masked_image = image.astype(np.uint32).copy()
    for i in range(N):
        color = colors[i]

        # Bounding box
        if not np.any(boxes[i]):
            # Skip this instance. Has no bbox. Likely lost in image cropping.
            continue
        y1, x1, y2, x2 = boxes[i]
        if show_bbox:
            p = patches.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2,
                                alpha=0.7, linestyle="dashed",
                                edgecolor=color, facecolor='none')
            #ax.add_patch(p)

        # Label
        if not captions:
            class_id = class_ids[i]
            score = scores[i] if scores is not None else None
            label = class_names[class_id]
            caption = "{} {:.3f}".format(label, score) if score else label
        else:
            caption = captions[i]
        #ax.text(x1, y1 + 8, caption,
        #        color='w', size=11, backgroundcolor="none")

        # Mask
        mask = masks[:, :, i]
        if show_mask:
            masked_image = apply_mask(masked_image, mask, color)

        # Mask Polygon
        # Pad to ensure proper polygons for masks that touch image edges.
        padded_mask = np.zeros(
            (mask.shape[0] + 2, mask.shape[1] + 2), dtype=np.uint8)
        padded_mask[1:-1, 1:-1] = mask
        contours = find_contours(padded_mask, 0.5)
        for verts in contours:
            # Subtract the padding and flip (y, x) to (x, y)
            verts = np.fliplr(verts) - 1
            p = Polygon(verts, facecolor="none", edgecolor=color)
            #ax.add_patch(p)
    #ax.imshow(masked_image.astype(np.uint8))
    #if auto_show:
        # plt.show()
    
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', masked_image.astype(np.uint8))[1]).tostring()
    pub_res.publish(msg)

def response(body_data):
    global verbose, detector, class_names
    body = body_data
    image = np.array(body['image_array'])
    desired_classes = body["classes"]
    detection_results = detector.detect(image, desired_classes)
    labels = pd.read_csv('labels.txt')
    #colors = dict()
    cls_id = detection_results["class_ids"]
    for ii in range(len(cls_id)):
        #colors[cls_id] = (int(random.random()*255), int(random.random()*255), int(random.random()*255))
        print("detected object:", labels[cls_id[ii]])
    display_instances(image, detection_results["rois"], detection_results["masks"], detection_results["class_ids"], class_names, detection_results["scores"])

def pointcloud2_to_rgb(pc):
        x = np.frombuffer(pc.data, 'uint8').reshape(-1, 8, 4)
        bgr = x[:pc.height*pc.width, 4, :3].reshape(pc.height, pc.width, 3)
        #rgb = bgr[:,:,::-1]

        return bgr

def detect(pc2_message, desired_classes=class_names):
    image = pointcloud2_to_rgb(pc2_message)
    body_data = {"image_array": image.tolist(), "classes":desired_classes}
    response(body_data)

rospy.spin()