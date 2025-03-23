#!/usr/bin/env python3.8
# filepath: ~/alphaz_ws/src/handle_detection/scripts/handle_detection_node.py

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge
import os
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import torch

import detectron2
from detectron2.data import MetadataCatalog
from detectron2.utils.logger import setup_logger


center_net_path = os.path.join(os.path.dirname(__file__), 'Detic/third_party/CenterNet2')
sys.path.insert(0, os.path.abspath(center_net_path))


# import some common detectron2 utilities
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
# from detectron2.data import MetadataCatalog, DatasetCatalog


from centernet.config import add_centernet_config
from Detic.detic.config import add_detic_config
from segment_anything import sam_model_registry, SamPredictor

############################################
def DETIC_predictor():
    # Build the detector and download our pretrained weights
    cfg = get_cfg()
    add_centernet_config(cfg)
    add_detic_config(cfg)
    cfg.merge_from_file("configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml")
    cfg.MODEL.WEIGHTS = 'https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth'
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.1 # set threshold for this model
    cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand'
    cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True # For better visualization purpose. Set to False for all classes.
    cfg.MODEL.DEVICE='cuda' # 'cuda' or 'cpu'
    detic_predictor = DefaultPredictor(cfg)
    return detic_predictor

def SAM_predictor(device):
    sam_checkpoint = "/root/sam_vit_h_4b8939.pth"
    model_type = "vit_h"
    device = device
    sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    sam.to(device=device)
    sam_predictor = SamPredictor(sam)
    return sam_predictor

def Detic(im, metadata, detic_predictor, visualize=False):
    if im is None:
        print("Error: Unable to read the image file")

    # Run model and show results
    output =detic_predictor(im[:, :, ::-1])  # Detic expects BGR images.
    v = Visualizer(im, metadata)
    out = v.draw_instance_predictions(output["instances"].to('cpu'))
    instances = output["instances"].to('cpu')
    boxes = instances.pred_boxes.tensor.numpy()
    classes = instances.pred_classes.numpy()
    if visualize:
        visualize_detic(out)
    return boxes, classes

def SAM(im, boxes, class_idx, metadata, sam_predictor):
    sam_predictor.set_image(im)
    input_boxes = torch.tensor(boxes, device=sam_predictor.device)
    transformed_boxes = sam_predictor.transform.apply_boxes_torch(input_boxes, im.shape[:2])
    masks, _, _ = sam_predictor.predict_torch(
        point_coords=None,
        point_labels=None,
        boxes=transformed_boxes,
        multimask_output=False,
    )
    return masks

def visualize_output(im, masks, input_boxes, classes, image_save_path, mask_only=False):
    plt.figure(figsize=(10, 10))
    plt.imshow(im)
    for mask in masks:
        show_mask(mask.cpu().numpy(), plt.gca(), random_color=True)
    if not mask_only:
        for box, class_name in zip(input_boxes, classes):
            show_box(box, plt.gca())
            x, y = box[:2]
            plt.gca().text(x, y - 5, class_name, color='white', fontsize=12, fontweight='bold', bbox=dict(facecolor='green', edgecolor='green', alpha=0.5))
    plt.axis('off')
    plt.savefig(image_save_path)
    plt.show()

def visualize_detic(output):
    output_im = output.get_image()[:, :, ::-1]
    cv2.imshow("Detic Predictions", output_im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)

def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2))

############################################


class HandleDetectionNode:
    def __init__(self):
        rospy.init_node('handle_detection_node', anonymous=True)

        # Parameters
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.classes = ["handle"]
        self.threshold = 0.3

        # Initialize predictors
        self.detic_predictor = DETIC_predictor()
        self.sam_predictor = SAM_predictor(self.device)
        self.metadata = MetadataCatalog.get("__unused2")
        self.metadata.thing_classes = self.classes

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        rospy.Subscriber("/camera_0/color/image_raw", rosImage, self.image_callback)

        # OpenCV Window
        cv2.namedWindow("Handle Detection", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        # try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None or cv_image.size == 0:
                rospy.logerr("cv_image is empty after conversion.")
                return
            
            # Perform detection
            boxes = [1, 2]
            # boxes, class_idx = Detic(cv_image, self.metadata, self.detic_predictor, visualize=False)
            if len(boxes) > 0:
                # masks = SAM(cv_image, boxes, class_idx, self.metadata, self.sam_predictor)
                # classes = [self.metadata.thing_classes[idx] for idx in class_idx]

                # Visualize results
                # visualize_output(cv_image, masks, boxes, classes, None, mask_only=False)

                # Show in OpenCV window
                cv2.imshow("Handle Detection", cv_image)
                # cv2.waitKey(1)
            else:
                rospy.loginfo("No handles detected.")
        # except Exception as e:
        #     rospy.logerr(f"Error in image_callback: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    setup_logger()      # detectron2 logger

    # Run the node
    node = HandleDetectionNode()
    node.run()