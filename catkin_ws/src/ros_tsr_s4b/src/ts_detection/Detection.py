from data_model.TSData import Region

from matplotlib import pyplot as plt
from math import sqrt
import numpy as np
import tensorflow as tf
from object_detection.utils import label_map_util

import cv2

class TSDetector:
    def findRegions(self,image):
        raise NotImplemented("This method must be implemented by a sub class of TSDetector.")

class SingleShotMultiBoxDetector(TSDetector):
    def __init__(self,model_folder):
        # Path to frozen detection graph. This is the actual model that is used for the object detection.

        self.PATH_TO_CKPT = model_folder+"/frozen_inference_graph.pb"
        self.PATH_TO_LABELS = model_folder+"/label_map.pbtxt"
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.NUM_CLASSES = len(self.label_map.item)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)
        self.startSession()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sess.close()
    def _load_image_into_numpy_array(self,image):
        (im_width, im_height, channels) = image.shape
        return np.array(image).reshape(
            (im_width, im_height, channels)).astype(np.uint8)
    def startSession(self):
        with tf.Session(graph=self.detection_graph) as self.sess:
            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def findRegions(self,image,min_score_thresh=0.5):
        width, height, channesl = image.shape
        image_np = self._load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        regions = []
        if (any(min_score_thresh < max(scores))):
            for i, box in enumerate(boxes):
                for j, score in enumerate(scores[i]):
                    if score > min_score_thresh:
                        y1 = box[j][0]
                        x1 = box[j][1]
                        y2 = box[j][2]
                        x2 = box[j][3]
                        class_name = self.category_index[int(classes[i][j])]['name']
                        region = Region(x1=x1,y1=y1,x2=x2,y2=y2,label=class_name,score=score)
                        regions.append(region)

        return regions