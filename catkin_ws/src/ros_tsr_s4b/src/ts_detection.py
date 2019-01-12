#!/usr/bin/env python
import rospy
from rospkg import RosPack
from sensor_msgs.msg import Image
from ros_tsr_s4b.msg import TSCandidate
from cv_bridge import CvBridge, CvBridgeError
from ts_detection.Detection import SingleShotMultiBoxDetector
from threading import Thread
from data_model.ts_model_repo import TSModelRepository

bridge = CvBridge();
rospack = RosPack()

## params ##

PKG_PATH = rospack.get_path('ros_tsr_s4b')
IMAGE_TOPIC = rospy.get_param('/ts_detection/image_topic')
MIN_CONFIDENCE = rospy.get_param('/ts_detection/min_confidence')
MODEL_NAME = rospy.get_param('/ts_detection/model_name')
MODEL_FOLDER = PKG_PATH+"/models/detection/"+MODEL_NAME
demo_mode = rospy.get_param('/ts_detection/visualize')
WIDTH=640
HEIGHT=480
## vars ##
repo = None
detector = None
frame = None


if demo_mode:
    from cv2 import rectangle,putText,FONT_HERSHEY_PLAIN


def on_frame(ros_msg):
    global frame

    try:
        frame = bridge.imgmsg_to_cv2(ros_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logwarn("TS Detector Warning: {}".format(e))


def detect():
    global detector,repo
    detector = SingleShotMultiBoxDetector(MODEL_FOLDER)
    repo = TSModelRepository(PKG_PATH+"/src/data_model/symbols")

    rospy.loginfo("Start detection..")
    ts_detection_publisher = rospy.Publisher("/ros_tsr_s4b/ts_detection",TSCandidate,queue_size=1)
    if demo_mode:
        demo_image_publisher = rospy.Publisher("/ros_tsr_s4b/ts_detection_demo",Image,queue_size=1)
    min_conf = MIN_CONFIDENCE*0.01
    while not rospy.is_shutdown():

        try:
            assert isinstance(detector, object)
            fr = frame
            regions = detector.findRegions(fr, min_conf)
            for r in regions:
                region = TSCandidate()
                region.label = str(r.label)
                region.symbol_path = repo.getSymbolPathByName(region.label)
                region.score = int(r.score*100.0)
                region.x1 = int(r.x1*float(WIDTH))
                region.y1 = int(r.y1*float(HEIGHT))
                region.x2 = int(r.x2*float(WIDTH))
                region.y2 = int(r.y2*float(HEIGHT))
                ts_detection_publisher.publish(region)

                if demo_mode:
                    color = (0,255,0)
                    rectangle(fr, (region.x1, region.y1), (region.x2, region.y2), color, 2)
                    text = region.label + " " + str(round(region.score, 2)) + "%"
                    putText(fr, text, (region.x1, region.y1 - 10), FONT_HERSHEY_PLAIN, 0.8,color, 1)
                    symbol = repo.getSymbolByName(region.label)
                    try:
                        if region.score > MIN_CONFIDENCE:
                            repo.transparentOverlay(fr, symbol, (region.x1, region.y1), scale=0.3)
                    except Exception as e:
                        print("TS Detector Warning: Error while loading symbol for {} , caused by {}".format(region.label,e))
            if demo_mode:
                demo_image_publisher.publish(bridge.cv2_to_imgmsg(fr))
        except Exception as e:
            rospy.logwarn("TS Detector Warning: {}".format(e))
            rospy.sleep(5)
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        rospy.init_node("ts_detection")
        rospy.Subscriber(IMAGE_TOPIC, Image, on_frame)
        detection_thread = Thread(target=detect)
        detection_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
