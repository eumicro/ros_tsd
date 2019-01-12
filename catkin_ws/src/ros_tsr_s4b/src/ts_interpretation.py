#!/usr/bin/env python
import rospy
from rospkg import RosPack
from ros_tsr_s4b.msg import TSCandidate
from ros_tsr_s4b.srv import *
from ts_interpretation.TSInterpretation import TSRuleBase
from threading import Thread
rospack = RosPack()
PKG_PATH = rospack.get_path('ros_tsr_s4b')
UPDATE_INTERVAL = rospy.get_param('/ts_interpretation/update_interval')
demo_mode = rospy.get_param('/ts_interpretation/visualize')
ts_list = []
rules = None
repo = None
if demo_mode:
    from sensor_msgs.msg import Image
    from data_model.ts_model_repo import TSModelRepository
    from cv_bridge import CvBridge, CvBridgeError
    from cv2 import rectangle,putText,FONT_HERSHEY_PLAIN
def on_ts_detected(ts):
    global ts_list
    new_ts_name = ts.label
    ts_list = rules.update(new_ts_name,ts_list)
    #rospy.loginfo("Update {} ".format(ts_list))
def update():
    if demo_mode:
        import numpy as np
        demo_image_publisher = rospy.Publisher("/ros_tsr_s4b/ts_interpretation_demo", Image, queue_size=1)

        bridge = CvBridge()
    while not rospy.is_shutdown():

        if demo_mode:
            new_image = np.zeros((200, 800, 3), np.uint8)
            color = (255,255,255)
            text_size = 1.0
            thickness = 1
            putText(new_image, "Speed", (150, 175), FONT_HERSHEY_PLAIN, text_size, color, thickness)
            putText(new_image, "Direction", (250, 175), FONT_HERSHEY_PLAIN, text_size, color, thickness)
            putText(new_image, "Warnings", (350, 175), FONT_HERSHEY_PLAIN, text_size, color, thickness)
            putText(new_image, "Regulation", (450, 175), FONT_HERSHEY_PLAIN, text_size, color, thickness)
            putText(new_image, "Rights of Way", (550, 175), FONT_HERSHEY_PLAIN, text_size, color, thickness)
            for ts_name in ts_list:

                symbol = repo.getSymbolByName(ts_name)
                names = rules.getByName(ts_name).getNames()

                if "speed-limit" in names:
                    position = (150,50)
                elif "mandatory-direction" in names:
                    position = (250, 50)
                elif "warning" in names:
                    position = (350, 50)
                elif "regulatory" in names:
                    position = (450, 50)
                elif "right-of-way" in names:
                    position = (550, 50)
                else:
                    position = (650, 50)
                repo.transparentOverlay(new_image,symbol,position, scale=0.55)
            demo_image_publisher.publish(bridge.cv2_to_imgmsg(new_image))

        rospy.sleep(UPDATE_INTERVAL)
def on_get_valid_signs_request(request):
    valid_ts_list = []
    for ts_name in ts_list:
        ts = TSCandidate()
        ts.label = ts_name
        ts.symbol_path = repo.getSymbolPathByName(ts_name)
        valid_ts_list.append(ts)
    return GetValidSignsResponse(valid_ts_list)

if __name__ == '__main__':
    global rules,repo
    try:
        repo = TSModelRepository(PKG_PATH + "/src/data_model/symbols")
        rules = TSRuleBase(PKG_PATH+"/models/interpretation/ts_rules.xml")
        rospy.init_node("ts_interpretation")
        rospy.loginfo("Start TS Interpreter")
        # wait for detector
        rospy.sleep(10)
        # subscribe to detector
        rospy.Subscriber("/ros_tsr_s4b/ts_detection", TSCandidate, on_ts_detected)
        update_thread = Thread(target=update)
        update_thread.start()
        # start service
        service = rospy.Service('/ros_tsr_s4b/ts_interpretation/get_valid_signs', GetValidSigns, on_get_valid_signs_request)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
