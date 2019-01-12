import cv2
from ts_detection.Detection import SingleShotMultiBoxDetector
from data_model.ts_model_repo import TSModelRepository
from threading import Thread
width = 600
height = 600
#source = '/home/eumicro/Schreibtisch/demo.avi'
source=0
video_capture = cv2.VideoCapture(source)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,width)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
#video_capture.set(cv2.CAP_PROP_BRIGHTNESS,0.6)
#video_capture.set(cv2.CAP_PROP_SATURATION,0.6)

model_path = '/home/eumicro/PycharmProjects/tensor_cnn/development/ssd_mobilenet_smallest_gen_data_v1model/model/graph-613622/frozen_inference_graph.pb'
label_path = '/home/eumicro/data/raw/GTS_Generated_600_Minimal/label_map.pbtxt'
num_classes = 58
detector = SingleShotMultiBoxDetector(model_path,label_path,num_classes)
detector.startSession()
frame = None
repo = TSModelRepository("/home/eumicro/PycharmProjects/tensor_cnn/application/data_model/symbols")
def detect(x1,y1,x2,y2):
    global frame
    window = frame[y1:y2,x1:x2]
    regions = detector.findRegions(window, min_score_thresh=0.8)
    ## visualize
    for r in regions:

        text = r.label + " " + str(round(r.score * 100, 2)) + "%"
        symbol = repo.getSymbolByName(r.label)
        color = (0, 255, 0)
        try:
            repo.transparentOverlay(frame, symbol, (x1 + r.x1, y1 + r.y1), scale=0.3)
        except:
            print("Error while loading symbol for " + r.label)
        cv2.rectangle(frame, (x1+r.x1,y1+r.y1), (x1+r.x2,y1+r.y2), color, 2)
        cv2.putText(frame, text, (x1+r.x1, y1+r.y1 - 10), cv2.FONT_HERSHEY_PLAIN, 0.8,
                    color, 1)

counter = 0
grid_size = 1
while (video_capture.isOpened()):

    #counter = (counter+1) % 5
    # Capture frame-by-frame

    # grid zoom

    #grid_size=2 if counter==0 else 1
    ret, frame = video_capture.read()
    #frame[:,:,0] = cv2.equalizeHist(frame[:,:,0])
    #frame[:, :, 1] = cv2.equalizeHist(frame[:, :, 1])
    #frame[:, :, 2] = cv2.equalizeHist(frame[:, :, 2])
    for i in range(grid_size):
        for j in range(grid_size):
            x1 = int((i/float(grid_size))*width)
            y1 = int((j/float(grid_size))*height)
            x2 = int(((i+1)/float(grid_size)) * width)
            y2 = int(((j+1)/float(grid_size)) * height)
            detect(x1, y1, x2, y2)
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0, 0, 0), 2)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
video_capture.release()
cv2.destroyAllWindows()
