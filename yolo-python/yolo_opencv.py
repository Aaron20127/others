#############################################
# Object detection - YOLO - OpenCV
# Author : Arun Ponnusamy   (July 16, 2018)
# Website : http://www.arunponnusamy.com
############################################

import cv2
import argparse
import numpy as np

# ap = argparse.ArgumentParser()
# ap.add_argument('-i', '--image', required=True,
#                 help = 'path to input image')
# ap.add_argument('-c', '--config', required=True,
#                 help = 'path to yolo config file')
# ap.add_argument('-w', '--weights', required=True,
#                 help = 'path to yolo pre-trained weights')
# ap.add_argument('-cl', '--classes', required=True,
#                 help = 'path to text file containing class names')
# args = ap.parse_args()

def yolo(image_dir):

    def get_label_color(label):
        color = { 'person':         np.array([  0.0, 255.0, 255.0]),
                    'bicycle':       np.array([255.0,   0.0,   0.0]),
                    'car':           np.array([255.0,   0.0, 255.0]),
                    'motorcycle':    np.array([160.0, 32.0, 240.0]),
                    'traffic light': np.array([255.0, 118.0, 72.0]),
                    'parking meter': np.array([144.0, 238.0, 144.0]),
                    'stop sign':     np.array([0.0, 165.0, 255.0])
                }
        if label in color:
            return color[label]
        else:
            return color['stop sign']

    def get_output_layers(net):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers

    def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):    
        label = str(classes[class_id])
        label_last = label +  ":{:.1f}%".format(confidence*100)

        tectangle_color = get_label_color(label)
        name_len = int(146 * len(label) / 6)
        confidence_len = 160
        cv2.rectangle(img, (x-20, y+10), (x-10 + name_len + confidence_len, y-55), tectangle_color, -1)
        cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), tectangle_color, 2)

        text_color = np.array([0.0, 0.0, 0.0])
        cv2.putText(img, label_last, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 1.4, text_color, 2)
 

    class myargs():
        classes='yolov3.txt'
        config='yolov3.cfg'
        image=image_dir
        weights='yolov3.weights'

    args = myargs()

    image = cv2.imread(args.image)

    Width = image.shape[1]
    Height = image.shape[0]
    scale = 0.00392

    classes = None

    with open(args.classes, 'r') as f:
        classes = [line.strip() for line in f.readlines()]

    # COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

    net = cv2.dnn.readNet(args.weights, args.config)

    blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

    net.setInput(blob)

    outs = net.forward(get_output_layers(net))

    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4


    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * Width)
                center_y = int(detection[1] * Height)
                w = int(detection[2] * Width)
                h = int(detection[3] * Height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])


    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    for i in indices:
        i = i[0]
        box = boxes[i]
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))

    # cv2.imshow("object detection", image)
    # cv2.waitKey(100)
        
    # # cv2.imwrite("object-detection.jpg", image)

    return image
    # cv2.destroyAllWindows()

# yolo()