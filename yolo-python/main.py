import cv2
import os
import argparse
import yolo_opencv

def covertFrameToVideo(img_dir, video_dir, fps, start_num, end_num, img_size):
    """
    描述: 将文件夹中的所有图片转换成视频
    参数: img_dir: 放置图片的文件夹，所有文件的名字使用数字按顺序命名
          video_dir: 最后产生视频的路径名，最后一个子路径是视频文件的名称
          fps: 生成视频的帧率
          start_num: 图片的起始位置
          end_num: 图片的结束位置
          img_size: 视频的分辨率，设置成和图片的分辨率一样
    返回: none
    """
    #fourcc = cv2.cv.CV_FOURCC('M','J','P','G')#opencv2.4
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G') #opencv3.0
    videoWriter = cv2.VideoWriter(video_dir, fourcc, fps, img_size)

    for i in range(start_num, end_num):
        img_name = img_dir + '/' + str(i)+'.jpg'
        frame = cv2.imread(img_name)
        videoWriter.write(frame)
        print(img_name)

    videoWriter.release()
    print('finish')

def covertIamgeToLabelledImageWithYolo(img_dir, labelled_img_dir, start_num, end_num):
    """
    描述: 使用yolo检测图片然后将标记后的图片保存到指定的文件夹
    参数: img_dir: 需要检测的图片文件
          labelled_img_dir: 保存加标签后的图片的文件夹
          start_num: 图片的起始序号
          end_num: 图片的结束序号
    返回: none
    """
    for i in range(start_num, end_num):
        img_path = img_dir + '/' + str(i) +'.jpg'
        labelled_img_path = labelled_img_dir + '/' + str(i) +'.jpg'
        labelled_img = yolo_opencv.yolo(img_path)
        # cv2.imshow("object detection", labelled_img)
        # cv2.waitKey(1)
        cv2.imwrite(labelled_img_path, labelled_img)
        print(labelled_img_path)

def getFrameFromVideo(video_dir, img_dir, fpsInterval): 
    """
    描述: 将视频中的帧以图片形式保存到文件夹
    参数: video_dir: 视频文件路径
          img_dir: 保存图片的路径
          fpsInterval: 每隔几帧保存一次图片
    返回: none
    """
    vc = cv2.VideoCapture(video_dir) #读入视频文件
    if vc.isOpened(): #判断是否正常打开
        rval , frame = vc.read()
    else:
        rval = False
    
    c=1
    index=0
    while rval:   #循环读取视频帧
        rval, frame = vc.read()
        if(c%fpsInterval == 0): #每隔timeF帧进行存储操作
            path = img_dir + '/' + str(index) + '.jpg'
            cv2.imwrite(path, frame) #存储为图像
            index += 1
            print(path)
        c = c + 1

    vc.release()


def main():

    def convertVideoToImage():
        video_dir = "video/new_video/video2.mp4"
        img_dir = "video/image/image2"
        fpsInterval = 1
        getFrameFromVideo(video_dir, img_dir, fpsInterval)


    def convertImageToLabelledImage():
        # img_dir= "4897.jpg"
        # labelled_img = yolo_opencv.yolo(img_path)
        # cv2.imwrite("object-detection.jpg", labelled_img)

        img_dir = "video/image/image3"
        labelled_img_dir = "video/labelled_image/image3"
        start_num = 5100
        end_num = 6051
        covertIamgeToLabelledImageWithYolo(img_dir, labelled_img_dir, start_num, end_num)
         
            
    def convertImageToVideo():
        video_path = "video/new_video/video2.mp4"
        labelled_img_dir = "video/labelled_image/image3_target"
        fps = 55
        start_num = 1100
        end_num = 6051
        img_size = (2928, 1052)
        # img_size = (1280, 720)   

        covertFrameToVideo(labelled_img_dir, video_path, fps, start_num, end_num, img_size)

    # cv2.namedWindow('object detection', cv2.WINDOW_NORMAL) #使opencv显示图片时适应屏幕大小

    convertVideoToImage()
    # convertImageToLabelledImage()
    # convertImageToVideo()


if __name__=="__main__":
    main()