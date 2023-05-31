import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image as cv_bridge_Image
import cv2
import numpy as np
from cv_bridge import CvBridge

rospy.init_node("circle_dectect_node")
rate = rospy.Rate(30)
bridge = CvBridge()
xyr = Point()
num_image = None
num_result = None
decting_num = False
image = None
circles = None

video_dir = 'videos/output.mp4'
# cap = cv2.VideoCapture(video_dir)
# cap = cv2.VideoCapture(0)

circle_center_pub = rospy.Publisher("/circle/point", Point, queue_size=1)
image_pub = rospy.Publisher("/camera/image_raw", cv_bridge_Image, queue_size=1)

print("circle_dectect_start")

def img_convert(img):
    # opencv图像转换为sensor_msgs/Image图像
    image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    # 把sensor_msgdect_num()s/Image图像转换为oposition.pencv图像
    # cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
    return image_message


def img_process(image):
    # HSV颜色空间过滤掉一些颜色
    imgHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower = np.array([0,0,200])
    upper = np.array([179,255,255])
    mask = cv2.inRange(imgHSV,lower,upper)
    imgResult = cv2.bitwise_and(image,image,mask=mask)
    return imgResult

def take_circle_point():
    global image
    # ret, image = cap.read()
    # frame = img_convert(image)
    # image_pub.publish(frame)

    image = img_process(image)

    medianb = cv2.medianBlur(image, 3)
    filter_gray = cv2.cvtColor(medianb, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(filter_gray,(9,9),0)

    # 霍夫圆检测
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT_ALT, 1.5, 30,
                                  param1=300,param2=0.9,minRadius=5,maxRadius=300)
    #circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 30,
    #                                param1=100,param2=30,minRadius=0,maxRadius=500)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        x, y, r = circles[0]
        xyr.x = x
        xyr.y = y
        xyr.z = r
        # global num_image
        # num_image = take_num(image, x, y, r)
        cv2.circle(image, (x, y), r, (0,255,0), 3)
        # Draw a rectangle(center) in the output image
        cv2.rectangle(image, (x - 2, y - 2), (x + 2, y + 2), (0,255,0), -1)
    cv2.imshow("Detections",image)
    cv2.waitKey(5)

def image_callback(msg):
    global image
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    frame = img_convert(image)
    image_pub.publish(frame)


image_iris_sub = rospy.Subscriber("/iris_0/usb_cam/image_raw", cv_bridge_Image, image_callback, queue_size=1)

while not rospy.is_shutdown(): # and cap.isOpened(): 
    if image is not None:
        take_circle_point()
        circle_center_pub.publish(xyr)
    rate.sleep()

cv2.destroyAllWindows()