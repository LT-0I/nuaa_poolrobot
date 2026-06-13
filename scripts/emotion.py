import cv2
import mediapipe as mp
from deepface import DeepFace
from collections import Counter
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 初始化Mediapipe人脸检测
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# 打开摄像头
cap = cv2.VideoCapture(0)

# 定义情绪历史列表
emotion_history = []
emotion_window = []

# 初始化ROS节点
rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('emotion', String, queue_size=10)
pub_image = rospy.Publisher('emotion_image', Image, queue_size=10)  # 发布图像的话题
rate = rospy.Rate(100)  # 10hz

# 初始化 CvBridge
bridge = CvBridge()

# 标志变量，用于控制循环退出
exit_flag = False

with mp_face_detection.FaceDetection(min_detection_confidence=0.7) as face_detection:
    while not rospy.is_shutdown() and not exit_flag:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头画面")
            break

        # 将图像转换为RGB格式（Mediapipe需要RGB格式）
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 使用Mediapipe检测人脸
        results = face_detection.process(rgb_frame)

        # 如果有检测到人脸
        if results.detections:
            for detection in results.detections:
                # 获取人脸的边界框
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                             int(bboxC.width * iw), int(bboxC.height * ih)

                # 确保边界框在图像范围内
                x = max(0, x)
                y = max(0, y)
                w = min(iw - x, w)
                h = min(ih - y, h)

                # 如果人脸区域太小，跳过处理
                if w < 100 or h < 100:
                    continue

                # 截取人脸区域
                face_image = frame[y:y+h, x:x+w]

                # 检查 face_image 是否为空
                if face_image.size == 0:
                    print("人脸区域为空，跳过处理")
                    continue

                try:
                    # 使用DeepFace进行情绪识别
                    analysis = DeepFace.analyze(face_image, actions=['emotion'], enforce_detection=False)
                    
                    # 检查分析结果是否为空
                    if not analysis:
                        print("未检测到人脸，跳过处理")
                        continue

                    # 获取主要情绪
                    dominant_emotion = analysis[0]['dominant_emotion']

                    # 多帧平均法
                    if len(emotion_history) > 10:  # 保留最近10帧的结果
                        emotion_history.pop(0)
                    emotion_history.append(dominant_emotion)
                    dominant_emotion = Counter(emotion_history).most_common(1)[0][0]

                    # 在图像上绘制情绪结果
                    cv2.putText(frame, dominant_emotion, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)

                    # 发布情绪结果到ROS话题
                    pub.publish(dominant_emotion)
                except Exception as e:
                    print(f"情绪识别失败: {e}")

                # 绘制人脸的边界框
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # 将图像帧转换为 ROS 消息并发布
        try:
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")  # 将 OpenCV 图像转换为 ROS 消息
            pub_image.publish(ros_image)  # 发布图像
        except Exception as e:
            print(f"图像转换失败: {e}")

        # 显示画面
        # cv2.imshow("Real-time Emotion Detection", frame)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag = True  # 设置退出标志
        
        rate.sleep()

# 释放资源
cap.release()
cv2.destroyAllWindows()

# 关闭ROS节点
rospy.signal_shutdown("程序退出")