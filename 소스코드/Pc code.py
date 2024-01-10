import cv2
import socket
import pickle
import numpy as np
import time
import requests
from PIL import Image, ImageDraw, ImageFont 
from twilio.rest import Client
time1=time.time()

##노트북 소켓 통신
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = "192.168.45.204"
port = 6666
s.bind(('', port))


##라즈베리파이 소켓 통신
pi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
pi_ip = "192.168.45.43" #라즈베리파이의 IP
pi_port = 6667
pi_socket.connect((pi_ip, pi_port))


##  박스 겹침 확인 함수
def check_overlap(box1, box2):
    x1_max = max(box1[0], box2[0])
    x2_min = min(box1[2], box2[2])
    y1_max = max(box1[1], box2[1])
    y2_min = min(box1[3], box2[3])
    
    overlap = max(0, x2_min - x1_max) * max(0, y2_min - y1_max)
    return overlap > 0

try:
    model = cv2.dnn.readNetFromTensorflow('C:\\SeoulAIOT\\server code\\frozen_inference_graph.pb',
                                        'C:\\SeoulAIOT\\server code\\ssd_mobilenet_v2_coco_2018_03_29.pbtxt')
            
    while True:
        start_time = time.time()
        data, addr = s.recvfrom(1000000)
        data_arr = np.frombuffer(data, np.uint8)
        img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        
        if img is not None:
            image_height, image_width, _ = img.shape
            roi_size = (640, 320)  # 원하는 ROI의 크기
            x_offset = (image_width - roi_size[0]) // 2
            y_offset = (image_height - roi_size[1]) // 2
            roi = img[y_offset:y_offset+roi_size[1], x_offset:x_offset+roi_size[0]]
        
        # 관심영역을 검은색 사각형으로 표시
            model.setInput(cv2.dnn.blobFromImage(roi, size=(300, 300), swapRB=True))
            output = model.forward()
            person_boxes = []
            knife_boxes = []

            for detection in output[0, 0, :, :]:
                confidence = detection[2]
                class_id = int(detection[1])
                
                box_x = detection[3] * roi_size[0] + x_offset  # 좌표 변환
                box_y = detection[4] * roi_size[1] + y_offset  # 좌표 변환
                box_width = detection[5] * roi_size[0] + x_offset  # 좌표 변환
                box_height = detection[6] * roi_size[1] + y_offset  # 좌표 변환

                # 사람인 경우
                if class_id == 1 and confidence > 0.5:
                    person_boxes.append([int(box_x), int(box_y), int(box_width), int(box_height)])
                    cv2.rectangle(img, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (0, 255, 0), 1)
                    
                    # Pillow를 사용하여 한글 텍스트를 그리기.
                    pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  # OpenCV 이미지를 PIL 이미지로 변환
                    draw = ImageDraw.Draw(pil_img)
                    font = ImageFont.truetype("malgun.ttf", 15)  # 한글 폰트 경로와 크기 설정
                    label_korean = f"민간인: {confidence:.2f}"
                    draw.text((int(box_x), int(box_y) - 20), label_korean, font=font, fill=(0, 255, 255))
                    img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)  # 다시 PIL 이미지를 OpenCV 이미지로 변환
                    
                
                # 칼인 경우
                if class_id == 49 and confidence > 0.05:
                    knife_boxes.append([int(box_x), int(box_y), int(box_width), int(box_height)])
                    cv2.rectangle(img, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (0, 51, 255), 1)
                    label = f"Knife: {confidence:.2f}"
                    cv2.putText(img, label, (int(box_x), int(box_y) - 10), cv2.FONT_HERSHEY_DUPLEX , 0.5, (0, 51, 255), 1)
                    
            # IOU 구현위해 겹침 확인 및 사각형 그리기
            for p_box in person_boxes:
                for k_box in knife_boxes:
                    if check_overlap(p_box, k_box):
                        cv2.rectangle(img, (p_box[0], p_box[1]), (p_box[2], p_box[3]), (0, 0, 255), 3)
                        cv2.putText(img, 'armed_person', (p_box[0], p_box[1]-10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 3)
                        pi_socket.send(b'armed_person_detected') #라즈베리파이로 무장한 사람이 감지되었다고 알림
                        '''                 
                        position='37.528291,126.933312'    
                        time2=time.time()
                    
                        if time2-time1>5:
                            sss=time.strftime('%c', time.localtime(time.time()))
                            data1= 'car1 knife 인식 위치:{} 시간:{} '.format(position,sss)
                            account_sid = 'your account sid'
                            auth_token = 'your token'
                            client = Client(account_sid, auth_token)
                            message = client.messages.create(
                            from_='+12563685788',
                            body=data1.encode(),
                            to='+821031198106'
                            )
                            print("data")
                            time1=time.time()
                            '''
        end_time = time.time()
        fps = 1 / (end_time - start_time)
        cv2.putText(img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 102, 0), 2)
            
        cv2.imshow('Img Server', img)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    cv2.destroyAllWindows()
    s.close()

   
            
