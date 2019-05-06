import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
from torchvision import models, transforms
from PIL import Image

def imshow(img, title=None):
    '''
    :param img: a tensor of img
    :return: show and save a image
    '''
    saving_path = 'C:\\Users\Ding\Desktop\zkcup\zkcup\Save picture\\'
    filename = time.strftime('%H-%M-%S', time.localtime(time.time()))
    # transform to numpy ndarrays
    # numpy image: H x W x C
    # torch image: C X H X W
    img = img.numpy().transpose(1, 2, 0)
    # innomalize
    # torch image:[0.0, 1.0]
    # numpy image:[0, 255]
    mean = [0.485, 0.456, 0.406]
    std = [0.229, 0.224, 0.225]
    img = img * std + mean
    # np.clip(img, low, high)截取函数, for x in img, if x < low: x = low; if x > high: x = high
    # 利用这一行才能接着使用plt.imshow
    img = np.clip(img, 0.0, 1.0)
    if title is not None:
        plt.title(title)
    plt.imshow(img)
   # plt.savefig(saving_path+filename+'.png')
    plt.pause(1)

def nnpred():

    result = 255
    CaptureFailCount = 0
    class_name = {0: "ADmilk", 1: "Deluxe", 2: "RedBull",
               3: "Tennis", 4: "Yakult"}
    model_path = 'C:\\Users\Ding\Desktop\zkcup\zkcup\Save model\loss=0.035.tar'

    # 加载模型
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = models.resnet18(pretrained=True)
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, len(class_name))
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model = model.to(device)
    model.eval()
    
    
    cap = cv2.VideoCapture(1)
    if cap.isOpened() == False:
        print("Camare open failed")
        cap.open(0)
    
    # 调用opencv拍照
    # ret == Ture if 成功截获一帧
    count = 0
    while(count < 30):
    # get a frame
        ret, frame = cap.read()
        # 截获失败处理
        while ret == False:
            if(CaptureFailCount > 5):
                # 截取失败
                result = 254
                return result
            print("Capture Failed, recapturing....")
            cv2.waitKey(100)
            ret, frame = cap.read()
            CaptureFailCount += 1
        # show a frame
        cv2.imshow("capture", frame)
        cv2.waitKey(1)
        count += 1

    cv2.imshow("Capture", frame)
    cv2.waitKey(500)

    cap.release()
    cv2.destroyAllWindows()

    # cv2图像转化成PIL图像
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    print(type(img))
    transform = transforms.Compose([
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    img = transform(img)
    img = img.unsqueeze(0)  # img增加一个sample维度

    # 前向传播
    output = model(img)
    print(output)
    _, result = torch.max(output, 1)
    imshow(img.squeeze(0), class_name[result.item()])

    return result.item()

if __name__ == '__main__':
    
    x = nnpred()
    print(x)
