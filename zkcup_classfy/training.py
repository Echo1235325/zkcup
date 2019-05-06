import torch
import torch.nn as nn
import torch.optim as optim
from torch.optim import lr_scheduler
import torchvision
from torchvision import models, datasets, transforms
import matplotlib.pyplot as plt
import numpy as np
import os
import copy
from scipy import optimize


def imshow(img, title=None):
    '''
    :param img: a tensor of img
    :return: show a image
    '''
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
    plt.pause(0.001)


def train(model, CostFunction, optimizer):
    model.train()
    torch.set_grad_enabled(True)
    for inputs, labels in DataLoader['train']:
        inputs = inputs.to(device)
        labels = labels.to(device)
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = CostFunction(outputs, labels)
        loss.backward()
        optimizer.step()
    return model


def test(model, CostFunction):
    model.eval()
    torch.set_grad_enabled(False)
    cnt = 0
    loss_sum = 0.0
    for inputs, labels in DataLoader['test']:
        inputs = inputs.to(device)
        labels = labels.to(device)
        outputs = model(inputs)
        # preds_index 储存outputs中每一行最大值的index
        max_, preds_index = torch.max(outputs, 1)
        loss = CostFunction(outputs, labels)
        loss_sum += loss.cpu().item()

        # labels == preds_index 返回一个tensor
        # labels == tensor([5, 3 ,2])
        # preds_index == tensor([3, 3, 2])
        # labels == preds_index -> tensor([0, 1, 1])
        # int可以和只有一个元素的tensor相加
        cnt += torch.sum(labels == preds_index)
    now_acc = 1.0 * cnt.item() / DataSetSize['test']
    return now_acc, loss_sum


def training_model(model, CostFunction, optimizer, scheduler, num_epochs=40):
    best_acc = 0.0
    min_loss = 100
    best_model_state_dict = copy.deepcopy(model.state_dict())

    for epoch in range(num_epochs):
        scheduler.step()
        print("Epoch {}/{}".format(epoch + 1, num_epochs))
        print('-' * 60)
        model = train(model, CostFunction, optimizer)
        now_acc, loss = test(model, CostFunction)

        if min_loss > loss:
            best_acc = now_acc
            min_loss = loss
            best_model_state_dict = copy.deepcopy(model.state_dict())

        print("Now Test accuracy = {:.4f}%".format(now_acc * 100))
        print("Now Loss = {}".format(loss))

    print('*' * 60)
    print("Training Complete")
    print("Best accuracy = {}%".format(best_acc * 100))
    model.load_state_dict(best_model_state_dict)
    torch.save(model.state_dict(),
               'C:\\Users\QinJingChang\PycharmProjects\zkcup\Save model\\' + 'loss=' + str(min_loss)[:5] + '.tar')
    return model


if __name__ == '__main__':
    plt.ion()
    # transform define
    transform = {
        'train': transforms.Compose([
            transforms.Resize(256),
            transforms.RandomResizedCrop(224),
            transforms.RandomHorizontalFlip(),
            transforms.RandomVerticalFlip(),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])]),

        'test': transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])
    }

    data_dir = r'C:\Users\QinJingChang\PycharmProjects\zkcup\data'
    DataSet = {x: datasets.ImageFolder(os.path.join(data_dir, x), transform=transform[x]) for x in ['train', 'test']}

    # DataLoader['train'] is a dataloader of train set
    # DataLoader['test'] is a dataloader of test set
    DataLoader = {x: torch.utils.data.DataLoader(DataSet[x], batch_size=4, shuffle=True, num_workers=4) for x in
                  ['train', 'test']}

    DataSetSize = {x: len(DataSet[x]) for x in ['train', 'test']}
    class_name = DataSet['train'].classes

    print(DataSet['train'].classes)
    print(DataSet['test'].classes)

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    print("device = ", device)

    # ################# a test part ####################
    # for inputs, labels in DataLoader['train']:
    #     inputs = inputs.to(device)
    #     labels = labels.to(device)
    #     # inputs.shape == torch.size([4, 3, 224, 224])
    #     # print(inputs.shape)
    #     # labels 为行向量, 对应一个batch里的四张图片的类别的index, 0->'ants', 1->'bees'
    #     # print(labels)
    #     for j in range(inputs.size(0)):
    #         imshow(inputs[j], title= class_name[labels[j]])
    #     break

    ############## model init #######################

    # 拆掉最后一层分类器，装新的分类器
    model = models.resnet18(pretrained=True)
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, len(class_name))


    CostFunction = nn.CrossEntropyLoss()
    optimizer = optim.SGD(model.parameters(), lr= 0.001, momentum= 0.9)
    scheduler = lr_scheduler.StepLR(optimizer, step_size= 7, gamma= 0.1)

    ################ training and save model ###################
    model = model.to(device)
    model = training_model(model, CostFunction, optimizer, scheduler, num_epochs=14)




    # 训练完成后的结果测试
    # model.load_state_dict(torch.load('C:\\Users\QinJingChang\PycharmProjects\zkcup\Save model\loss=0.035.tar'))
    # model = model.to(device)
    # acc, loss = test(model,CostFunction=CostFunction)
    # print("acc = {}, loss = {}".format(acc, loss))



    # 用于训练完成后结果显示
    # for inputs, labels in DataLoader['test']:
    #     inputs = inputs.to(device)
    #     labels = labels.to(device)
    #
    #     outputs = model(inputs)
    #     _, preds = torch.max(outputs, 1)
    #     for j in range(inputs.size(0)):
    #         ax = plt.subplot(2, 2, j + 1)
    #         ax.axis("off")
    #         ax.set_title("predict:{}".format(class_name[preds[j]]))
    #         plt.tight_layout()
    #         imshow(inputs.cpu()[j])
    #     plt.ioff()
    #     plt.show()
    #     break
