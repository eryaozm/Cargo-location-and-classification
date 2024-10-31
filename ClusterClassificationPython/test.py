from ActualDataLoader import DataLoader
import os
import torch
import logging
import sys
import datetime
from pathlib import Path
from Model import get_model
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = BASE_DIR
sys.path.append(os.path.join(ROOT_DIR, 'models'))

def main():

    '''选定GPU'''
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"

    # '''创建目录'''
    # timestr = str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M'))
    # experiment_dir = Path('test_log/classification/')
    # experiment_dir.mkdir(exist_ok=True)
    # experiment_dir = experiment_dir.joinpath(timestr)
    # experiment_dir.mkdir(exist_ok=True)
    #
    # '''日志'''
    # logger = logging.getLogger("Model")
    # logger.setLevel(logging.INFO)
    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # file_handler = logging.FileHandler('%s/eval.txt' % experiment_dir)
    # file_handler.setLevel(logging.INFO)
    # file_handler.setFormatter(formatter)
    # logger.addHandler(file_handler)

    '''加载数据'''
    data_path = 'C:/Users/eryao/Documents/PycharmProjects/PointClassification/actual_data/data/'
    test_dataset = DataLoader(root=data_path)
    testDataLoader = torch.utils.data.DataLoader(test_dataset, batch_size=1, shuffle=False, num_workers=0)

    '''加载模型'''
    num_class = 3
    classifier = get_model(num_class, normal_channel=False).cuda()
    checkpoint = torch.load('C:/Users/eryao/Documents/PycharmProjects/PointClassification/checkpoints/best_model.pth')
    classifier.load_state_dict(checkpoint['model_state_dict'])

    with open('C:/Users/eryao/Documents/PycharmProjects/PointClassification/data/shape_names.txt', 'r') as file:
        # 使用readlines()将每行内容保存为列表的元素
        lines = file.readlines()
    # 去除每行的换行符
    lines = [line.strip() for line in lines]
    '''预测'''
    result_file =  open("C:/Users/eryao/Documents/PycharmProjects/PointClassification/actual_data/result.txt", "w")

    with torch.no_grad():
        classifier = classifier.eval()
        for j, (points,fn) in enumerate(testDataLoader):
            points = points.cuda()
            points = points.transpose(2, 1)
            pred, _ = classifier(points)
            pred_choice = pred.data.max(1)[1]
            # print(str(fn[0])+':'+lines[pred_choice.cpu().item()])
            result_file.write("{}\n".format(str(pred_choice.cpu().item())))
    result_file.close()

if __name__ == '__main__':
    main()
