
import os
import numpy as np
import warnings
from torch.utils.data import Dataset

warnings.filterwarnings('ignore')
def pc_normalize(pc):
    centroid = np.mean(pc, axis=0)
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc ** 2, axis=1)))
    pc = pc / m
    return pc

def farthest_point_sample(point, npoint):
    N, D = point.shape
    xyz = point[:, :3]
    centroids = np.zeros((npoint,))
    distance = np.ones((N,)) * 1e10
    farthest = np.random.randint(0, N)
    for i in range(npoint):
        centroids[i] = farthest
        centroid = xyz[farthest, :]
        dist = np.sum((xyz - centroid) ** 2, -1)
        mask = dist < distance
        distance[mask] = dist[mask]
        farthest = np.argmax(distance, -1)
    point = point[centroids.astype(np.int32)]
    return point

class DataLoader(Dataset):
    def __init__(self, root):
        self.root = root
        self.npoints = 1024
        self.process_data = True
        self.uniform = True
        self.use_normals = False
        self.num_category = 3
        self.catfile = r'C:/Users/eryao/Documents/PycharmProjects/PointClassification/data/shape_names.txt'
        self.cat = [line.rstrip() for line in open(self.catfile)]
        self.classes = dict(zip(self.cat, range(len(self.cat))))
        # 读取文件
        self.datapath = [os.path.join(root, file) for file in os.listdir(root) if os.path.isfile(os.path.join(root, file))]
        # print('The size of actual data is %d' % (len(self.datapath)))

    def __len__(self):
        return len(self.datapath)

    # 数据集采样npoints个点送入网络
    def _get_item(self, index):
        fn = self.datapath[index]
        point_set = np.loadtxt(fn, delimiter=' ').astype(np.float32)
        if self.uniform:
            # 使用最远点采样方法
            point_set = farthest_point_sample(point_set, self.npoints)
        else:
            # 否则取前npoints个点，数据集组织应有随机性
            point_set = point_set[0:self.npoints, :]
        point_set[:, 0:3] = pc_normalize(point_set[:, 0:3])
        if not self.use_normals:
            point_set = point_set[:, 0:3]
        return point_set,fn

    def __getitem__(self, index):
        return self._get_item(index)

if __name__ == '__main__':
    import torch
    data = DataLoader('C:/Users/eryao/Documents/PycharmProjects/PointClassification/actual_data/')
    DataLoader = torch.utils.data.DataLoader(data, batch_size=1, shuffle=True)
    for point  in DataLoader:
        print(point.shape)
