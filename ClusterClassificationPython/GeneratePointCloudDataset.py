import os
import numpy as np
import random

def get_files_in_directory(directory_path):
    # 初始化一个空列表保存文件路径
    file_list = []

    # 使用os.listdir遍历目录下的所有文件和文件夹
    for filename in os.listdir(directory_path):
        # 构建完整的文件路径
        full_path = os.path.join(directory_path, filename)

        # 如果是文件则添加到列表中
        if os.path.isfile(full_path):
            file_list.append(full_path)

    return file_list

def trans(raw_file_dir,current_dir,type_name,train_flag,list_file):
    if not os.path.exists(current_dir):
        os.mkdir(current_dir)
    if train_flag:
        r = 200
        print('Generate new train '+ type_name +' point clouds.')
    else:
        r = 60
        print('Generate new test ' + type_name + ' point clouds.')

    file_list = get_files_in_directory(raw_file_dir)

    for i in range(0,r):
        if (i+1) % 10 == 0 : print(i)
        for j, file in enumerate(file_list):
            # 偏移参数
            x_offset = random.uniform(-10, 10)
            y_offset = random.uniform(-10, 10)
            z_offset = random.uniform(-10, 10)

            # 缩放参数
            scale = random.uniform(0.5, 2.0)

            # 旋转角度
            roate_x = random.uniform(-np.pi / 10, np.pi / 10)
            roate_y = random.uniform(-np.pi / 10, np.pi / 10)
            roate_z = random.uniform(-np.pi / 10, np.pi / 10)

            roate_x_matrix = np.array([
                [1, 0, 0, 0],
                [0, np.cos(roate_x), -np.sin(roate_x), 0],
                [0, np.sin(roate_x), np.cos(roate_x), 0],
                [0, 0, 0, 1]
            ])
            roate_y_matrix = np.array([
                [np.cos(roate_y), 0, np.sin(roate_y), 0],
                [0, 1, 0, 0],
                [-np.sin(roate_y), 0, np.cos(roate_y), 0],
                [0, 0, 0, 1]
            ])
            roate_z_matrix = np.array([
                [np.cos(roate_z), -np.sin(roate_z), 0, 0],
                [np.sin(roate_z), np.cos(roate_z), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            # 变换矩阵
            transformation_matrix = np.array([
                [scale, 0, 0, x_offset],
                [0, scale, 0, y_offset],
                [0, 0, scale, z_offset],
                [0, 0, 0, 1]
            ]).dot(roate_z_matrix).dot(roate_y_matrix).dot(roate_x_matrix)

            #加载文件
            raw_array=np.loadtxt(file)
            raw_xyz=raw_array[:,:3]

            #补充数据为齐次项
            ones_data=np.ones(raw_xyz.shape[0])
            raw_xyz=np.insert(raw_xyz,3,values=ones_data,axis=1)

            #变换数据
            new_xyz = np.dot(transformation_matrix,raw_xyz.T)
            new_array=np.concatenate((new_xyz.T[:,:3],raw_array[:,3:]),axis=1)
            new_file_name = type_name + '_' + str(i) + '_' + str(j) + '.txt'
            new_file = os.path.join(current_dir,new_file_name)
            np.savetxt(new_file,new_array,fmt='%.06f')
            list_file.write(type_name + '_' + str(i) + '_' + str(j) +'\n')


if __name__ == '__main__':
    cone_dir = r'data/raw_data/cone/'
    cuboid_dir = r'data/raw_data/cuboid/'
    cylinder_dir = r'data/raw_data/cylinder/'
    train_data_dir = 'data/train/'
    test_data_dir = 'data/test/'

    train_list_file = open('data/train/train_file.txt','w')
    test_list_file = open('data/test/test_file.txt','w')
    trans(cone_dir, train_data_dir + 'cone/', 'cone',True,train_list_file)
    trans(cuboid_dir, train_data_dir + 'cuboid/', 'cuboid',True,train_list_file)
    trans(cylinder_dir, train_data_dir + 'cylinder/', 'cylinder',True,train_list_file)
    trans(cone_dir, test_data_dir + 'cone/', 'cone',False,test_list_file)
    trans(cuboid_dir, test_data_dir + 'cuboid/', 'cuboid', False,test_list_file)
    trans(cylinder_dir, test_data_dir + 'cylinder/', 'cylinder', False,test_list_file)
    train_list_file.close()
    test_list_file.close()
    print('done.')


