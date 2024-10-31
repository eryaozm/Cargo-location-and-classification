#include "Dispatch.h"
#include <iostream>
#include <chrono>
#include "SimObject.h"
#include "Macro.h"
static std::mutex lock;

// 计算两个坐标之间的距离
double calculateDistance(const std::vector<double>& pos1, const std::vector<double>& pos2) {
    return std::sqrt(std::pow(pos1[0] - pos2[0], 2) +
                     std::pow(pos1[1] - pos2[1], 2) +
                     std::pow(pos1[2] - pos2[2], 2));
}

void Dispatch(Cluster* cluster_array,bool* act,Position* position,std::string type,RemoteAPIObject::sim sim)
{
    // 大车移动电机句柄
    int RightJointHandle = sim.getObject("/RightJoint");
    // 小车移动电机句柄
    int BeamJointRightHandle = sim.getObject("/BeamJointRight");
    // 钩子移动电机句柄
    int CarJointHandle = sim.getObject("/CarJoint");
    // 钩子旋转电机句柄
    int RevoluteJointHandle = sim.getObject("/RevoluteJoint");
    // 钩子锚点句柄
    int HookForceSensorHandle = sim.getObject("/HookForceSensor");

    std::vector<SimObject> ShapeHandles;
    // 获取所有形状的对象用于将Cluster和对象相互匹配
    for(int index=0,ret=0;ret!=-1;index++)
    {
        ret = sim.getObjects(index, sim.object_shape_type);
        if(ret!=-1)
        {
            SimObject handle = SimObject();
            handle.handle = ret;
            std::vector<double> pos = sim.getObjectPosition(ret,sim.handle_world);;
            handle.position.swap(pos);
            handle.path = sim.getObjectAlias(ret,2); // options = 2:获取完整路径
            ShapeHandles.push_back(handle);
        }
    }
    // 依次处理所有的Cluster
    for(int i=0;i<cluster_array[0].num;)
    {
        if(*act)
        {
            if(!(cluster_array[i].type_name == type || type == "all"))
            {
                i++;
                if(i<cluster_array[0].num)
                    continue;
                else
                {
                    // 时间更新
                    double t0 = sim.getSimulationTime();
                    double JointX = sim.getJointPosition(RightJointHandle);
                    double JointY = sim.getJointPosition(BeamJointRightHandle);
                    while(true)
                    {
                        double dt = sim.getSimulationTime() - t0;
                        double posX = JointX - VelZ * dt;
                        double posY = JointY - VelZ * dt;
                        bool ActX = posX > 0.05;
                        bool ActY = posY > 0.05;
                        if(ActX)
                            sim.setJointPosition(RightJointHandle,posX);
                        if(ActY)
                            sim.setJointPosition(BeamJointRightHandle,posY);
                        if(!ActX && !ActY)
                        {
                            sim.setJointPosition(RightJointHandle,0);
                            sim.setJointPosition(BeamJointRightHandle,0);
                            sim.setJointPosition(CarJointHandle,0);
                            std::cout << "Hoisting completed & Crane Reset." << std::endl;
                            *act = false;
                            break;
                        }
                        sim.step();
                    }
                    // 仿真结束
                    sim.stopSimulation();
                    continue;
                }

            }
            // 获取Cluster中心坐标
            std::vector<double> ClusterPos;
            ClusterPos.reserve(3);
            ClusterPos.push_back(cluster_array[i].centroid.x);
            ClusterPos.push_back(cluster_array[i].centroid.y);
            ClusterPos.push_back(cluster_array[i].centroid.z);
            // 寻找当前Cluster匹配的形状对象
            SimObject ClusterHandle;
            double closestDistance = std::numeric_limits<double>::max();
            for (int i = 0; i < ShapeHandles.size(); i++) {
                std::vector<double> CurrentShapePosition = ShapeHandles[i].position;
                double distance = calculateDistance(ClusterPos, CurrentShapePosition);
                if (distance < closestDistance) {
                    closestDistance = distance;
                    ClusterHandle = ShapeHandles[i];
                }
                std::cout << sim.getObjectAlias(ShapeHandles[i].handle) << std::endl;
            }
            std::cout << "Cluster " << i << "'s name is: " << sim.getObjectAlias(ClusterHandle.handle,2) << std::endl;
            // 获取钩子的当前位置
            std::vector<double> CurrentHookPosition = sim.getObjectPosition(HookForceSensorHandle,sim.handle_world);
            // Cluster的Dummy位置减去当前钩子关节的位置得到关节应当移动的距离
            double MoveX = cluster_array[i].centroid.x - CurrentHookPosition[0];
            double MoveY = cluster_array[i].centroid.y - CurrentHookPosition[1];
            double MoveZ = CurrentHookPosition[2] - cluster_array[i].highest_point.z; //钩子下降方向与Z轴正方向相反
            std::cout << "Move:" << std::endl;
            std::cout << "X:" << MoveX << "\n" << "Y:" << MoveY << "\n" << "Z:" << MoveZ << std::endl;
            // 当前各关节的伸长量
            double JointX = sim.getJointPosition(RightJointHandle);
            double JointY = sim.getJointPosition(BeamJointRightHandle);
            double JointZ = sim.getJointPosition(CarJointHandle);
            std::cout << "Current Joint Pos:" << std::endl;
            std::cout << "X:" << JointX << "\n" << "Y:" << JointY << "\n" << "Z:" << JointZ << std::endl;
            // 记录伸长量正负
            int FlagX = MoveX > 0?1:-1;
            int FlagY = MoveY > 0?1:-1;
            int FlagZ = MoveZ > 0?1:-1;
            // 各关节的目标长度
            double PosX = JointX + MoveX;
            double PosY = JointY + MoveY;
            double PosZ = JointZ + MoveZ;
            std::cout << "Target Joint Pos:" << std::endl;
            std::cout << "X:" << PosX << "\n" << "Y:" << PosY << "\n" << "Z:" << PosZ << std::endl;
            //获取初始时间
            double t0 = sim.getSimulationTime();
            //设置初始速度

            // XY动到达Cluster上方
            while(true)
            {
                double dt = sim.getSimulationTime() - t0;
                // 当前时刻关节位置 = 初始位置 + 移动方向*速度*仿真时间
                double posX = JointX + FlagX * VelX * dt;
                double posY = JointY + FlagY * VelY * dt;
                // XY运动标志位，正向移动时当前位置小于目标位置或者负向移动当前位置大于目标位置时为true
                bool ActX = (FlagX > 0 && posX < PosX) || (FlagX < 0 && posX > PosX);
                bool ActY = (FlagY > 0 && posY < PosY) || (FlagY < 0 && posY > PosY);
                // X动
                if(ActX)
                    sim.setJointPosition(RightJointHandle, posX);
                // Y动
                if(ActY)
                    sim.setJointPosition(BeamJointRightHandle, posY);
                if(!ActX && !ActY)
                {
                    // 更新运动后关节的长度
                    JointX = sim.getJointPosition(RightJointHandle);
                    JointY = sim.getJointPosition(BeamJointRightHandle);
                    break;
                }
                sim.step();
            }
            // 时间更新
            t0 = sim.getSimulationTime();
            // Z下降，钩子与物体建立连接
            while(true)
             {
                double dt = sim.getSimulationTime() - t0;
                double posZ = JointZ + FlagZ * VelZ * dt;
                // Z运动标志位，正向移动时当前位置小于目标位置或者负向移动当前位置大于目标位置时为true
                bool ActZ = (FlagZ > 0 && posZ < PosZ) || (FlagZ < 0 && posZ > PosZ);
                // Z动，完成后钩子和物体建立连接
                if(ActZ)
                    sim.setJointPosition(CarJointHandle,posZ);
                else
                {
                    sim.setObjectParent(ClusterHandle.handle, HookForceSensorHandle, true);
                    // 更新运动后关节的长度
                    JointZ = sim.getJointPosition(CarJointHandle);
                    break;
                }
                sim.step();
            }
            // 时间戳更新
            t0 = sim.getSimulationTime();
            // Z上升，吊起物体
            while(true)
            {
                double dt = sim.getSimulationTime() - t0;
                double posZ = JointZ - VelZ * dt;
                bool ActZ = posZ >= 0.05;
                if(ActZ)
                    sim.setJointPosition(CarJointHandle,posZ);
                else
                {
                    // Z归零
                    sim.setJointPosition(CarJointHandle,0);
                    JointZ = 0;
                    break;
                }
                sim.step();
            }
            // 更新当前钩子位置
            CurrentHookPosition = sim.getObjectPosition(HookForceSensorHandle,sim.handle_world);
            // 伸长量更新，指向目标位置
            MoveX = position->x - CurrentHookPosition[0];
            MoveY = position->y - CurrentHookPosition[1];
            MoveZ = CurrentHookPosition[2] - cluster_array[i].highest_point.z;
            std::cout << "Target Move:" << std::endl;
            std::cout << "X:" << MoveX << "\n" << "Y:" << MoveY << "\n" << "Z:" << MoveZ << std::endl;
            std::cout << "Current Joint Pos:" << std::endl;
            std::cout << "X:" << JointX << "\n" << "Y:" << JointY << "\n" << "Z:" << JointZ << std::endl;
            // 标志位更新
            FlagX = MoveX > 0?1:-1;
            FlagY = MoveY > 0?1:-1;
            FlagZ = MoveZ > 0?1:-1;
            // 目标长度更新
            PosX = JointX + MoveX;
            PosY = JointY + MoveY;
            PosZ = JointZ + MoveZ; // 物理引擎限制，防止穿模
            std::cout << "Target Joint Pos:" << std::endl;
            std::cout << "X:" << PosX << "\n" << "Y:" << PosY << "\n" << "Z:" << PosZ << std::endl;

            // 时间更新
            t0 = sim.getSimulationTime();
            // XY到目标上方
            while(true)
            {
                double dt = sim.getSimulationTime() - t0;
                // 当前时刻关节位置 = 初始位置 + 移动方向*速度*仿真时间
                double posX = JointX + FlagX * VelX * dt;
                double posY = JointY + FlagY * VelY * dt;
                // XY运动标志位，正向移动时当前位置小于目标位置或者负向移动当前位置大于目标位置时为true
                bool ActX = (FlagX > 0 && posX < PosX) || (FlagX < 0 && posX > PosX);
                bool ActY = (FlagY > 0 && posY < PosY) || (FlagY < 0 && posY > PosY);
                // X动
                if(ActX)
                    sim.setJointPosition(RightJointHandle, posX);
                // Y动
                if(ActY)
                    sim.setJointPosition(BeamJointRightHandle, posY);
                if(!ActX && !ActY)
                {
                    // 更新运动后关节的长度
                    JointX = sim.getJointPosition(RightJointHandle);
                    JointY = sim.getJointPosition(BeamJointRightHandle);
                    break;
                }
                sim.step();
            }
            // 时间更新
            t0 = sim.getSimulationTime();
            while(true)
            {
                double dt = sim.getSimulationTime() - t0;
                double posZ = JointZ + FlagZ * VelZ * dt;
                // Z运动标志位，正向移动时当前位置小于目标位置或者负向移动当前位置大于目标位置时为true
                bool ActZ = (FlagZ > 0 && posZ < PosZ) || (FlagX < 0 && posZ > PosZ);
                // Z动，完成后钩子和物体脱离
                if(ActZ)
                    sim.setJointPosition(CarJointHandle,posZ);
                else
                {
                    // 将物体从钩子上释放
                    sim.setObjectParent(ClusterHandle.handle, -1, true);
                    // 更新运动后关节的长度
                    JointZ = sim.getJointPosition(CarJointHandle);
                    break;
                }
                sim.step(); // 步进
            }
            // 时间戳更新
            t0 = sim.getSimulationTime();
            // Z再次上升复位
            while(true)
            {
                double dt = sim.getSimulationTime() - t0;
                double posZ = JointZ - VelZ * dt;
                bool ActZ = posZ >= 0.05;
                if(ActZ)
                    sim.setJointPosition(CarJointHandle,posZ);
                else
                {
                    // 老登滚蛋
                    std::vector<double> POSITION = {20,20,-20};
                    sim.setObjectPosition(ClusterHandle.handle, POSITION);
                    // Z归零
                    sim.setJointPosition(CarJointHandle,0);
                    JointZ = 0;
                    break;
                }
                sim.step();
            }
            i++;
            // 所有吊装完成后行车复位
            if(i == cluster_array[0].num)
            {
                // 时间更新
                t0 = sim.getSimulationTime();
                while(true)
                {
                    double dt = sim.getSimulationTime() - t0;
                    double posX = JointX - VelZ * dt;
                    double posY = JointY - VelZ * dt;
                    bool ActX = posX > 0.05;
                    bool ActY = posY > 0.05;
                    if(ActX)
                        sim.setJointPosition(RightJointHandle,posX);
                    if(ActY)
                        sim.setJointPosition(BeamJointRightHandle,posY);
                    if(!ActX && !ActY)
                    {
                        sim.setJointPosition(RightJointHandle,0);
                        sim.setJointPosition(BeamJointRightHandle,0);
                        sim.setJointPosition(CarJointHandle,0);
                        std::cout << "Hoisting completed & Crane Reset." << std::endl;
                        *act = false;
                        break;
                    }
                    sim.step();
                }
                // 仿真结束
                sim.stopSimulation();
            }
            // *act = false;
        }
        else
        {
            // 线程休眠1ms
            std::this_thread::sleep_for(std::chrono::nanoseconds(1000));
        }
    }
}