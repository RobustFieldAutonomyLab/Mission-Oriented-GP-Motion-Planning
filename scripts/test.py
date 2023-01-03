import GPPlanning
import numpy as np
import matplotlib.pyplot as plt


def planning2D():
    planner = GPPlanning.pyPlanning2D(True, 0.2, 0.001, 0.01, 0.2, 5)
    planner.buildMap(1, (0, 0), 50, 50)

    delta_t = 0.4
    total_time_step = 5
    start_pose = (5, 5, 0)
    start_vel = (0.0, 0.0, 0.0)

    end_pose = (45, 45, 0)
    end_vel = (0.0, 0.0, 0.0)

    avg_vel = (end_pose[0] - start_pose[0]) / total_time_step / delta_t, \
              (end_pose[1] - start_pose[1]) / total_time_step / delta_t, \
              (end_pose[2] - start_pose[2] / total_time_step / delta_t)
    poses = []
    vels = []
    for i in range(0, 6):
        pose = (start_pose[0] * (total_time_step - i) / total_time_step + end_pose[0] * i / total_time_step,
                start_pose[1] * (total_time_step - i) / total_time_step + end_pose[1] * i / total_time_step,
                start_pose[2] * (total_time_step - i) / total_time_step + end_pose[2] * i / total_time_step)
        poses.append(pose)
        vels.append(avg_vel)

    aa = planner.optimize(np.array(poses), np.array(vels), delta_t)
    print(aa[0][0])
    print(aa[0][1])
    print(aa)

def planning3D():
    planner = GPPlanning.pyPlanning3D(0.2, 0.002, 0.2, 5)
    planner.buildMap(1, 1, (0, 0, -15), 50, 50, 15, 5)

    delta_t = 0.4
    total_time_step = 5
    start_pose = (5, 5, 0)
    start_vel = (0.0, 0.0, 0.0)

    end_pose = (45, 45, -10)
    end_vel = (0.0, 0.0, 0.0)

    avg_vel = ((end_pose[0] - start_pose[0]) / total_time_step / delta_t, \
                (end_pose[1] - start_pose[1]) / total_time_step / delta_t, \
                    (end_pose[2] - start_pose[2]) / total_time_step / delta_t)
    poses = []
    vels = []
    for i in range(0, 6):
        pose = (start_pose[0] * (total_time_step - i) / total_time_step + end_pose[0] * i / total_time_step,
                start_pose[1] * (total_time_step - i) / total_time_step + end_pose[1] * i / total_time_step,
                start_pose[2] * (total_time_step - i) / total_time_step + end_pose[2] * i / total_time_step)
        poses.append(pose)
        vels.append(avg_vel)
    for i in range(0, 4):
        pose = (end_pose[0] * (total_time_step - i) / total_time_step + start_pose[0] * i / total_time_step,
                end_pose[1] * (total_time_step - i) / total_time_step + start_pose[1] * i / total_time_step,
                end_pose[2] * (total_time_step - i) / total_time_step + start_pose[2] * i / total_time_step)
        poses.append(pose)
        vels.append(avg_vel)

    aa = planner.optimize(np.array(poses), np.array(vels), delta_t)
    print("velocity:", vels[0])
    for i in range(0, len(aa)):
        print(aa[i])
    poses = np.array(poses)
    aa = np.array(aa)

    plt.plot(poses[:,2],label = "initial")
    plt.plot(aa[:,2], label = "result")
    plt.savefig("1.png")




planning2D()