import numpy as np
from scipy.spatial import KDTree
from scan_match_class import ScanMatch
import time
import random
# source = np.array([[1, 1, 0],
#                   [2, 2, 0],
#                   [3, 3, 0],
#                   [4, 4, 0],
#                   [5, 5, 0],
#                   [6, 6, 0],
#                   [7, 7, 0],
#                   [8, 8, 0],
#                   [9, 9, 0],
#                   [10, 10, 0],
#                   [12, 12, 0],
#                   [15, 15, 0],
#                   [20, 20, 0],
#                   [25, 25, 0]])


def normalize_angle(angle):
    """
    Normalize an angle to be within the range [0, 2*pi].
    """
    return (angle + 2 * np.pi) % (2 * np.pi)


while True:

    source = np.random.uniform(-100, 100, (10000, 3))

    # transform = np.eye(4)
    # transform[0, 3] = 0.3
    # transform[1, 3] = 0.3
    T_x = random.randint(0, 100)
    T_y = random.randint(0, 100)
    T_z = 0
    theta = random.uniform(0, 2*np.pi)
    phi = 0
    # testing with rotation
    R_11 = np.cos(phi)*np.cos(theta)
    R_12 = -np.cos(phi)*np.sin(theta)
    R_13 = np.sin(phi)
    R_21 = (np.sin(phi)*np.sin(phi)*np.cos(theta)+np.cos(phi)*np.sin(theta))
    R_22 = -(np.sin(phi)*np.sin(phi)*np.sin(theta)-np.cos(phi)*np.cos(theta))
    R_23 = np.sin(phi)*np.cos(phi)
    R_31 = 0
    R_32 = 0
    R_33 = 1

    transform = np.array([[R_11, R_12, R_13, T_x],
                          [R_21, R_22, R_23, T_y],
                          [R_31, R_32, R_33, T_z],
                          [0, 0, 0, 1]])
    print('\n', 'oringal tranform', transform)
    print('oringal theta', theta)
    source_r = np.transpose(np.column_stack((source, np.ones(len(source)))))
    # print(source_r)

    target = np.transpose(np.dot(transform, source_r))
    # print(target)
    target_r = np.transpose(target)
    target = target_r[0:3].T
    target = target[0:random.randint(1000, 9000)]
    # print(target)

    # TRansform guess
    T_x = T_x + random.uniform(-10, 10)
    T_y = T_y + random.uniform(-10, 10)
    T_z = 0
    theta = theta + random.uniform(-20*np.pi/180, 20*np.pi/180)
    phi = 0
    # testing with rotation
    R_11 = np.cos(phi)*np.cos(theta)
    R_12 = -np.cos(phi)*np.sin(theta)
    R_13 = np.sin(phi)
    R_21 = (np.sin(phi)*np.sin(phi)*np.cos(theta)+np.cos(phi)*np.sin(theta))
    R_22 = -(np.sin(phi)*np.sin(phi)*np.sin(theta)-np.cos(phi)*np.cos(theta))
    R_23 = np.sin(phi)*np.cos(phi)
    R_31 = 0
    R_32 = 0
    R_33 = 1
    transform_guess = np.array([[R_11, R_12, R_13, T_x],
                                [R_21, R_22, R_23, T_y],
                                [R_31, R_32, R_33, T_z],
                                [0, 0, 0, 1]])

    outliers = np.random.uniform(-100, 100, (100, 3))
    target = np.concatenate((target, outliers), axis=0)

    outliers1 = np.random.uniform(-100, 100, (100, 3))
    source = np.concatenate((source, outliers1), axis=0)

    # print('\n', 'target', target)
    Scan = ScanMatch(source, target, 100, 4, 1e-4, 20)
    t = Scan.icp(transform_guess)

    # t = icp_with_outlier_rejection(source, target)  # , T=transform_guess)

    print('\n', 'Estimated Transform', t)
    theta_NEW = np.arctan2(t[1, 0], t[0, 0])
    theta_new = normalize_angle(theta_NEW)
    print('Theta', theta_new, '\n')
    time.sleep(0.1)
    print('-----------------------------------------------------------------------------------------------------')
    # test = np.array([[1, 2, 0],
    #                [1, 4, 0],
    #                [2, 4, 0],
    #                [5, 5, 0]])
    #
    #
    # test2 = np.mean(test, axis=0)
    # print(test2)
    # tst_2 = test - np.mean(test, axis=0)
    #
    # print(tst_2)
    # print(tst_2.T)

    # YOU EED TO SAVE ALL THE TRANSFORMS SEPRAELY SINCE YOU transform the source cloud with the accumalated transform all the time instead of transofmring it with the single
    # needed transform for the step!!!!!!!
