#!/usr/bin/env python3

import os
import os.path as osp

import rospy
import rospkg

from spinup.utils.run_utils import setup_logger_kwargs
from spinup.utils.logx import EpochLogger
from std_msgs.msg import Float64MultiArray

class PlannerLogger():
    def __init__(self, info_list, logger_kwargs=dict()):
        self.planner_logger =  EpochLogger(**logger_kwargs)
        self.planner_logger.save_config(locals())
        self.info_list = info_list
        rospy.Subscriber("/planning_info", Float64MultiArray, self._info_callback)
    
    def _info_callback(self, info):
        loops = info.data[0]
        prediction = info.data[1]
        trackerror = info.data[2]
        self.planner_logger.log_tabular('loops', loops)
        self.planner_logger.log_tabular('prediction', prediction)
        self.planner_logger.log_tabular('trackerror', trackerror)
        self.planner_logger.log_tabular('treesize', info.data[3])
        self.planner_logger.log_tabular('prunetreesize', info.data[4])
        self.planner_logger.dump_tabular()

if __name__ == '__main__':
    rospy.init_node('planner_info_logger', anonymous=True, log_level=rospy.WARN)    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp_name', type=str, default='cl_rrt_star')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    info_list = ['loop', 'prediction', 'trackerror', 'treesize', 'prunetreesize']
    DEFAULT_DATA_DIR = osp.join(osp.abspath(osp.dirname(osp.dirname(__file__))),'data')
    logger_kwargs = setup_logger_kwargs(args.exp_name, args.seed, data_dir = DEFAULT_DATA_DIR)
    logger = PlannerLogger(info_list=info_list, logger_kwargs=logger_kwargs)
    rospy.spin()
