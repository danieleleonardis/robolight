#!/usr/bin/env python
# coding: utf-8
import numpy as np

d = 0.236
inv_d = 1/d
r_w = 0.1
gear_ratio = 1
rads2rpm=30/np.pi
driver_constant = rads2rpm * gear_ratio * 2**15/12000
encoder_constant=2*np.pi/4096/gear_ratio
encoder_counter_maxvalue=65536

jacobian = np.array([[1, 0.236],[1, -0.236]], np.float32) / r_w

inv_jacobian = np.array([[     1,      1],
                         [ inv_d, -inv_d]], np.float32) * r_w / 2









