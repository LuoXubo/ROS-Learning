#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   getGT.py
@Time    :   2023/07/24 12:15:10
@Author  :   Xubo Luo 
@Version :   1.0
@Contact :   luoxubo@hotmail.com
@License :   (C)Copyright 2017-2018, Liugroup-NLPR-CASIA
@Desc    :   读师姐的.csv文件，格式化输出为可以做evo精度评定的文件
'''

# here put the import lib

# input file
with open('./gt.csv', 'r') as f:
    lines = f.readlines()[1:]
f.close()

# output file
with open('./gt.txt', 'w+') as f:
    for line in lines:
        items = line.split(',')
        f.write('%s %s %s %s %s %s %s %s\n'%(items[0][:10]+'.'+items[0][10:], items[1], items[2], items[3], items[4], items[5], items[6], items[7]))
f.close()