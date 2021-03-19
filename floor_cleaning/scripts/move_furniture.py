#! /usr/bin/env python
from utilities import set_model_state
from geometry_msgs.msg import Pose, Point

items = ['cabinet', 'cabinet_0', 'cabinet_1', 'cabinet_2', 'cabinet_3', 
         'cabinet_4', 'table', 'table_0', 'table_1', 'bookshelf',
         'bookshelf_1', 'bookshelf_2', 'bookshelf_3', 'bookshelf_4',
         'bookshelf_5', 'bookshelf_6', 'bookshelf_0']
newpos = [(4.5,1.5,0), (4.5,-4.5,0), (-4.5,-3.5,0), (-4.5,-4.5,0), (4.5,-2.5,0), 
          (-4.5,-2.5,0), (-3,-3.6,0), (4,-3.6,0), (-1,-12.5,0), (1,-12.5,0),
          (-4.5,1.8,0), (-4.5,-12.5,0), (2.25,-12.5,0), (3.5,-12.5,0),
          (-3,-12.5,0), (-1,-3.2,0), (-3.5, -2.25, 0)]

for item, pos in zip(items, newpos):
    x, y, z = pos
    set_model_state(item, Pose(position=Point(x,y,z)))