#! /usr/bin/env python

import math


def create_path(x,y):
        import pandas as pd
        read_path = pd.read_csv('/home/car/gencoupe/src/path_planning/scripts/path.csv', keep_default_na=False)
        car = [x, y]
        path = []
        dist = []
        new_path = []

        print('Creating New Path...')

        for i in range(len(read_path['X'])):
            path.append([read_path['X'][i],read_path['Y'][i]])
            dist.append(math.sqrt(( car[0] - read_path['X'][i] ) ** 2 + ( car[1] - read_path['Y'][i] ) ** 2 ))

        min_dist = dist.index(min(dist))+1

        for i in range(min_dist, len(read_path['X'])):
            new_path.append([read_path['X'][i],read_path['Y'][i]])

        print('Save New Path...')
        new_CSV = pd.DataFrame(new_path, columns=['X','Y'])
        new_CSV.to_csv('/home/car/gencoupe/src/path_planning/scripts/path.csv',index=None)

        return new_path