import numpy as np
import array


class radpoint(object):
    def __init__(self, x, y, radius, orientation, id, marker_id, type, target):
        self.x = x
        self.y = y
        self.radius = radius
        self.orientation = orientation
        self.id = id
        self.marker_id = marker_id
        self.type = type
        self.target = target

    def __str__(self):
        return f'x: {self.x}, y: {self.y}, radius: {self.radius}, orientation: {self.orientation}, id: {self.id}, marker_id: {self.marker_id}, type: {self.type}, target: {self.target}'


class test_objekt():
    def __init__(self):
        self.object_list = {}
        self.start_list()

    def add_object(self, new_object):
        found = False
        try:
            if self.object_list[new_object.type] == {}:
                new_object.id = new_object.type + str(0)
                self.object_list[new_object.type][new_object.id] = new_object
                found = True
            else:
                for k in self.object_list[new_object.type]:
                    if self.check_dist(self.object_list[new_object.type][k].x, new_object.x, self.object_list[new_object.type][k].y, new_object.y) < 0.5:
                        found = True
                        new_object.id = k
                        print('Object already in list')
                        if new_object.target:
                            #if object is marked as target overwrite everything since it hs more info, i.e marker_id and orientation exists
                            self.object_list[new_object.type][k] = new_object
                        else:
                            self.object_list[new_object.type][k].x = new_object.x
                            self.object_list[new_object.type][k].y = new_object.y
                        return
                if not found:
                    new_object.id = new_object.type + \
                        str(len(self.object_list[new_object.type]))
                    self.object_list[new_object.type][new_object.id] = new_object
        except (KeyError):
            print('Not an acceptabel type of object')

    def check_dist(self, x_old, x_new, y_old, y_new):
        dist = np.sqrt((x_old - x_new)**2 + (y_old - y_new)**2)
        return dist

    def start_list(self):
        list = ['red cube', 'red ball', 'blue cube', 'blue ball', 'green cube',
                'green ball', 'wooden cube', 'kiki', 'bobo', 'momo', 'dede', 'box']
        for shit in list:
            self.object_list[shit] = {}


if __name__ == '__main__':
    test = test_objekt()
    red_cube = radpoint(1, 1, 0.1, 0.5, None, None, 'red cube', True)
    red_cube_2 = radpoint(1.2, 1.1, 0.1, 0.5, None, None, 'red cube', True)
    red_cube_3 = radpoint(3, 3, 0.1, 0.5, None, None, 'red cube', True)
    none = radpoint(0.0, 0.0, 0.2, 0.6, None, None, None, False)
    box = radpoint(5 ,5 ,0.50 , np.pi, None, None, 'box', False)
    box_2 = radpoint(10 ,10 ,0.50 , np.pi, None, 2, 'box', True)
    box_3 = radpoint(5.4 ,5.4 ,0.50 , np.pi, None, 1, 'box', True)
    test.add_object(red_cube)
    test.add_object(red_cube_2)
    test.add_object(red_cube_3)
    test.add_object(none)
    test.add_object(box)
    test.add_object(box_2)
    test.add_object(box_3)

    temp_list = []
    for k in test.object_list:
        for i in test.object_list[k]:
            temp_list.append(test.object_list[k][i])
    for shit in temp_list:
        print(shit)
