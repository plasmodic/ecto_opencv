#!/usr/bin/env python
"""
Module defining a function that returns the appropriate ecto cells for Feature and Descriptor finding
"""

import ecto
from ecto_opencv import features2d

class FeatureDescriptor(ecto.BlackBox):
    """
    Function that takes JSON parameters for Feature/Descriptor extraction and that returns the appropriate blackbox
    combining the two (or just using one cell in the background)
    """
    ORB_combination = features2d.ORB
    SIFT_combination = features2d.SIFT

    def declare_params(self, p):
        p.declare('json_feature_params', 'Parameters for the feature as a JSON string. '
                  'It should have the format: "{"type":"ORB/SIFT whatever", "opencv_param_1":val1, ....}', '{"type":"ORB"}')
        p.declare('json_descriptor_params', 'Parameters for the descriptor as a JSON string. '
                  'It should have the format: "{"type":"ORB/SIFT whatever", "opencv_param_1":val1, ....}', '{"type":"ORB"}')

    def declare_io(self, p, i, o):
        types = []
        for dict_key, p_val, desc in [ ('_feature_params', p.json_feature_params, 'features'),
                                      ('_descriptor_params', p.json_descriptor_params, 'descriptors') ]:
            try:
                self.__dict__[dict_key] = eval(p_val)
            except:
                raise RuntimeError('Invalid JSON for the ' + desc + ': ' + p.json_feature_params)
            try:
                types.append(self.__dict__[dict_key].pop('type'))
            except:
                raise RuntimeError('No "type" given for the ' + desc)

        feature_type = types[0]
        descriptor_type = types[1]
        if feature_type == 'ORB' and descriptor_type == 'ORB':
            self._feature_name = 'ORB_combination'
            self._descriptor_name = 'ORB_combination'
        else:
            raise RuntimeError('parameters not supported for FeatureDescriptor')

        i.forward('image', cell_name = self._feature_name, cell_key = 'image')
        i.forward('mask', cell_name = self._feature_name, cell_key = 'mask')

        if self._feature_name != self._descriptor_name:
            i.forward('image', cell_name = self._descriptor_name, cell_key = 'image')
            o.forward('keypoints', cell_name = self._feature_name, cell_key = 'keypoints')
        else:
            o.forward('keypoints', cell_name = self._descriptor_name, cell_key = 'keypoints')

        o.forward('descriptors', cell_name = self._descriptor_name, cell_key = 'descriptors')

    def configure(self, p, i, o):
        # Deal with the combinations first
        if self._feature_name == 'ORB_combination':
            #print self._feature_descriptor_params
            self._feature_cell = self.ORB_combination(**self._feature_params)

    def connections(self):
        if self._feature_name in [ 'ORB_combination', 'SIFT_combination' ]:
            return []
        else:
            return [ self._feature_cell['keypoints'] >> self._descriptor_cell['keypoints'] ]
