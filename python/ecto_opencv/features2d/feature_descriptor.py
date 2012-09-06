#!/usr/bin/env python
"""
Module defining a function that returns the appropriate ecto cells for Feature and Descriptor finding
"""

import ecto_opencv.ecto_cells.features2d as features2d
import ecto
import inspect
import pkgutil

def find_cell(modules, cell_name):
    '''
    Given a list of python packages, or modules, find an object of the given name.
    :param module: The module to look the cell into
    :param cell_name: the name of cell to find
    :returns: the object class itself
    '''
    ms = []
    for module in modules:
        if module == '':
            continue

        m = __import__(module)
        ms += [m]
        for loader, module_name, is_pkg in  pkgutil.walk_packages(m.__path__):
            if is_pkg:
                module = loader.find_module(module_name).load_module(module_name)
                ms.append(module)

    for pymodule in ms:
        for name, potential_cell in inspect.getmembers(pymodule):
            if name==cell_name:
                return potential_cell

    return None

class FeatureDescriptor(ecto.BlackBox):
    """
    Function that takes JSON parameters for Feature/Descriptor extraction and that returns the appropriate blackbox
    combining the two (or just using one cell in the background).
    Both the Feature and the Descriptor will be computed
    """
    ORB_combination = features2d.ORB
    SIFT_combination = features2d.SIFT
    _image_passthrough = ecto.Passthrough
    _mask_passthrough = ecto.Passthrough

#    def __init__(self, **kwargs):
#        ecto.BlackBox.__init__(self, **kwargs)
#        self._feature_cell = None
#        self._descriptor_cell = None
#        self._feature_descriptor_cell = None

    def declare_params(self, p):
        p.declare('json_feature_params', 'Parameters for the feature as a JSON string. '
                  'It should have the format: "{"type":"ORB/SIFT whatever", "opencv_param_1":val1, ....}',
                  '{"type": "ORB", "package": "ecto_opencv"}')
        p.declare('json_descriptor_params', 'Parameters for the descriptor as a JSON string. '
                  'It should have the format: "{"type":"ORB/SIFT whatever", "opencv_param_1":val1, ....}',
                  '{"type": "ORB", "package": "ecto_opencv"}')

    def declare_io(self, p, i, o):
        # Make sure the parameters are valid
        for dict_key, p_val, desc in [ ('_feature_params', p.json_feature_params, 'feature'),
                                      ('_descriptor_params', p.json_descriptor_params, 'descriptor') ]:
            try:
                self.__dict__[dict_key] = eval(p_val)
            except:
                raise RuntimeError('Invalid JSON for the ' + desc + ': ' + p.json_feature_params)
            for key in ['type', 'package']:
                if key not in self.__dict__[dict_key]:
                    raise RuntimeError('No "%s" given for the %s; params: %s' % (key, desc, str(self.__dict__[dict_key])))
            self.__dict__['_%s_type' % desc] = self.__dict__[dict_key].pop('type')
            self.__dict__['_%s_package' % desc] = self.__dict__[dict_key].pop('package')

        # Deal with the combinations first
        self._feature_cell = None
        self._descriptor_cell = None
        self._feature_descriptor_cell = None
        if self._feature_type == self._descriptor_type and \
                                self._feature_package == self._descriptor_package == 'ecto_opencv':
            # deal with the combo case
            try:
                if self._feature_type == 'ORB':
                    self._feature_descriptor_cell = self.ORB_combination(**self._feature_params)
                elif self._feature_type == 'SIFT':
                    self._feature_descriptor_cell = self.SIFT_combination(**self._feature_params)
            except:
                raise RuntimeError('Parameters not supported for FeatureDescriptor: feature %s; descriptor: %s' %
                                   (self._feature_params, self._descriptor_params))
            if self._feature_type not in ['ORB', 'SIFT']:
                raise RuntimeError('Parameters not supported for FeatureDescriptor: feature %s; descriptor: %s' %
                                   (self._feature_params, self._descriptor_params))
        else:
            # if we are not computing everything at once, define the feature and the descriptor separately
            feature_class = find_cell([self._feature_package], self._feature_type)
            if feature_class is None:
                raise RuntimeError('Feature class not found: (type, package) = (%s, %s)' % (self._feature_type, self._feature_package))
            self._feature_cell = feature_class(**self._feature_params)

            descriptor_class = find_cell([self._descriptor_package], self._descriptor_type)
            if descriptor_class is None:
                raise RuntimeError('Descriptor class not found: (type, package) = (%s, %s)' % (self._descriptor_type, self._descriptor_package))
            self._descriptor_cell = descriptor_class(**self._descriptor_params)

        # everybody needs an image
        self._image_passthrough = ecto.Passthrough()
        if self._feature_descriptor_cell is None:
            i.forward('image', cell_name = '_image_passthrough', cell_key = 'in')
            i.forward('mask', cell_name = '_mask_passthrough', cell_key = 'in')
            if 'depth' in self._feature_cell.inputs.keys():
                i.forward('depth', cell_name = '_feature_cell', cell_key = 'input')
            o.forward('keypoints', cell_name = '_feature_cell', cell_key = 'keypoints')
            o.forward('descriptors', cell_name = '_descriptor_cell', cell_key = 'descriptors')
        else:
            # deal with the combo case
            i.forward_all('_feature_descriptor_cell')
            o.forward_all('_feature_descriptor_cell')

    def configure(self, p, i, o):
        pass

    def connections(self):
        connections = []
        if self._feature_descriptor_cell is None:
            connections += [ self._image_passthrough[:] >> self._feature_cell['image'],
                       self._image_passthrough[:] >> self._descriptor_cell['image'] ]
            connections += [ self._mask_passthrough[:] >> self._feature_cell['mask'],
                       self._mask_passthrough[:] >> self._descriptor_cell['mask'] ]
            connections += [ self._feature_cell['keypoints'] >> self._descriptor_cell['keypoints'] ]
        else:
            connections = [ ]

        return connections
