#!/usr/bin/env python
"""
Module defining a function that returns the appropriate ecto cells for Feature and Descriptor finding
"""

import ecto
import inspect
import pkgutil
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

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
        if '__path__' in m.__dict__:
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

    @staticmethod
    def _figure_out_cell_types(p):
        properties = {}
        # Make sure the parameters are valid
        for dict_key, p_val, desc in [ ('feature_params', p.json_feature_params, 'feature'),
                                      ('descriptor_params', p.json_descriptor_params, 'descriptor') ]:
            try:
                properties[dict_key] = eval(p_val)
            except:
                raise RuntimeError('Invalid JSON for the ' + desc + ': ' + p.json_feature_params)
            for key in ['type', 'module']:
                if key not in properties[dict_key]:
                    raise RuntimeError('No "%s" given for the %s; parameters are: %s' % (key, desc,
                                                                                         str(properties[dict_key])))
            properties['%s_type' % desc] = properties[dict_key].pop('type')
            properties['%s_module' % desc] = properties[dict_key].pop('module')

        # Deal with the combinations first
        cell_types = {'feature_descriptor_cell':None, 'feature_cell':None, 'descriptor_cell':None}
        if properties['feature_type'] == properties['descriptor_type'] and \
                                properties['feature_module'] == properties['descriptor_module']:
            # deal with the combo case first
            feature_descriptor_class = find_cell([properties['feature_module']], properties['feature_type'])
            try:
                cell_types['feature_descriptor_cell'] = feature_descriptor_class
            except:
                raise RuntimeError('Parameters not supported for FeatureDescriptor: feature %s; descriptor: %s' %
                                   (properties['feature_params'], properties['descriptor_params']))
        else:
            # if we are not computing everything at once, define the feature and the descriptor separately
            feature_class = find_cell([properties['feature_module']], properties['feature_type'])
            if feature_class is None:
                raise RuntimeError('Feature class not found: (type, module) = (%s, %s)' % (properties['feature_type'], properties['feature_module']))
            cell_types['feature_cell'] = feature_class

            descriptor_class = find_cell([properties['descriptor_module']], properties['descriptor_type'])
            if descriptor_class is None:
                raise RuntimeError('Descriptor class not found: (type, module) = (%s, %s)' % (properties['descriptor_type'], properties['descriptor_module']))
            cell_types['descriptor_cell'] = descriptor_class
        return cell_types

    @staticmethod
    def declare_cells(p):
        cell_types = FeatureDescriptor._figure_out_cell_types(p)

        cells = {}
        if cell_types['feature_descriptor_cell'] is not None:
            cells['feature_descriptor_cell'] = CellInfo(cell_types['feature_descriptor_cell'])
        else:
            cells['image_passthrough'] = CellInfo(ecto.Passthrough)
            cells['mask_passthrough'] = CellInfo(ecto.Passthrough)
            cells['feature_cell'] = CellInfo(cell_types['feature_cell'])
            cells['descriptor_cell'] = CellInfo(cell_types['descriptor_cell'])

        return cells

    @staticmethod
    def declare_direct_params(p):
        p.declare('json_feature_params', 'Parameters for the feature as a JSON string. '
                  'It should have the format: "{"type":"ORB/SIFT whatever", "module":"where_it_is", "param_1":val1, ....}',
                  '{"type": "ORB", "module": "ecto_opencv.features2d"}')
        p.declare('json_descriptor_params', 'Parameters for the descriptor as a JSON string. '
                  'It should have the format: "{"type":"ORB/SIFT whatever", "module":"where_it_is", "param_1":val1, ....}',
                  '{"type": "ORB", "module": "ecto_opencv.features2d"}')

    @staticmethod
    def declare_forwards(p):
        cell_types = FeatureDescriptor._figure_out_cell_types(p)

        # everybody needs an image
        if cell_types['feature_descriptor_cell'] is None:
            i = {'image_passthrough': [Forward('in', 'image')],
                 'mask_passthrough': [Forward('in','mask')]}

            if 'depth' in cell_types['feature_cell'].inputs.keys():
                i['feature_cell'] = [Forward('depth')]

            o = {'feature_cell': Forward('keypoints'),
                 'descriptor_cell': Forward('descriptors')}
        else:
            # deal with the combo case
            i = {'feature_descriptor_cell': 'all'}
            o = {'feature_descriptor_cell': 'all'}

        return ({},i,o)

    def connections(self, _p):
        connections = []
        if self.feature_descriptor_cell is None and self.feature_cell is not None and self.descriptor_cell is not None:
            connections += [ self.image_passthrough[:] >> self.feature_cell['image'],
                       self.image_passthrough[:] >> self.descriptor_cell['image'] ]
            connections += [ self.mask_passthrough[:] >> self.feature_cell['mask'],
                       self.mask_passthrough[:] >> self.descriptor_cell['mask'] ]
            connections += [ self.feature_cell['keypoints'] >> self.descriptor_cell['keypoints'] ]
        else:
            connections += [ self.feature_descriptor_cell ]

        return connections
