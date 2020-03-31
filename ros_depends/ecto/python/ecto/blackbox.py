#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
import collections
import ecto
import types

class BlackBoxError(RuntimeError):
    def __init__(self, *args, **kwargs):
        RuntimeError.__init__(self, *args, **kwargs)

_BlackBoxForward = collections.namedtuple('BlackBoxForward', ['key', 'new_key', 'new_doc', 'new_default'])

def BlackBoxForward(key, new_key=None, new_doc=None, new_default=None):
    """
    Function to define the information of a forward in :func:`ecto.blackbox.BlackBox.declare_forwards`

    :param key: tendril to forward from the cell
    :param new_key: the name under which the tendril will be forwarded. If not given or None or emtpy, it will be the same
    :param new_doc: the new documentation of the tendril. If not given or None or emtpy, it will be the same
    :param new_default: the new default value of the tendril. If not given or None or emtpy, it will be the same
    :return: a _BlackBoxForward object which is a private namedtuple that users should not interact with
    """
    if not new_key:
        new_key = key
    return _BlackBoxForward(key, new_key, new_doc, new_default)

_BlackBoxCellInfo = collections.namedtuple('BlackBoxCellInfo', ['python_class', 'params', 'name'])

def BlackBoxCellInfo(python_class, params=None, name=None):
    """
    Function to define the information of a cell in :func:`ecto.blackbox.BlackBox.declare_cells`

    :param python_class: a Class object describing the cell
    :param params: a dictionary of parameters that will be sent to the constructor of
                  the cell (with the forwarded parameters if any, and they have priority over forwards)
    :param name: the name of the cell when instantiating it
    :return: a _BlackBoxCellInfo object which is a private namedtuple that users should not interact with
    """
    if name is None:
        name = str(python_class)
    if params is None:
        params = {}
    return _BlackBoxCellInfo(python_class, params, name)

def _deep_copy_tendrils(tendrils_in, values=None):
    """
    Given some tendrils and their values, deep copy them
    :param tendrils_in: an ecto.Tendrils()
    :param values: a dictionary {'tendril_key': tendril_value}
                  if 'tendril_key' is not in 'tendrils_in', it will be ignored
    :return: a deep copy of 'tendrils_in' with the values from 'values'
    """
    if values is None:
        values = {}
    tendrils_out = ecto.Tendrils()
    for key, tendril in tendrils_in:
        new_tendril = ecto.Tendril.createT(tendril.type_name)
        if key in values:
            new_tendril.set(values[key])
        else:
            new_tendril.copy_value(tendril)
        new_tendril.doc = tendril.doc
        tendrils_out.declare(key, new_tendril)

    return tendrils_out

def _get_param_tendrils(cell_info, forwards=None, params=None):
    """
    This function returns the params ecto.Tendrils of a cell but it also overrides
    the default values with the arguments, in order of priority (from low to high):
    - the defaults of cell_info
    - the forwards
    - any given parameters
    :param cell_info: a BlackBoxCellInfo object
    :param forwards: a list of BlackBoxForward or the string 'all'
    :param params: a dictionary of values to give to the parameters that have no default
    :return: params Tendrils of a class with values overriden from cell_info or forwards
    """
    if forwards is None:
        forwards = []
    if params is None:
        params = {}

    cell_class = cell_info.python_class
    if hasattr(cell_class, 'params'):
        # this is a C++ cell visible from Python
        tendrils = _deep_copy_tendrils(cell_class.params)
    else:
        # otherwise, call the declare_params function
        tendrils = ecto.Tendrils()
        if issubclass(cell_class, BlackBox):
            cell_class.declare_params(tendrils, **params)
        else:
            cell_class.declare_params(tendrils)

    # override the values with whatever is in params
    for key, val in params.items():
        if key in tendrils:
            tendrils.at(key).set(val)

    # override the values with whatever is in forwards
    if isinstance(forwards, list):
        for forward in forwards:
            if forward.key in tendrils and forward.new_default is not None:
                tendrils.at(forward.key).set(forward.new_default)

    # override the values with whatever is in cell_info
    for key, val in cell_info.params.items():
        if key in tendrils:
            tendrils.at(key).set(val)

    return tendrils

def _deep_copy_tendrils_to_tendrils(tendrils_from, forwards, tendrils_to):
    """
    Copies some tendrils from "tendrils_from" to "tendrils_to" according to the
    BlackBoxForward's in "forwards".
    Contrary to _copy_tendrils, new tendrils are created and their attributes
    are copied over
    :param tendrils_from: some ecto.Tendrils object from a cell. Values from
                          it will be copied to "tendrils_to"
    :param forwards: forwards to follow the rules of
    :param tendrils_to: the tendrils to which data will be copied to
    """
    if forwards == 'all':
        for key, tendril in tendrils_from:
            if key not in tendrils_to:
                new_tendril = ecto.Tendril.createT(tendril.type_name)
                new_tendril.copy_value(tendril)
                new_tendril.doc = tendril.doc

                tendrils_to.declare(key, new_tendril)
            else:
                tendrils_to.at(key).copy_value(tendril)
            tendrils_to.at(key).doc = tendril.doc
    else:
        for forward in forwards:
            key, new_key, new_doc, new_default = forward
            if key not in tendrils_from:
                raise BlackBoxError('No tendril "%s" found when declaring forward %s' %
                                    (key, str(forward)))
            # declare the new forwarded tendril
            tendril = tendrils_from.at(key)
            if new_key not in tendrils_to:
                new_tendril = ecto.Tendril.createT(tendril.type_name)
                new_tendril.copy_value(tendril)
                new_tendril.doc = tendril.doc

                tendrils_to.declare(new_key, new_tendril)
            else:
                tendrils_to.at(new_key).copy_value(tendril)

            # update the docs if needed
            if new_doc:
                tendrils_to.at(new_key).doc = new_doc

            # update the default if needed
            if new_default is not None:
                tendrils_to.at(new_key).set(new_default)

def _copy_tendrils_to_tendrils(tendrils_from, forwards, tendrils_to):
    """
    Copies some tendrils from "tendrils_from" to "tendrils_to" according to the
    BlackBoxForward's in "forwards".
    :param tendrils_from: some ecto.Tendrils object from a cell. Values from
                          it will be copied to "tendrils_to"
    :param forwards: forwards to follow the rules of
    :param tendrils_to: the tendrils to which data will be copied to
    """
    if forwards == 'all':
        for key, tendril in tendrils_from:
            tendrils_to.declare(key, tendril)
    else:
        if not isinstance(forwards, list):
            raise BlackBoxError('Your declare_forwards must return "all" or an array of BlackBoxForward. '
                                'It now is %s' % str(forwards))
        for forward in forwards:
            key, new_key, new_doc, new_default = forward
            if key not in tendrils_from:
                raise BlackBoxError('No tendril "%s" found when declaring forward %s' %
                                    (key, str(forward)))

            # declare the new forwarded tendril
            tendrils_to.declare(new_key, tendrils_from.at(key))

            # update the docs if needed
            if new_doc:
                tendrils_to.at(new_key).doc = new_doc

            # update the default if needed
            if new_default is not None:
                tendrils_to.at(new_key).set(new_default)

class BlackBox(object):
    """
    The BlackBox may be used as an encapsulation idiom within ecto, to declare reusable plasms.
    
    A BlackBox is a meta-cell. 3 functions need to be implemented when inheriting.
    *declare_fowards*: defines the inputs/outputs/parameters that are forwarded to the inner cells.
    *declare_cells*: declare_io and declare_params should not be overriden but in order to have them work 
    statically, information about the inner cells have to be given in *declare_cells*.
    *declare_direct_params*: a BlackBox can have its own parameters and this is where they are declared
    """
    __looks_like_a_cell__ = True

    def __init__(self, *args, **kwargs):
        """
        :param args: if given, only the first one is considered, and it becomes the name
                     of the BlackBox
        :param kwargs: any parameter that is a declared parameter or a forwarded one
        """
        # check that the overrides have proper types
        # declare_direct_params has to be a classmethod or a staticmethod (as declare_params uses it and it's a
        # classmethod)
        for func in [self.__class__.declare_cells, self.__class__.declare_direct_params,
                     self.__class__.declare_forwards]:
            if type(func) != types.FunctionType:
                if type(func) != types.MethodType:
                    raise BlackBoxError('The "%s" member of the BlackBox %s ' % (func.__name__,
                                        self.__class__.__name__) + 'is not a function.')
                if func.__self__ is None:
                    raise BlackBoxError('The "%s" function of the BlackBox %s '  % (func.__name__,
                                self.__class__.__name__) + 'needs to be decorated with @classmethod or @staticmethod.')

        self.niter = kwargs.get('niter', 1)
        self.__impl = None
        self.__doc__ = ''

        self.__params = ecto.Tendrils()
        self.__inputs = ecto.Tendrils()
        self.__outputs = ecto.Tendrils()

        self.__configure(**kwargs)
        self.__connect()
        self.__impl.name(self.__class__.__name__)
        self.__gen_doc()
        if len(args) > 0:
            self.__impl.name(args[0])

    def __getitem__(self, key):
        ''' This acts just like any other module when asking for a spec,
        e.g. spec = m['key1','key2']
        '''
        return self.__impl[key]

    def __gen_doc(self):
        short_doc = ''
        if self.__doc__:
            short_doc = self.__doc__
        self.__doc__ = self.__impl.gen_doc(short_doc)

    def __connect(self):
        ''' Connect oneself to the plasm.
        '''
        plasm = ecto.Plasm()
        try:
            connections = self.connections(self.__params)
        except Exception as e:
            raise BlackBoxError('Got error "%s" when calling connections on "%s"' % (str(e), str(self)))
        if not connections:
            raise BlackBoxError('Your BlackBox has empty connections')
        for x in connections:
            if not getattr(x, '__iter__', False):
                plasm.insert(x)
            else:
                plasm.connect(x)
        self.__impl = ecto.create_black_box(plasm,
                                           niter=self.niter,
                                           parameters=self.__params,
                                           inputs=self.__inputs,
                                           outputs=self.__outputs)

    def __getattr__(self, name):
        if name in ('parameters',):
            name = 'params'
        if '__impl' not in  name and name in dir(self.__impl):
            return getattr(self.__impl, name)
        if name == '__impl':
            return self.__impl
        else:
            raise AttributeError(self, name)

    def __dir__(self):
        return ['outputs', 'inputs', 'params'] + self.__dict__.keys()

    @classmethod
    def inspect(cls, *args, **kwargs):
        '''This emulates the inspect method that each ecto cell has, which creates a light
        version of the cell, where the default values for p,i,o are acceptable.
        '''
        return cls(*args, **kwargs)

    @staticmethod
    def declare_direct_params(p):
        """
        This function can be overriden by a child class.

        This function declares normal parameters for the BlackBox, i.e. you need to declare them
        using p.declare('key_name', 'key_docs', default_value) like you would in a normal
        Python cell.

        If you want to have your cell expose different parameters according to different
        parameters sent to the constructor of the BlackBox, you can make this function
        non-static. declare_params and declare_io (that are static) might then fail if called
        statically but they are not used to build a BlackBox.

        :param p: an ecto.Tendrils() object
        """
        pass

    @classmethod
    def declare_cells(cls, p):
        """
        This function can be overriden by a child class.

        Given some parameters, define the characteristics of the cells.

        :param p: an ecto.Tendrils() object
        :return: a dictionary of the form:
                    {'cell_name': BlackBoxCellInfo_call, 'cell_name': cell_instance}
        """
        return {}

    @classmethod
    def declare_params(cls, p, **kwargs):
        """
        This method should NOT be overridden by a child class.

        This function returns the parameter tendrils but as there are two kinds
        (namely the BlackBox ones defined in "declare_direct_params" and the
        forwarded parameters), there are two levels of parameters. If you
        just call the function with no kwargs, you get the default parametes. But
        if you know the parameters you will send to your BlackBox constructor, 
        call declare_params with it and you will have a "runtime estimate" of your
        parameters.

        :param p: an ecto.Tendrils() object
        :param kwargs: anything sent to the constructor of the BlackBox
        """
        # First, figure out the direct parameters
        cls.declare_direct_params(p)

        # complete the p using the default values given in **kwargs
        for key, val in kwargs.items():
            if key in p:
                p.at(key).set(val)

        # now that we know the parameters, define the cells accordingly
        cell_infos = cls.declare_cells(p)
        if not isinstance(cell_infos, dict):
            raise BlackBoxError('Your declare_cells needs to return a dictionary '
                                'of the form:\n'
                                '{"cell_name": BlackBoxCellInfo}')

        # parse the cell BlackBoxForwards and add them to the tendrils p calling
        # the declare_params from the cells or their .params to get tendril info
        p_forwards, _i_forwards, _o_forwards = cls.declare_forwards(p)
        for cell_name, forwards in p_forwards.items():
            if isinstance(cell_infos[cell_name], _BlackBoxCellInfo):
                tendrils_params = _get_param_tendrils(cell_infos[cell_name], forwards, kwargs)
            else:
                tendrils_params = cell_infos[cell_name].params
            _deep_copy_tendrils_to_tendrils(tendrils_params, forwards, p)

    @classmethod
    def declare_forwards(cls, _p):
        """
        This function can be overriden by a child class.

        :param _p: an ecto.Tendrils object from which some logic can be done to know what to forward.
                  This function will be called after declare_direct_params which fills 'p' with BlackBox
                  specific parameters (non-forwarded)
        :return: a tuple of three dictionaries definining the params/input/output
                    forwarded to/from the inner cells. It has the format:
                    {'cell_name': 'all', 'cell_name': [BlackBoxForward_call1, BlackBoxForward_call2]}
        """
        return ({}, {}, {})

    @classmethod
    def declare_io(cls, p, i, o):
        """
        This function has the same meaning as in C++ and should NOT be overriden by a child class.

        :param p: an ecto.Tendrils object for the parameters
        :param i: an ecto.Tendrils object for the inputs
        :param o: an ecto.Tendrils object for the outputs   
        """
        cell_infos = cls.declare_cells(p)
        try:
            _p_forwards, i_forwards, o_forwards = cls.declare_forwards(p)
        except:
            raise BlackBoxError('Your declare_forwards needs to return a tuple of 3 dictionaries '
                                'of the form:\n'
                                '{"cell_name": "all", "cell_name": [BlackBoxForward1, BlackBoxForward2]}')

        # go over the two sets of tendrils: i and o
        for info_tuple in [(i_forwards, i, 'inputs'), (o_forwards, o, 'outputs')]:
            cell_forwards, tendrils, tendril_type = info_tuple
            for cell_name, forwards in cell_forwards.items():
                cell_info = cell_infos[cell_name]
                if isinstance(cell_infos[cell_name], _BlackBoxCellInfo):
                    python_class = cell_infos[cell_name].python_class
                    if hasattr(python_class, tendril_type):
                        # get the tendrils from the class if it has them
                        _deep_copy_tendrils_to_tendrils(getattr(python_class, tendril_type), forwards, tendrils)
                    else:
                        # in case the cell has no 'inputs'/'outputs' attribute (e.g. if it is a BlackBox
                        # or a pure Python cell)
                        cell_params = _get_param_tendrils(cell_info, forwards.get(cell_name, {}))
                        cell_tendrils = ecto.Tendrils()
                        if tendril_type == 'inputs':
                            python_class.declare_io(cell_params, cell_tendrils, ecto.Tendrils())
                        else:
                            python_class.declare_io(cell_params, ecto.Tendrils(), cell_tendrils)

                        _deep_copy_tendrils_to_tendrils(cell_tendrils, forwards, tendrils)
                else:
                    _deep_copy_tendrils_to_tendrils(getattr(cell_info, tendril_type), forwards, tendrils)

    def __configure(self, **kwargs):
        """
        Private implementation that generates the cells/tendrils inside the BlackBox
        """
        p = ecto.Tendrils()
        self.declare_direct_params(p)

        # complete the p using the default values given in **kwargs
        for key, val in kwargs.items():
            if key in p:
                p.at(key).set(val)

        try:
            p_forwards, i_forwards, o_forwards = self.declare_forwards(p)
        except Exception as e:
            raise BlackBoxError('Your declare_forwards needs to return a tuple of 3 dictionaries '
                                'of the form:\n'
                                '{"cell_name": "all", "cell_name": [BlackBoxForward1, BlackBoxForward2]}: %s' % e)

        # create the cells
        cell_infos = self.declare_cells(p)
        for cell_name, cell_info in cell_infos.items():
            # if a full-fledged cell is given, just assign it to the blackbox member right away
            if not isinstance(cell_info, _BlackBoxCellInfo):
                setattr(self, cell_name, cell_info)
                continue

            cell_tendrils = _get_param_tendrils(cell_info, p_forwards.get(cell_name, []), kwargs)

            # convert the tendrils to parameters to send to the constructor
            params = {}
            for key, tendril in cell_tendrils.items():
                params[key] = tendril.val

            # add the cell to the BlackBox
            if cell_info.name:
                setattr(self, cell_name, cell_info.python_class(cell_info.name, **params))
            else:
                setattr(self, cell_name, cell_info.python_class(**params))

        # redefine the parameters so that they are linked to tendrils of actual cells
        self.__params = ecto.Tendrils()
        self.declare_direct_params(self.__params)

        for cell_name, forwards in p_forwards.items():
            cell = getattr(self, cell_name)
            _copy_tendrils_to_tendrils(cell.params, forwards, self.__params)

        # complete the params using the default values given in **kwargs
        for key, val in kwargs.items():
            if key in self.__params:
                self.__params.at(key).set(val)

        # redefine the io so that they are linked to tendrils of actual cells
        self.__inputs = ecto.Tendrils()
        self.__outputs = ecto.Tendrils()
        for tendril_type, all_forwards in [('inputs', i_forwards), ('outputs', o_forwards)]:
            for cell_name, forwards in all_forwards.items():
                if not hasattr(self, cell_name):
                    raise BlackBoxError('Cell of name "%s" needs to be ' % cell_name +
                                        'declared in "declare_cells"')
                cell = getattr(self, cell_name)
                if not hasattr(cell, tendril_type):
                    raise BlackBoxError('Cell of name "%s" needs has no %s ' %
                                        (cell_name, tendril_type))
                if tendril_type=='inputs':
                    _copy_tendrils_to_tendrils(cell.inputs, forwards, self.__inputs)
                else:
                    _copy_tendrils_to_tendrils(cell.outputs, forwards, self.__outputs)

        # call the configure from the user
        self.configure(self.__params, self.__inputs, self.__outputs)

    def configure(self, p, i, o):
        """
        This function has the same meaning as in C++ and can be overriden by a child class.

        This function should be used to allocate all cells internal to a BlackBox

        :param p: an ecto.Tendrils object for the parameters
        :param i: an ecto.Tendrils object for the inputs
        :param o: an ecto.Tendrils object for the outputs
        """
        pass

    def connections(self, p):
        """
        This function has to be overriden by a child class.

        :param p: an already filled ecto.Tendrils for the parameters that can be used
                  to decide on certain connections
        :return: an iterable of tendril connections.
        """
        raise NotImplementedError("All BlackBox's must implement at least the connections function....")

    def cell(self):
        '''
        Return an instance of the cell that backs this
        BlackBox. Useful for ecto.If, or other places that expect a
        cell
        '''
        return self.__impl
