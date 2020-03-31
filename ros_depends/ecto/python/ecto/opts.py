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
'''
Set of helper functions for ecto scripts, that expose a few common args for
the schedulers.
'''

import ecto

def use_ipython(options, sched, plasm, locals={}):
    '''Launch a plasm using ipython, and a scheduler of choice.

       Keyword arguments:
       options -- are from scheduler_options
       sched -- is an already initialized scheduler for plasm.
       plasm -- The graph to execute
       locals -- are a dictionary of locals to forward to the ipython shell, use locals()
    '''
    #expose the locals to the ipython prompt.
    for key, val in locals.items():
        vars()[key] = val

    sched.prepare_jobs(options.niter)
    print "Scheduler ready to run. To execute for some number of microseconds, type:"
    print "sched.run(1000)"

    import IPython
    if IPython.__version__ < '0.11':
        from IPython.Shell import IPShellEmbed
        #the argv is important so that the IPShellEmbed doesn't use the global
        #Also fancy colors!!!!
        ipython_argv = ['-prompt_in1', 'Input <\\#>', '-colors', 'LightBG']
        ipshell = IPShellEmbed(ipython_argv)
        ipshell()
    else:
        from IPython import embed
        embed() # this call anywhere in your program will start IPython

def run_plasm(options, plasm, locals={}):
    ''' Run the plasm given the options from the command line parser.
        :param options: The command line, preparsed options object.
        It is assumed that you have filled this object using scheduler_options.
        :param plasm: The plasm to run.
        :param locals: Any local variables that you would like available to the iPython shell session.
    '''
    if options.graphviz:
        ecto.view_plasm(plasm)
    if len(options.dotfile) > 0:
        print >> open(options.dotfile, 'wt'), plasm.viz()
    if len(options.logfile) > 0:
        ecto.log_to_file(options.logfile)
    if options.gui:
        from ecto.gui import gui_execute
        gui_execute(plasm)
    else:
        sched = ecto.Scheduler(plasm)
        if options.ipython:
            use_ipython(options, sched, plasm, locals)
        else:
            sched.execute(options.niter)
    if options.stats:
        print sched.stats()

class CellFactory(object):
    '''A factory for cells that are created from command line args.'''
    def __init__(self, CellType, CellProtoType, prefix=None):
        '''
        cellType a type of ecto cell to create
        prefix the prefix used by the parser
        '''
        self.cellType = CellType
        self.cellProtoType = CellProtoType
        self.prefix = prefix
    def __call__(self, args, cellname=''):
        '''
        args from a parser where cell_options was used to add arguments.
        cellname option cellname
        '''
        params = {}
        prototype = self.cellProtoType
        for key, value in args.__dict__.iteritems():
            if not self.prefix or key.startswith(self.prefix + '_'):
                if self.prefix:
                    p = ''.join(key.split(self.prefix + '_')[:])
                else:
                    p = key
                if p not in prototype.params.keys():
                    continue
                t = type(prototype.params[p])
                if 'values' in t.__dict__ and type(value) != t and type(value) == str:
                    params[p] = t.names[value]
                else:
                    params[p] = t(value)

        nkwargs = ()
        if len(cellname) > 0:
            nkwargs = (cellname,) #tuple
        return self.cellType(*nkwargs, **params)

class CellYamlFactory(object):
    '''A factory to go between cells and YAML files

    :param CellOrCellType: The prototype to base the factory off of.
    :type CellOrCellType: Cell Class, or Cell Instance
    :param prefix: The top level key for the parameters dict for this factory
    :type prefix: str
    '''

    def __init__(self, CellOrCellType, prefix):

        cell_type, cell = _cell_type_instance(CellOrCellType)
        self.cell_type = cell_type
        self.cell = cell
        self.prefix = prefix
        params = cell.params
        c = {}
        for x in params:
            c[x.key()] = x.data().get()
            c[x.key() + '__doc__'] = x.data().doc
        self.params = c

    def dump(self, stream=None):
        '''Dump YAML for this factory.

        :param stream: A stream like object to print the YAML to.
        :returns: The string YAML representation of the prototype used to initialize the factory.
        '''
        from yaml import dump
        return dump({self.prefix:self.params}, default_flow_style=False, stream=stream)

    def load(self, parsed, cell_name=''):
        '''Create a cell from a parsed bit of YAML.

        :param parsed: A dictionary that has been parsed from a YAML file.
        :param cell_name: will be used as the cell's instance name.
        :returns: A brand new instance of a cell, modeled after whatever is in the YAML
        '''
        params = parsed[self.prefix]
        nkwargs = ()
        params_clean = {}
        for x in self.cell.params:
            if x.key() in params:
                params_clean[x.key()] = params[x.key()]
        if len(cell_name) > 0:
            nkwargs = (cell_name,) #tuple
        cell = self.cell_type(*nkwargs, **params_clean)
        return cell

def _cell_type_instance(CellOrCellType):
    c = CellOrCellType
    cell_type = CellOrCellType
    if isinstance(cell_type, object.__class__):
        c = cell_type.inspect()
    else:
        cell_type = c.__class__
    # print "....", (cell_type, c)
    return (cell_type, c)



def cell_options(parser, CellType, prefix=None):
    '''Creates an argument parser group for any cell.
    CellType may be either a __class__ type, or an instance object.
    '''
    group = parser.add_argument_group('%s options' % prefix)
    cell_type, cell = _cell_type_instance(CellType)

    params = cell.params

    for x in params:
        if prefix:
            dest = '%s_%s' % (prefix, x.key())
        else:
            dest = x.key()
        tendril_type = type(x.data().val)
        if 'values' in tendril_type.__dict__:
            choices = tendril_type.names.keys()
            tendril_type = str
        else:
            choices = None

        kwargs = dict(metavar='%s' % dest.upper(),
                      dest=dest,
                      type=tendril_type,
                      default=x.data().val,
                      help=x.data().doc + ' ~  (default: %(default)s)'
                      )
        if tendril_type is type(True):
            if x.data().val:
                kwargs = dict(dest=dest, help=x.data().doc + ' Disables %s' % dest, action='store_false')
                dest = '%s_disable' % dest
            else:
                kwargs = dict(dest=dest, help=x.data().doc + ' Enables %s' % dest, action='store_true')
                dest = '%s_enable' % dest
        if choices:
            kwargs['choices'] = choices
            kwargs['help'] = kwargs['help'] + ' (choices: %(choices)s)'
        group.add_argument('--%s' % dest, **kwargs)

    factory = CellFactory(cell_type, cell, prefix)
    return factory

def scheduler_options(parser,
                      default_niter=0,
                      default_shell=False,
                      default_graphviz=False):
    '''Creates an argument parser for ecto schedulers.  Operates inplace on the
    given parser object.
    '''
    ecto_group = parser.add_argument_group('Ecto runtime parameters')
    ecto_group.add_argument('--niter', metavar='ITERATIONS', dest='niter',
                        type=int,
                        default=default_niter,
                        help='''Run the graph for niter iterations.
                        0 means run until stopped by a cell or external forces.
                        (default: %(default)s)'''
                        )
    ecto_group.add_argument('--shell', dest='ipython', action='store_const',
                        const=True, default=default_shell,
                        help=''''Bring up an ipython prompt,
                        and execute asynchronously.(default: %(default)s)
                        '''
                        )
    ecto_group.add_argument('--gui', dest='gui', action='store_true',
                        help='Bring up a gui to help execute the plasm.'
                        )

    ecto_group.add_argument('--logfile', metavar='LOGFILE', dest='logfile', type=str,
                        default='',
                        help='''Log to the given file, use tail -f LOGFILE to see the
                       live output. May be useful in combination with --shell'''
                       )
    ecto_group.add_argument('--graphviz', dest='graphviz',
                        action='store_const',
                        const=True, default=default_graphviz,
                        help='Show the graphviz of the plasm. (default: %(default)s)'
                        )
    ecto_group.add_argument('--dotfile', dest='dotfile', type=str, default='',
                        help='''Output a graph in dot format to the given file.
                        If no file is given, no output will be generated. (default: %(default)s)'''
                        )
    ecto_group.add_argument('--stats', dest='stats', action='store_const',
                        const=True, default=default_graphviz,
                        help='Show the runtime statistics of the plasm.')

def doit(plasm, description="An ecto graph.", locals={}, args=None,
         default_niter=0,
         default_shell=False,
         default_graphviz=False):
    ''' doit is a short hand for samples, that is a combination of a
       call to scheduler_options, and then run_plasm.  This function
       in not intended to allow customization of parameter parsing.
       If this is needed please call scheduler_options and run_plasm
       yourself.

       :param args: If this is None, default to using the sys.argv
           args, otherwise this overrides it.
       :param locals: May be used to forward any local variables to
           the ipython shell. Suggest either vars() or locals() to do
           this.
       :param default_shell: Override
       :param default_graphviz: Override
    '''
    import argparse
    parser = argparse.ArgumentParser(description=description)
    scheduler_options(parser,
                      default_niter=default_niter,
                      default_shell=default_shell,
                      default_graphviz=default_graphviz)
    options = parser.parse_args(args=args)
    run_plasm(options, plasm, locals)

if __name__ == '__main__':
    import ecto.ecto_test as ecto_test
    import yaml
    import argparse
    parser = argparse.ArgumentParser(description='My awesome program thing.')
    parser.add_argument('-i,--input', metavar='IMAGE_FILE', dest='imagefile',
                        type=str, default='', help='an image file to load.')
    scheduler_options(parser, default_niter=2)

    multiply_factory = cell_options(parser, ecto_test.Multiply, prefix='mult')
    const_factory = cell_options(parser, ecto.Constant(value=0.50505), prefix='const')

    #parser.print_help()
    options = parser.parse_args()

    c = const_factory(options)
    m = multiply_factory(options)
    cyaml = CellYamlFactory(c, 'const')
    print cyaml.dump()
    c = cyaml.load(yaml.load(cyaml.dump()))
    pr = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(c[:] >> m[:],
                  m[:] >> pr[:]
                  )

    run_plasm(options, plasm, locals=vars())

