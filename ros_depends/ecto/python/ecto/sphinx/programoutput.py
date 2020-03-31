# -*- coding: utf-8 -*-
# Copyright (c) 2010, 2011, Sebastian Wiesner <lunaryorn@googlemail.com>
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.

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


"""
    sphinxcontrib.programoutput
    ===========================

    This extension provides a directive to include the output of commands as
    literal block while building the docs.

    .. moduleauthor::  Sebastian Wiesner  <lunaryorn@googlemail.com>
"""

__version__ = '0.4.1'

import sys, shlex, os
from subprocess import Popen, CalledProcessError, PIPE, STDOUT
from collections import defaultdict

from docutils import nodes
from docutils.parsers import rst
from docutils.parsers.rst.directives import flag, unchanged


class program_output(nodes.Element):
    pass


def _slice(value):
    parts = [int(v.strip()) for v in value.split(',')]
    if len(parts) > 2:
        raise ValueError('too many slice parts')
    return tuple((parts + [None]*2)[:2])


class ProgramOutputDirective(rst.Directive):
    has_content = False
    final_argument_whitespace = True
    required_arguments = 1

    option_spec = dict(shell=flag, prompt=flag, nostderr=flag,
                       in_srcdir=flag, ellipsis=_slice, extraargs=unchanged,
                       expect_error=flag, until=unchanged)

    def run(self):
        rest = ''
        if 'in_srcdir' in self.options:
            document = self.state.document
            arg0 = self.arguments[0]
            if arg0.count(' ') > 0:
                (filename, rest) = arg0.split(' ', 1)
            else:
                filename = arg0
            env = document.settings.env
  
            if filename.startswith('/') or filename.startswith(os.sep):
                rel_fn = filename[1:]
            else:
                docdir = os.path.dirname(env.doc2path(env.docname, base=None))
                rel_fn = os.path.join(docdir, filename)
            try:
                fn = os.path.join(env.srcdir, rel_fn)
            except UnicodeDecodeError:
                # the source directory is a bytestring with non-ASCII characters;
                # let's try to encode the rel_fn in the file system encoding
                rel_fn = rel_fn.encode(sys.getfilesystemencoding())
                fn = os.path.join(env.srcdir, rel_fn)
        else:
            fn = self.arguments[0]

        node = program_output()
        if len(rest) > 0:
            fn += (' ' + rest)

        node['command'] = fn

        if self.name == 'command-output':
            node['show_prompt'] = True
        else:
            node['show_prompt'] = 'prompt' in self.options

        node['hide_standard_error'] = 'nostderr' in self.options
        node['extraargs'] = self.options.get('extraargs', '')
        node['use_shell'] = 'shell' in self.options
        if 'until' in self.options:
            node['until'] = self.options['until']
            
        node['expect_error'] = 'expect_error' in self.options
        if 'ellipsis' in self.options:
            node['strip_lines'] = self.options['ellipsis']
        return [node]


def full_executable_path(invoked, path):
    
    if invoked[0] == '/':
        return invoked

    for d in path:
        full_path = os.path.join(d, invoked)
        if os.path.exists( full_path ):
            return full_path

    return invoked # Not found; invoking it will likely fail


class ProgramOutputCache(defaultdict):
    """
    :class:`collections.defaultdict` sub-class, which caches program output.

    If a program's output is not contained in this cache, the program is
    executed, and its output is placed in the cache.
    """

    def __missing__(self, key):
        """
        Called, if a command was not found in the cache.

        ``key`` is a triple of ``(cmd, shell, hide_stderr)``.  ``cmd`` is
        the command tuple.  If ``shell`` is ``True``, the command is
        executed in the shell, otherwise it is executed directly.  If
        ``hide_stderr`` is ``True``, the standard error of the program is
        discarded, otherwise it is included in the output.
        """
        (cmd, shell, hide_stderr, path, expect_error) = key
        shell = True
        if cmd.count(' '):
            (bin_name, args) = cmd.split(' ', 1)
        else:
            bin_name = cmd
            args = ''
        fullpath = full_executable_path(bin_name, path)
        cmd = fullpath + " " + args

        print "program-output about to run "+ cmd
        proc = Popen(cmd, shell=shell, stdout=PIPE, stderr=PIPE if hide_stderr else STDOUT)
        stdout = proc.communicate()[0].decode(sys.getfilesystemencoding()).rstrip()

        if proc.returncode != 0 and (not expect_error):
            print "Unable to run '" + cmd + "' returned " + str(proc.returncode)\
                + 'stdout: ' + stdout
            raise CalledProcessError
        self[key] = stdout
        return stdout


def run_programs(app, doctree):
    """
    Execute all programs represented by ``program_output`` nodes in
    ``doctree``.  Each ``program_output`` node in ``doctree`` is then
    replaced with a node, that represents the output of this program.

    The program output is retrieved from the cache in
    ``app.env.programoutput_cache``.
    """
    if app.config.programoutput_use_ansi:
        # enable ANSI support, if requested by config
        from sphinxcontrib.ansi import ansi_literal_block
        node_class = ansi_literal_block
    else:
        node_class = nodes.literal_block

    cache = app.env.programoutput_cache

    for node in doctree.traverse(program_output):
        command = node['command']
        cmd_bytes = command.encode(sys.getfilesystemencoding())
        extra_args = node.get('extraargs', '').encode(sys.getfilesystemencoding())

        cmd = cmd_bytes
        if extra_args:
            cmd += ' ' + extra_args

        cache_key = (cmd, node['use_shell'], node['hide_standard_error'], tuple(app.config.programoutput_path),
                     node['expect_error'])
        output = cache[cache_key]

        # replace lines with ..., if ellipsis is specified
        if 'strip_lines' in node:
            lines = output.splitlines()
            start, stop = node['strip_lines']
            lines[start:stop] = ['...']
            output = '\n'.join(lines)

        if 'until' in node:
            output = output[:output.find(node['until'])]

        if node['show_prompt']:
            tmpl = app.config.programoutput_prompt_template
            output = tmpl % dict(command=command, output=output)

        new_node = node_class(output, output)
        new_node['language'] = 'text'
        node.replace_self(new_node)


def init_cache(app):
    """
    Initialize the cache for program output at
    ``app.env.programoutput_cache``, if not already present (e.g. being
    loaded from a pickled environment).

    The cache is of type :class:`ProgramOutputCache`.
    """
    if not hasattr(app.env, 'programoutput_cache'):
        app.env.programoutput_cache = ProgramOutputCache()


def setup(app):
    app.add_config_value('programoutput_use_ansi', False, 'env')
    app.add_config_value('programoutput_path', [], 'env')
    app.add_config_value('programoutput_prompt_template',
                         '$ %(command)s\n%(output)s', 'env')
    app.add_directive('program-output', ProgramOutputDirective)
    app.add_directive('command-output', ProgramOutputDirective)
    app.connect('builder-inited', init_cache)
    app.connect('doctree-read', run_programs)
