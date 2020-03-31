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

foreach(file
    throw_in_interpreter_thread

    test_addergraph
    test_async_bp_objects
    test_async_execution
    test_async_execution_harsh
    test_async_multiple_sched
    test_async
    test_blackbox
    test_break
    #test_bp_to_cell_ptr
    test_circular
    test_constant
    test_dealer
    test_doc
    test_dual_line_plasm
    test_entanglement
    test_exception
    test_exception_in_constructor
    test_fileIO
    test_handles
    test_If
    test_metrics
    test_module_qualification
    test_modules
    test_multiplasm_scheduler
    test_multiprocess
    test_nested_if
    test_no_ecto_import
    test_options
    test_parameter_callbacks
    test_passthrough
    test_ptime_bindings
    test_plasm
    test_process_return_values
    test_python_module
    test_random
    test_reconnect
    test_redirect
    test_required_io
    test_required_param
    # test_restart  # this needs thinking about... can't test nexecutions with a stateful cell
    test_shared_pass
    test_small_plasms
    test_start_stop_counter
    test_strands
    test_tendril
    test_tendril_type_reg
    test_tendrils
    test_tendrils_mux
    test_tendrils_serialization
    test_type_mismatch_errors
    test_wrong_param_type
    )
    add_test(ecto_${file}
        ${CATKIN_ENV}
        ${CMAKE_CURRENT_SOURCE_DIR}/${file}.py)

endforeach()

catkin_add_nosetests(test_lineplasm.py)
catkin_add_nosetests(save_load_plasm_file.py DEPENDENCIES plasm_loader)
