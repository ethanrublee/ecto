// 
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
#include <ecto/schedulers/singlethreaded.hpp>
#include <ecto/schedulers/multithreaded.hpp>

namespace bp = boost::python;

namespace ecto {
  namespace py {

    template <typename T> int execute0 (T& s) { return s.execute(); }
    template <typename T> int execute1 (T& s, unsigned arg1) { return s.execute(arg1); }
    template <typename T> int execute2 (T& s, unsigned arg1, unsigned arg2) { return s.execute(arg1, arg2); }

    template <typename T> void execute_async0 (T& s) { return s.execute_async(); }
    template <typename T> void execute_async1 (T& s, unsigned arg1) { return s.execute_async(arg1); }
    template <typename T> void execute_async2 (T& s, unsigned arg1, unsigned arg2) 
    { 
      return s.execute_async(arg1, arg2); 
    }

    template <typename T> 
    void wrap_scheduler(const char* name)
    {
      using bp::arg;
      bp::class_<T, boost::noncopyable>(name, bp::init<ecto::plasm::ptr>())
        .def("execute", &execute0<T>)
        .def("execute", &execute1<T>, arg("niter"))
        .def("execute", &execute2<T>, (arg("niter"), arg("nthreads")))

        .def("execute_async", &execute_async0<T>)
        .def("execute_async", &execute_async1<T>, arg("niter"))
        .def("execute_async", &execute_async2<T>, (arg("niter"), arg("nthreads")))

        .def("interrupt", &T::interrupt)
        .def("stop", &T::stop)
        .def("running", (bool (scheduler::*)() const) &scheduler::running)
        .def("wait", &T::wait)
        .def("stats", &T::stats)
        ;
    }

    void wrapSchedulers()
    {
      //      bp::detail::init_module("ecto.schedulers", initschedulers);
      bp::object schedulers_module(bp::handle<>(bp::borrowed(PyImport_AddModule("ecto.schedulers"))));
      bp::scope().attr("schedulers") = schedulers_module;
      bp::scope schedulers_scope = schedulers_module;
      
      using namespace ecto::schedulers;
      using bp::arg;

      //      wrap_scheduler<singlethreaded>("Singlethreaded");

      wrap_scheduler<singlethreaded>("Singlethreaded");

      wrap_scheduler<multithreaded>("Multithreaded");
        
    }
  }
}

