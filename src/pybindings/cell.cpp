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
#include <ecto/log.hpp>
#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/iterator.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/python/std_map_indexing_suite.hpp>
#include <ecto/python/raw_constructor.hpp>
#include <ecto/python/repr.hpp>
#include <string>
namespace bp = boost::python;
#include "tendril_spec.hpp"

namespace ecto
{
  void inspect_impl(ecto::cell_ptr m, const boost::python::tuple& args,
                    const boost::python::dict& kwargs);

  namespace py
  {
    std::string type_name(const cell& c)
    {
      return c.type();
    }
    const tendrils& inputs(cell& mod)
    {
      return mod.inputs;
    }
    tendrils& outputs(cell& mod)
    {
      return mod.outputs;
    }
    tendrils& params(cell& mod)
    {
      return mod.parameters;
    }

    void wrapCell()
    {
      //use private names so that python people know these are internal
      bp::class_<cell, boost::shared_ptr<cell>, boost::noncopyable>("_cell_cpp", bp::no_init)
        .def("type", type_name)
        .def("typename",type_name)
        .def("configure", ((void(cell::*)()) &cell::configure))
        .def("_set_strand", &cell::set_strand)
        .def("_reset_strand", &cell::reset_strand)
        .def("construct", &inspect_impl)
        .def("declare_params", &cell::declare_params)
        .def("declare_io", ((void(cell::*)()) &cell::declare_io))
        .def("configure", ((void(cell::*)()) &cell::configure))
        .def("process", (void(cell::*)()) &cell::process)
        .def("start", (void(cell::*)()) &cell::start)
        .def("stop", (void(cell::*)()) &cell::stop)
        .def("verify_params", &cell::verify_params)
        .def("verify_inputs", &cell::verify_inputs)
        .add_property("inputs", make_function(inputs, bp::return_internal_reference<>()))
        .add_property("outputs", make_function(outputs, bp::return_internal_reference<>()))
        .add_property("params", make_function(params, bp::return_internal_reference<>()))
        .def("name",(((std::string(cell::*)() const) &cell::name)))
        .def("name",(((void(cell::*)(const std::string&)) &cell::name)))
        .def("short_doc",(std::string(cell::*)() const) &cell::short_doc)
        .def("gen_doc", &cell::gen_doc)
        .def("__getitem__", getitem_str)
        .def("__getitem__", getitem_tuple)
        .def("__getitem__", getitem_list)
        .def("__getitem__", getitem_slice)
        ;

      bp::def("__getitem_str__", getitem_str);
      bp::def("__getitem_slice__", getitem_slice);
      bp::def("__getitem_tuple__", getitem_tuple);
      bp::def("__getitem_list__", getitem_list);

      bp::class_<TendrilSpecification>("TendrilSpecification")
        .def_readwrite("module_input", &TendrilSpecification::mod_input)
        .def_readwrite("module_output", &TendrilSpecification::mod_output)
        .def_readwrite("key", &TendrilSpecification::key)
        .def("to_tendril",&TendrilSpecification::toTendril)
        ;

      bp::class_<TendrilSpecifications>("TendrilSpecifications", bp::init<bp::list>())
        .def("to_tendrils", &TendrilSpecifications::toTendrils)
        .staticmethod("to_tendrils")
        .def("to_spec", &TendrilSpecifications::toSpec)
        .def("__rshift__", rshift_spec)
        .def("__rshift__", rshift_spec_tuples)
        ;

      bp::enum_<tendril_type>("tendril_type")
        .value("INPUT",INPUT)
        .value("OUTPUT",OUTPUT)
        .value("PARAMETER",PARAMETER)
        .export_values()
        ;
      bp::enum_<ecto::ReturnCode>("ReturnCode")
        .value("OK",OK)
        .value("QUIT",QUIT)
        .export_values()
        ;
    }
  }

  void inspect_impl(ecto::cell_ptr m, const boost::python::tuple& args,
                    const boost::python::dict& kwargs)
  {

    if (bp::len(args) > 1)
      throw std::runtime_error("Only one non-keyword argument allowed, this will specify instance name");

    if (bp::len(args) == 0)
      {
        // generate default name == type
        m->name(m->type());
      }
    else
      {
        bp::extract<std::string> e(args[0]);
        if (! e.check())
          throw std::runtime_error("Non-keyword argument (instance name) not convertible to string.");
        m->name(e());
      }
    m->declare_params();

    ECTO_LOG_DEBUG("inspect_impl %s", m->name());

    bp::list l = kwargs.items();
    for (int j = 0; j < bp::len(l); ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        if (keystring == "strand")
          {
            ecto::strand s = bp::extract<ecto::strand>(value);
            m->strand_ = s;
            ECTO_LOG_DEBUG("Found a strand for cell %s, id=%p", m->name() % s.id());
          }
        else
          {
            tendril_ptr tp = m->parameters[keystring];
            try{
              *tp << value;
            }catch(ecto::except::TypeMismatch& e)
              {
                e << except::tendril_key(keystring);
                e << except::cell_name(m->name());
                throw;
              }
            tp->user_supplied(true);
            tp->dirty(true);
          }
      }
    m->declare_io();
  }
}

