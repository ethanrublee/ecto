/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
//boost python first.
#include <boost/python.hpp>
//boost stuff
#include <boost/shared_ptr.hpp>

//do not include this in ecto lib files, only in client modules

//ecto includes
#include <ecto/module.hpp>
#include <ecto/tendril.hpp>
#include <ecto/plasm.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/util.hpp>
#include <ecto/python/raw_constructor.hpp>

/**
 * \namespace ecto
 * \brief ecto is ...
 *
 * ecto is a plugin architecture / pipeline tool based on boost::python.
 */
namespace ecto
{
/**
 * \internal
 */
template<typename T>
struct doc
{
  static std::string doc_str; //!<a doc string for humans to read in python.
  static std::string name; //!< the name for this type.
  //! get the doc
  static std::string getDoc()
  {
    return doc_str;
  }
  //! get the name
  static std::string getName()
  {
    return name;
  }
};
template<typename T>
std::string doc<T>::doc_str;
template<typename T>
std::string doc<T>::name;

template<typename T>
boost::shared_ptr<ecto::module_<T> > raw_construct(boost::python::tuple args,
    boost::python::dict kwargs)
{
  typedef ecto::module_<T> module_t;

  namespace bp = boost::python;

  //SHOW();
  boost::shared_ptr<module_t> m(new module_t());
  m->module::initialize();
  bp::list l = kwargs.items();
  for (unsigned j = 0; j < bp::len(l); ++j)
  {
    bp::object key = l[j][0];
    bp::object value = l[j][1];
    std::string keystring = bp::extract<std::string>(key);
    std::string valstring = bp::extract<std::string>(value.attr("__repr__")());
    m->parameters.at(keystring).set(value);
  }
  m->module::configure();
  return m;
}

template<typename T>
void wrap(const char* name, std::string doc_str = "A module...")
{
  typedef ecto::module_<T> module_t;
  //SHOW();
  boost::python::class_<module_t, boost::python::bases<module>, boost::shared_ptr<module_t>,
      boost::noncopyable> m(name);
  typedef doc<T> docT;
  docT::name = name;
  docT::doc_str = doc_str;

  m.def("__init__", boost::python::raw_constructor(&raw_construct<T> ));
  m.def("doc", &docT::getDoc) .staticmethod("doc");
  m.def("name", &docT::getName);
  m.staticmethod("name");
}
}

