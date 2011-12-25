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

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/any.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <ecto/forward.hpp>
#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/strand.hpp>
#include <ecto/util.hpp>
#include <ecto/profile.hpp>
#include <ecto/traits.hpp>

namespace ecto
{

  /**
   * \brief Return values for modules' process functions. These
   * are appropriate for non exceptional behavior.
   */
  enum ReturnCode
  {
    OK = 0, //!< Everything A OK.
    QUIT = 1, //!< Explicit quit now.
    BREAK = 2, //!< Stop execution in my scope, jump to outer scope
    CONTINUE = 3, //!< Stop execution in my scope, jump to top of scope
    UNKNOWN = -1
  //!< Unknown return code.
  };

#define ECTO_RETURN_VALUES                                                \
    (OK)(QUIT)(CONTINUE)(BREAK)(UNKNOWN)                                  \

  const std::string&
  ReturnCodeToStr(int rval);

  struct cell_factory;
  struct sigs
  {
    typedef void
    declare_params(tendrils&);
    typedef void
    declare_io(const tendrils&, tendrils&, tendrils&);
    typedef void
    configure(const tendrils&, const tendrils&, const tendrils&);
    typedef int
    process(const tendrils&, const tendrils&);
    typedef void
    start(void);
    typedef void
    stop(void);

    typedef boost::function<declare_params> fn_declare_params;
    typedef boost::function<declare_io> fn_declare_io;
    typedef boost::function<configure> fn_configure;
    typedef boost::function<process> fn_process;
    typedef boost::function<start> fn_start;
    typedef boost::function<stop> fn_stop;

#define REG_DECLARE_GET_F(FncName)                                             \
    struct get_f_##FncName##_                                                  \
    {                                                                          \
      template<class U,typename Sig>                                           \
      static boost::function<Sig> has_f(__decltype(&U::FncName))               \
          {return boost::function<Sig>(&U::FncName);}                          \
      template<class U,typename Sig>                                           \
      static boost::function<Sig> has_f(...)                                   \
          {return boost::function<Sig>();}                                     \
    }

#define REG_GET_F(Type,FncName,Signature)                                      \
    ecto::sigs::get_f_##FncName##_::has_f<Type,Signature>(0)

    REG_DECLARE_GET_F(declare_params); //
    REG_DECLARE_GET_F(declare_io); //
    REG_DECLARE_GET_F(configure); //
    REG_DECLARE_GET_F(process); //
    REG_DECLARE_GET_F(start); //
    REG_DECLARE_GET_F(stop);
    //
  };

  /**
   * \brief ecto::cell is the non virtual interface to the basic building
   * block of ecto graphs.  This interface should never be the parent of
   * client cell, but may be used for polymorphic access to client cells.
   *
   * Clients should expose their code to this interface through
   * ecto::wrap, or ecto::create_cell<T>().
   *
   * For a client's cell to satisfy the ecto::cell idium, it must
   * look similar to the following definition.
   * @code
   struct MyEctoCell
   {
   //called first thing, the user should declare their parameters in this
   //free standing function.
   static void declare_params(tendrils& params);
   //declare inputs and outputs here. The parameters may be used to
   //determine the io
   static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
   //called right after allocation of the cell, exactly once.
   void configure(tendrils& params, tendrils& inputs, tendrils& outputs);
   //called at every execution of the graph
   int process(const tendrils& in, tendrils& out);
   };
   * @endcode
   *
   * It is important to note that all functions have are optional and they all have
   * default implementations.
   */
  struct ECTO_EXPORT cell: boost::noncopyable
  {
    typedef boost::shared_ptr<cell> ptr; //!< A convenience pointer typedef

    cell(const cell_factory* factory);

    /**
     * \brief Dispatches parameter declaration code. After this code, the parameters
     * for the cell will be set to their defaults.
     */
    void
    declare_params();
    /**
     * \brief Dispatches input/output declaration code.  It is assumed that the parameters
     * have been declared before this is called, so that inputs and outputs may be dependent
     * on those parameters.
     */
    void
    declare_io();

    /**
     * \brief Given initialized parameters,inputs, and outputs, this will dispatch the client
     * configuration code.  This will allocated an instace of the clients cell, so this
     * should not be called during introspection.
     */
    void
    configure();

    /**
     scheduler is going to call process() zero or more times.
     */
    void
    start();

    /**
     scheduler is not going to call process() for a while.
     */
    void
    stop();

    /**
     * \brief Dispatches the process function for the client cell.  This should only
     * be called from one thread at a time.
     *
     * Also, this function may throw exceptions...
     *
     * @return A return code, ecto::OK , or 0 means all is ok. Anything non zero should be considered an
     * exit signal.
     */
    ReturnCode
    process();

    /**
     * \brief Return the type of the child class.
     * @return A human readable non mangled name for the client class.
     */
    const std::string&
    type() const;

    /**
     * \brief Grab the name of the instance.
     * @return The name of the instance, or the address if none was given when object was constructed
     */
    std::string
    name() const;

    /**
     * \brief Set the name of the instance.
     */
    void
    name(const std::string&);

    /**
     * \brief Set the short_doc_ of the instance.
     */
    std::string
    short_doc() const;

    /**
     * \brief Set the short_doc_ of the instance.
     */
    void
    short_doc(const std::string&);

    void
    reset_strand();
    void
    set_strand(ecto::strand);

    /**
     * \brief Generate an Restructured Text doc string for the cell. Includes documentation for all parameters,
     * inputs, outputs.
     * @param doc The highest level documentation for the cell.
     * @return A nicely formatted doc string.
     */
    std::string
    gen_doc(const std::string& doc = "A module...") const;

    void
    verify_params() const;
    void
    verify_inputs() const;

    ptr
    clone() const;

    tendrils parameters; //!< Parameters
    tendrils inputs; //!< Inputs, inboxes, always have a valid value ( may be NULL )
    tendrils outputs; //!< Outputs, outboxes, always have a valid value ( may be NULL )

    boost::optional<strand> strand_; //!< The strand that this cell should be executed in.
    profile::stats_type stats; //!< For collecting execution statistics for process.

    std::size_t
    tick() const;
    void
    inc_tick();
    void
    reset_tick();

    bool
    stop_requested() const
    {
      return stop_requested_;
    }
    void
    stop_requested(bool b)
    {
      stop_requested_ = b;
    }

    boost::signals2::signal<void
    (cell&, bool)> bsig_process;

  private:
    void
    init();
    std::string instance_name_;
    std::string short_doc_;
    bool stop_requested_;
    bool configured;
    std::size_t tick_;
    boost::mutex mtx;

#if defined(ECTO_STRESS_TEST)
    boost::mutex process_mtx;
#endif

    friend struct ecto::schedulers::access;
  public:
    const cell_factory* factory_;
    boost::any holder_;
    sigs::fn_declare_params declare_params_;
    sigs::fn_declare_io declare_io_;
    sigs::fn_configure configure_;
    sigs::fn_process process_;
    sigs::fn_start start_;
    sigs::fn_stop stop_;
  };

  struct cell_factory
  {
    virtual
    ~cell_factory();

    boost::shared_ptr<cell>
    create() const;

    virtual
    void
    populate_member(cell& c) const = 0;

    virtual
    void
    populate_static(cell& c) const = 0;

    virtual const std::string&
    type() const = 0;

    virtual const char* name() const = 0;
    virtual const char* docstring() const = 0;
  };

  template<class CellT>
  struct cell_factory_: cell_factory
  {
    typedef boost::shared_ptr<CellT> PtrT;

    //static functions.
    sigs::fn_declare_params declare_params;
    sigs::fn_declare_io declare_io;

    //member functions, have different signatures...
    //these must be handled with boost::bind
    typedef void
    sig_configure(CellT*, const tendrils&, const tendrils&, const tendrils&);
    typedef int
    sig_process(CellT*, const tendrils&, const tendrils&);
    typedef void
    sig_start(CellT*);
    typedef void
    sig_stop(CellT*);
    boost::function<sig_configure> configure;
    boost::function<sig_process> process;
    boost::function<sig_start> start;
    boost::function<sig_stop> stop;
    mutable const char* name_;
    mutable const char* docstring_;

    cell_factory_()
        :
          declare_params(REG_GET_F(CellT,declare_params,sigs::declare_params)),
          declare_io(REG_GET_F(CellT,declare_io,sigs::declare_io)),
          configure(REG_GET_F(CellT,configure,sig_configure)),
          process(REG_GET_F(CellT,process,sig_process)),
          start(REG_GET_F(CellT,start,sig_start)),
          stop(REG_GET_F(CellT,stop,sig_stop)),
          name_(name_of<CellT>().c_str()),
          docstring_("No docs...")
    {
    }

    void
    populate_member(cell& c) const
    {
      PtrT p;
      p.reset(new CellT);

      //these handle finalizing the registration of spores that
      //were registered at static time.

      c.holder_ = p;
      c.parameters.realize_potential(p.get());
      c.inputs.realize_potential(p.get());
      c.outputs.realize_potential(p.get());
      if (configure)
        c.configure_ = boost::bind(configure, p.get(), _1, _2, _3);
      if (process)
        c.process_ = boost::bind(process, p.get(), _1, _2);
      if (stop)
        c.stop_ = boost::bind(stop, p.get());
      if (start)
        c.start_ = boost::bind(start, p.get());
    }

    void
    populate_static(cell& c) const
    {
      if(!typename ecto::detail::is_threadsafe<CellT>::type())
      {
        static ecto::strand strand_;
        c.strand_ = strand_;
        ECTO_LOG_DEBUG("%s cell has strand id=%p", c.type() % c.strand_->id());
      }
      c.declare_params_ = declare_params;
      c.declare_io_ = declare_io;
    }

    static PtrT
    extract(cell& c)
    {
      return boost::any_cast<PtrT>(c.holder_);
    }

    static cell_ptr
    create()
    {
      return instance()->create();
    }

    static const cell_factory*
    instance()
    {
      static cell_factory_ cf;
      return &cf;
    }
    const std::string&
    type() const
    {
      return name_of<CellT>();
    }
    const char* name() const
    {
      return name_;
    }
    const char* docstring() const
    {
      return docstring_;
    }
  };

}//namespace ecto

