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

#include <Python.h>
#include <ecto/forward.hpp>
#include <ecto/log.hpp>
#include <ecto/strand.hpp>
#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/traits.hpp>
#include <ecto/util.hpp>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace ecto
{
  /** \brief Return values for cell::process().
   * TODO: Should these live in cell?
   * These are appropriate for non exceptional behavior.
   */
  enum ReturnCode
  {
    OK       =  0, //!< Everything A OK.
    QUIT     =  1, //!< Explicit quit now.
    DO_OVER  =  2, //!< This modules' process call needs to be made again
    BREAK    =  3, //!< Stop execution in my scope, jump to outer scope
    CONTINUE =  4, //!< Stop execution in my scope, jump to top of scope
    UNKNOWN  = -1  //!< Unknown return code.
  };

#define ECTO_RETURN_VALUES                        \
    (OK)(QUIT)(DO_OVER)(BREAK)(CONTINUE)(UNKNOWN) \

  const std::string&
  ReturnCodeToStr(int rval);

  /** \brief ecto::cell is the non virtual interface to the basic building
   * block of ecto graphs.  This interface should never be the parent of
   * client cell, but may be used for polymorphic access to client cells.
   *
   * Clients should expose their code to this interface through
   * ecto::wrap, or ecto::create_cell<T>().
   *
   * For a client's cell to satisfy the ecto::cell idiom, it must
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
   * All functions are optional.
   */
  struct ECTO_EXPORT cell: boost::noncopyable
  {
    typedef boost::shared_ptr<cell> ptr; //!< A convenience pointer typedef

    cell();
    virtual ~cell();

    /** \brief Dispatches parameter declaration code. After this code,
     * the parameters for the cell will be set to their defaults.
     */
    void declare_params();
    /** \brief Dispatches input/output declaration code.  It is assumed that
     * the parameters have been declared before this is called, so that inputs
     * and outputs may be dependent on those parameters.
     */
    void declare_io();

    /** \brief Given initialized parameters,inputs, and outputs, this will
     * dispatch the client configuration code.  This will allocated an instace
     * of the clients cell, so this should not be called during introspection.
     */
    void configure();

    /** \brief Activate the cell. i.e. Put it into a "ready" state,
     * opening sockets, etc.
     */
    void activate();

    /** \brief Deactivate the cell. i.e. Put it into an "unready" state,
     * closing sockets, etc.
     */
    void deactivate();

    /** scheduler is going to call process() zero or more times. */
    void start();

    /** scheduler is not going to call process() for a while. */
    void stop();

    /** \brief Dispatches the process function for the client cell.
     * This should only be called from one thread at a time.
     * Also, this function may throw exceptions...
     *
     * @return A return code, ecto::OK , or 0 means all is ok.
     * Anything non zero should be considered an exit signal.
     */
    ReturnCode process();
    ReturnCode process_with_only_these_inputs(const tendrils& connected_inputs);

    /** \brief Return the type of the child class.
     * @return A human readable non mangled name for the client class.
     */
    std::string type() const { return dispatch_name(); }

    /** \brief Grab the name of the instance.
     * @return The name of the instance, or the address if none was given
     *   when object was constructed.
     */
    std::string name() const
    { return instance_name_.size() ? instance_name_ : dispatch_name(); }

    /** \brief Set the name of the instance. */
    void name(const std::string& name)
    { instance_name_ = name; }

    /** \brief Set the short_doc_ of the instance. */
    std::string short_doc() const
    { return dispatch_short_doc(); }

    /** \brief Set the short_doc_ of the instance. */
    void short_doc(const std::string& short_doc)
    { dispatch_short_doc(short_doc); }

    void reset_strand();
    void set_strand(ecto::strand);

    /** \brief Generate an Restructured Text doc string for the cell.
     * Includes documentation for all parameters, inputs, outputs.
     * @param doc The highest level documentation for the cell.
     * @return A nicely formatted doc string.
     */
    std::string gen_doc(const std::string& doc = "A module...") const;

    void verify_params() const;
    void verify_inputs() const;

    /**
     * \brief Processing mode query - only connected tendrils or all.
     * Typically used by the scheduler to arrange input tendrils when
     * invoking a process.
     * @return flag only connected input tendrils if true, all otherwise.
     */
    bool process_connected_inputs_only() const {
      return process_connected_inputs_only_;
    }
    void set_process_connected_inputs_only(const bool &value) {
      process_connected_inputs_only_ = value;
    }

    ptr clone() const;

    //! Parameters
    tendrils parameters;
    //! Inputs, inboxes, always have a valid value ( may be NULL )
    tendrils inputs;
    //! Outputs, outboxes, always have a valid value ( may be NULL )
    tendrils outputs;

    //! The strand that this cell should be executed in.
    boost::optional<strand> strand_;

    virtual bool init() = 0;

  protected:

    virtual void dispatch_declare_params(tendrils& t) = 0;

    virtual void dispatch_declare_io(const tendrils& params, tendrils& inputs,
                                     tendrils& outputs) = 0;

    virtual void dispatch_configure(const tendrils& params,
                                    const tendrils& inputs,
                                    const tendrils& outputs) = 0;

    virtual void dispatch_activate() = 0;

    virtual void dispatch_deactivate() = 0;

    virtual ReturnCode dispatch_process(const tendrils& inputs,
                                        const tendrils& outputs) = 0;

    virtual void dispatch_start() = 0;
    virtual void dispatch_stop() = 0;

    virtual std::string dispatch_name() const = 0;

    virtual ptr dispatch_clone() const = 0;

    virtual std::string dispatch_short_doc() const
    { return std::string(); }

    virtual void dispatch_short_doc(const std::string&) { }

  private:
    cell(const cell&);

    std::string instance_name_;
    bool configured_;
    bool activated_;
    bool process_connected_inputs_only_;

  }; // cell


  /** \brief Helper class for determining if client modules have function
   * implementations or not.
   * @internal
   */
  template<class T>
  struct has_f
  {
    typedef char yes;
    typedef char (&no)[2];

    // SFINAE eliminates this when the type of arg is invalid
    // overload resolution prefers anything at all over "..."
    template<class U>
    static yes test_declare_params(__typeof__(&U::declare_params));
    template<class U>
    static no test_declare_params(...);
    enum
    {
      declare_params = sizeof(test_declare_params<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_declare_io(__typeof__(&U::declare_io));
    template<class U>
    static no test_declare_io(...);
    enum
    {
      declare_io = sizeof(test_declare_io<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_configure(__typeof__(&U::configure));
    template<class U>
    static no test_configure(...);
    enum
    {
      configure = sizeof(test_configure<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_activate(__typeof__(&U::activate));
    template<class U>
    static no test_activate(...);
    enum
    {
      activate = sizeof(test_activate<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_deactivate(__typeof__(&U::deactivate));
    template<class U>
    static no test_deactivate(...);
    enum
    {
      deactivate = sizeof(test_deactivate<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_process(__typeof__(&U::process));
    template<class U>
    static no test_process(...);
    enum
    {
      process = sizeof(test_process<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_start(__typeof__(&U::start));
    template<class U>
    static no test_start(...);
    enum
    {
      start = sizeof(test_start<T> (0)) == sizeof(yes)
    };

    template<class U>
    static yes test_stop(__typeof__(&U::stop));
    template<class U>
    static no test_stop(...);
    enum
    {
      stop = sizeof(test_stop<T> (0)) == sizeof(yes)
    };
  }; // has_f

  /** \brief cell_<T> is for registering an arbitrary class with the the cell
   * NVI. This adds a barrier between client code and the cell.
   */
  template<class Impl>
  struct cell_: cell
  {
    typedef boost::shared_ptr<cell_<Impl> > ptr; //convience type def

    cell_()
    { init_strand(typename ecto::detail::is_threadsafe<Impl>::type()); }

    ~cell_() {
      // Make sure all cells in the plasm are deactivated prior to exiting.
      dispatch_deactivate();
    }
    template <int I> struct int_ { };
    typedef int_<0> not_implemented;
    typedef int_<1> implemented;

    // declare_params
    typedef int_<has_f<Impl>::declare_params> has_declare_params;

    static void declare_params(tendrils& params, not_implemented) { }

    static void declare_params(tendrils& params, implemented)
    { Impl::declare_params(params); }

    static void declare_params(tendrils& params)
    { declare_params(params, has_declare_params()); }

    void dispatch_declare_params(tendrils& params)
    { declare_params(params); }


    // declare_io
    static void declare_io(const tendrils& params, tendrils& inputs,
                           tendrils& outputs, not_implemented)
    { }

    static void declare_io(const tendrils& params, tendrils& inputs,
                           tendrils& outputs, implemented)
    { Impl::declare_io(params, inputs, outputs); }

    typedef int_<has_f<Impl>::declare_io> has_declare_io;

    static void declare_io(const tendrils& params, tendrils& inputs,
                           tendrils& outputs)
    { declare_io(params, inputs, outputs, has_declare_io()); }

    void dispatch_declare_io(const tendrils& params, tendrils& inputs,
                             tendrils& outputs)
    { declare_io(params, inputs, outputs); }

    // configure
    void configure(const tendrils&, const tendrils& , const tendrils&,
                   not_implemented)
    { }

    void configure(const tendrils& params, const tendrils& inputs,
                   const tendrils& outputs, implemented)
    { impl_->configure(params,inputs,outputs); }

    void dispatch_configure(const tendrils& params, const tendrils& inputs,
                            const tendrils& outputs)
    { configure(params, inputs, outputs, int_<has_f<Impl>::configure> ()); }

    // activate
    void activate(not_implemented) { }

    void activate(implemented)
    { if(impl_) impl_->activate(); }

    void dispatch_activate()
    { activate(int_<has_f<Impl>::activate> ()); }

    // deactivate
    void deactivate(not_implemented) { }

    void deactivate(implemented)
    { if(impl_) impl_->deactivate(); }

    void dispatch_deactivate()
    { deactivate(int_<has_f<Impl>::deactivate> ()); }

    // process
    ReturnCode process(const tendrils&, const tendrils&, not_implemented)
    { return OK; }

    ReturnCode process(const tendrils& inputs, const tendrils& outputs,
                       implemented)
    { return ReturnCode(impl_->process(inputs, outputs)); }

    ReturnCode dispatch_process(const tendrils& inputs,
                                const tendrils& outputs)
    { return process(inputs, outputs, int_<has_f<Impl>::process> ()); }

    // start
    void start(not_implemented) { }
    void start(implemented) { impl_->start(); }
    void dispatch_start()
    { start(int_<has_f<Impl>::start> ()); }

    // stop
    void stop(not_implemented) { }
    void stop(implemented) { impl_->stop(); }
    void dispatch_stop() { stop(int_<has_f<Impl>::stop> ()); }

    std::string dispatch_name() const { return CELL_TYPE_NAME; }
    std::string dispatch_short_doc() const { return SHORT_DOC; }
    void dispatch_short_doc(const std::string&) { }

    cell::ptr dispatch_clone() const
    { return cell::ptr(new cell_<Impl> ()); }

    bool init();
  public:
    static std::string SHORT_DOC;
    //! The python name for the cell.
    static std::string CELL_NAME;
    //! The module that the cell is part of.
    static std::string MODULE_NAME;
    static const std::string CELL_TYPE_NAME;
    //! Grab a typed reference to the implementation of the cell.
    Impl& impl() {
      ECTO_ASSERT(impl_, "impl is null, call configure first");
      return *impl_;
    }
    const Impl& impl() const {
      ECTO_ASSERT(impl_, "impl is null, call configure first");
      return *impl_;
    }

  private:
    boost::scoped_ptr<Impl> impl_;
    void init_strand(boost::mpl::true_) { } // threadsafe

    void init_strand(boost::mpl::false_) {
      static ecto::strand strand_;
      cell::strand_ = strand_;
      ECTO_ASSERT(cell::strand_->id() == strand_.id(), "Catastrophe... cells not correctly assignable");
      ECTO_LOG_DEBUG("%s cell has strand id=%p", cell::type() % cell::strand_->id());
    }
  }; // cell_

  template<typename Impl>
  bool cell_<Impl>::init()
  {
    try {
      if(!impl_)
      {
        impl_.reset(new Impl);
        Impl* i=impl_.get();
        //these handle finalizing the registration of spores that
        //were registered at static time.
        parameters.realize_potential(i);
        inputs.realize_potential(i);
        outputs.realize_potential(i);
      }
      return static_cast<bool>(impl_);
    } catch (const std::exception& e) {
      ECTO_TRACE_EXCEPTION("const std::exception&");
      BOOST_THROW_EXCEPTION(except::CellException()
                            << except::when("Construction")
                            << except::type(name_of(typeid(e)))
                            << except::cell_name(name())
                            << except::what(e.what()));
    } catch (...) {
      ECTO_TRACE_EXCEPTION("...");
      BOOST_THROW_EXCEPTION(except::CellException()
                            << except::when("Construction")
                            << except::what("(unknown exception)")
                            << except::cell_name(name()));
    }
  }

  template<typename Impl>
  std::string cell_<Impl>::SHORT_DOC;

  template<typename Impl>
  std::string cell_<Impl>::CELL_NAME;

  template<typename Impl>
  std::string cell_<Impl>::MODULE_NAME;

  template<typename Impl>
  const std::string cell_<Impl>::CELL_TYPE_NAME = ecto::name_of<Impl>();

} // End of namespace ecto.
