#include <ecto/parameters.hpp>
#include <ecto/impl/parameters.hpp>
#include <boost/preprocessor/seq/for_each.hpp>


namespace ecto
{

#define ECTO_INSTANTIATE_BOUNDED(r, data, T) \
  template struct bounded<T>;

  BOOST_PP_SEQ_FOR_EACH(ECTO_INSTANTIATE_BOUNDED, _, ECTO_COMMON_TYPES);

}
