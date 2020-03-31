#include <boost/version.hpp>
#include <stdio.h>

int main(int argc, char** argv)
{
  printf("%u.%u.%u",
         BOOST_VERSION / 100000, (BOOST_VERSION / 100) % 100, BOOST_VERSION  % 100);
}
