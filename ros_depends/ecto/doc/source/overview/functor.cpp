#include <iostream>
#include <string>
//start
struct Printer
{
  Printer(const std::string& prefix, const std::string& suffix)
      :
        prefix_(prefix),
        suffix_(suffix)
  {
  }
  void
  operator()(std::ostream& out, const std::string& message)
  {
    out << prefix_ << message << suffix_;
  }
  std::string prefix_, suffix_;
};
//end

int
main()
{
  Printer p("start>> ", "<< stop\n");
  std::string message = "Hello Function.";
  p(std::cout, message);
  return 0;
}
