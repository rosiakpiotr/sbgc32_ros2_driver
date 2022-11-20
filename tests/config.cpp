#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace boost::unit_test;

struct Config {
  Config() {
    BOOST_TEST_MESSAGE("START");
    unit_test_log.set_threshold_level(log_all_errors);
  }

  ~Config() { BOOST_TEST_MESSAGE("END"); }
};

BOOST_GLOBAL_FIXTURE(Config);
