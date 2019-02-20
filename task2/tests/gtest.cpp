#include <gtest/gtest.h>
#include "logger.hpp"

TEST(SquareRootTest, PositiveNos) { 
    Logger mylogger();
    ASSERT_EQ(36, mylogger.write_log_to_file(36.0));
    ASSERT_EQ(36.01, mylogger.write_log_to_file(36.01));
    ASSERT_EQ("Hello word", mylogger.write_log_to_file("Hello word"));
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


