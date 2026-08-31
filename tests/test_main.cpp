#include <gtest/gtest.h>

#include <string>

#include "req_trace.hpp"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    std::string output_path = "test_trace.json";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        const std::string prefix = "--trace_output=";
        if (arg.rfind(prefix, 0) == 0) output_path = arg.substr(prefix.size());
    }

    ::testing::UnitTest::GetInstance()->listeners().Append(new cbs_test::RequirementTraceListener(output_path));
    return RUN_ALL_TESTS();
}
