// Requirements-traceability instrumentation for GoogleTest, same shape as
// the companion flight-software-test-harness project's req_trace.hpp:
// TRACE_REQ("REQ-NNN") inside a TEST() body records that this test verifies
// that requirement; a listener collects tag + pass/fail per test and writes
// them to JSON at program exit. tools/gen_traceability.py merges this with
// the end-to-end driver scripts' own trace JSON (see driver/common.py) and
// cross-references both against requirements.yaml.
#pragma once
#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace cbs_test {

struct TraceRecord {
    std::vector<std::string> requirement_ids;
    bool passed = false;
};

inline std::map<std::string, TraceRecord>& trace_registry() {
    static std::map<std::string, TraceRecord> registry;
    return registry;
}

inline std::vector<std::string>& current_test_requirements() {
    static std::vector<std::string> current;
    return current;
}

inline void record_requirement(const std::string& req_id) {
    current_test_requirements().push_back(req_id);
}

class RequirementTraceListener : public ::testing::EmptyTestEventListener {
public:
    explicit RequirementTraceListener(std::string output_path) : output_path_(std::move(output_path)) {}

    void OnTestStart(const ::testing::TestInfo&) override { current_test_requirements().clear(); }

    void OnTestEnd(const ::testing::TestInfo& info) override {
        std::string full_name = std::string(info.test_suite_name()) + "." + info.name();
        TraceRecord rec;
        rec.requirement_ids = current_test_requirements();
        rec.passed = info.result()->Passed();
        trace_registry()[full_name] = rec;
    }

    void OnTestProgramEnd(const ::testing::UnitTest&) override {
        std::ofstream out(output_path_);
        out << "[\n";
        bool first_record = true;
        for (const auto& [name, rec] : trace_registry()) {
            if (!first_record) out << ",\n";
            first_record = false;
            out << "  {\"test\": \"" << name << "\", \"passed\": " << (rec.passed ? "true" : "false")
                << ", \"requirements\": [";
            for (size_t i = 0; i < rec.requirement_ids.size(); ++i) {
                if (i) out << ", ";
                out << "\"" << rec.requirement_ids[i] << "\"";
            }
            out << "]}";
        }
        out << "\n]\n";
    }

private:
    std::string output_path_;
};

}  // namespace cbs_test

#define TRACE_REQ(id) ::cbs_test::record_requirement(id)
