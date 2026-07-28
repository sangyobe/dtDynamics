#pragma once

#include <dtDynamics/dtArray.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#ifndef DTDYNAMICS_BENCHMARK_BUILD_TYPE
#define DTDYNAMICS_BENCHMARK_BUILD_TYPE "unknown"
#endif

namespace dt::Dynamics::benchmark
{

struct ScalarSummary
{
    double min = 0.0;
    double median = 0.0;
    double max = 0.0;
};

inline std::string SanitizeCsvValue(const std::string &value)
{
    std::string sanitized = value;
    for (char &ch : sanitized)
    {
        if (ch == ',' || ch == '\n' || ch == '\r')
        {
            ch = ' ';
        }
    }
    return sanitized.empty() ? "unknown" : sanitized;
}

inline std::string FirstCpuModel()
{
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line))
    {
        const std::string key = "model name";
        const std::size_t keyPos = line.find(key);
        if (keyPos == std::string::npos)
        {
            continue;
        }

        const std::size_t colon = line.find(':', keyPos + key.size());
        if (colon == std::string::npos)
        {
            continue;
        }

        std::size_t valueBegin = colon + 1;
        while (valueBegin < line.size() && line[valueBegin] == ' ')
        {
            ++valueBegin;
        }
        return SanitizeCsvValue(line.substr(valueBegin));
    }
    return "unknown";
}

inline std::string Compiler()
{
#if defined(__clang__)
    return std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
    return std::string("gcc ") + __VERSION__;
#elif defined(_MSC_VER)
    return std::string("msvc ") + std::to_string(_MSC_VER);
#else
    return "unknown";
#endif
}

inline std::string Platform()
{
#if defined(__linux__)
    return "linux";
#elif defined(_WIN32)
    return "windows";
#elif defined(__APPLE__)
    return "macos";
#else
    return "unknown";
#endif
}

inline void PrintMetadataRow(std::ostream &out, const char *key, const std::string &value)
{
    out << "benchmark_metadata," << key << ',' << SanitizeCsvValue(value) << '\n';
}

inline bool BenchmarkEmitSamples()
{
    const char *value = std::getenv("DTDYNAMICS_BENCHMARK_EMIT_SAMPLES");
    return value != nullptr && value[0] != '\0' && std::string(value) != "0";
}

inline void PrintSampleHeader(std::ostream &out)
{
    out << "benchmark_sample,executable,engine_or_stage,fixture,state,quantity_scope_or_stage,sample_index,runtime_us,alloc_count\n";
}

inline void PrintSampleRow(std::ostream &out,
                           const char *benchmarkName,
                           const std::string &engineOrStage,
                           const std::string &fixture,
                           const std::string &state,
                           const std::string &scopeOrStage,
                           const int sampleIndex,
                           const double runtimeUs,
                           const unsigned long long allocationCount)
{
    out << "benchmark_sample,"
        << SanitizeCsvValue(benchmarkName) << ','
        << SanitizeCsvValue(engineOrStage) << ','
        << SanitizeCsvValue(fixture) << ','
        << SanitizeCsvValue(state) << ','
        << SanitizeCsvValue(scopeOrStage) << ','
        << sampleIndex << ','
        << runtimeUs << ','
        << allocationCount << '\n';
}

inline int BenchmarkBatchCount()
{
    const char *value = std::getenv("DTDYNAMICS_BENCHMARK_BATCHES");
    if (value == nullptr || value[0] == '\0')
    {
        return 1;
    }

    char *end = nullptr;
    const long requested = std::strtol(value, &end, 10);
    if (end == value || requested <= 1)
    {
        return 1;
    }

    constexpr long kMaxBatchCount = 100;
    return static_cast<int>(std::min(requested, kMaxBatchCount));
}

inline ScalarSummary SummarizeScalars(const dtArray<double> &samples)
{
    if (samples.empty())
    {
        return {};
    }

    dtArray<double> sorted;
    sorted.resize(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        sorted[i] = samples[i];
    }
    std::sort(sorted.begin(), sorted.end());

    ScalarSummary summary;
    summary.min = sorted[0];
    summary.median = sorted[sorted.size() / 2];
    summary.max = sorted[sorted.size() - 1];
    return summary;
}

inline void PrintBatchSummaryHeader(std::ostream &out)
{
    out << "benchmark_batch_summary,benchmark,fixture,state,stage,batches,"
           "mean_us_min,mean_us_median,mean_us_max,"
           "median_us_min,median_us_median,median_us_max,"
           "p95_us_min,p95_us_median,p95_us_max,"
           "p99_us_min,p99_us_median,p99_us_max,allocation_max_max\n";
}

inline std::string BuildBatchSummaryRow(const char *benchmarkName,
                                        const std::string &fixture,
                                        const std::string &state,
                                        const std::string &stage,
                                        const int batchCount,
                                        const dtArray<double> &meanUs,
                                        const dtArray<double> &medianUs,
                                        const dtArray<double> &p95Us,
                                        const dtArray<double> &p99Us,
                                        const unsigned long long allocationMax)
{
    const ScalarSummary meanSummary = SummarizeScalars(meanUs);
    const ScalarSummary medianSummary = SummarizeScalars(medianUs);
    const ScalarSummary p95Summary = SummarizeScalars(p95Us);
    const ScalarSummary p99Summary = SummarizeScalars(p99Us);

    std::ostringstream out;
    out << "benchmark_batch_summary,"
        << benchmarkName << ','
        << SanitizeCsvValue(fixture) << ','
        << SanitizeCsvValue(state) << ','
        << SanitizeCsvValue(stage) << ','
        << batchCount << ','
        << meanSummary.min << ','
        << meanSummary.median << ','
        << meanSummary.max << ','
        << medianSummary.min << ','
        << medianSummary.median << ','
        << medianSummary.max << ','
        << p95Summary.min << ','
        << p95Summary.median << ','
        << p95Summary.max << ','
        << p99Summary.min << ','
        << p99Summary.median << ','
        << p99Summary.max << ','
        << allocationMax;
    return out.str();
}

inline void PrintBenchmarkMetadata(std::ostream &out,
                                   const char *benchmarkName,
                                   const int warmupIterations,
                                   const int measuredIterations)
{
    out << "benchmark_metadata,key,value\n";
    PrintMetadataRow(out, "benchmark", benchmarkName);
    PrintMetadataRow(out, "build_type", DTDYNAMICS_BENCHMARK_BUILD_TYPE);
    PrintMetadataRow(out, "compiler", Compiler());
    PrintMetadataRow(out, "cxx_standard", std::to_string(__cplusplus));
    PrintMetadataRow(out, "platform", Platform());
    PrintMetadataRow(out, "cpu_model", FirstCpuModel());
    PrintMetadataRow(out, "hardware_concurrency", std::to_string(std::thread::hardware_concurrency()));
    PrintMetadataRow(out, "clock", "std::chrono::steady_clock");
    PrintMetadataRow(out, "clock_is_steady", std::chrono::steady_clock::is_steady ? "true" : "false");
    PrintMetadataRow(out, "warmup_iterations", std::to_string(warmupIterations));
    PrintMetadataRow(out, "measured_iterations", std::to_string(measuredIterations));
    PrintMetadataRow(out, "batch_count", std::to_string(BenchmarkBatchCount()));
#ifdef NDEBUG
    PrintMetadataRow(out, "ndebug", "true");
#else
    PrintMetadataRow(out, "ndebug", "false");
#endif
}

} // namespace dt::Dynamics::benchmark
