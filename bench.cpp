#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

static void BM_concat(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        cld += single;
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_emplace(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;
    for (auto _: state) {
        for(int i = 0; i < count; ++i)
        cld.points.emplace_back(pt);
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_old(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        if (single.header.stamp > cld.header.stamp) {
            cld.header.stamp = single.header.stamp;
        }
        std::size_t nr_points = cld.size();
        cld.points.resize(nr_points + single.points.size());
        for (auto i = nr_points; i < cld.points.size(); ++i) {
            cld.points[i] = single[i-nr_points];
        }
        cld.width = static_cast<uint32_t>(cld.points.size());
        cld.height = 1;
        if (single.is_dense && cld.is_dense)
            cld.is_dense = true;
        else
            cld.is_dense = false;
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_resize_raw(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        cld.header.stamp = std::max(cld.header.stamp, single.header.stamp);

        std::size_t nr_points = cld.size();
        cld.points.resize(cld.size() + single.points.size());
        for (auto i = nr_points; i < cld.points.size(); ++i) {
            cld.points[i] = single[i-nr_points];
        }
        cld.width = static_cast<uint32_t>(cld.points.size());
        cld.height = 1;
        cld.is_dense &= single.is_dense;
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_resize_copy(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        cld.header.stamp = std::max(cld.header.stamp, single.header.stamp);

        std::size_t nr_points = cld.size();
        cld.points.resize(cld.size() + single.points.size());
        std::copy(single.points.begin(), single.points.end(), cld.points.begin() + nr_points);
        cld.width = static_cast<uint32_t>(cld.points.size());
        cld.height = 1;
        cld.is_dense &= single.is_dense;
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_reserve_raw(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        cld.header.stamp = std::max(cld.header.stamp, single.header.stamp);

        std::size_t nr_points = cld.size();
        cld.points.reserve(cld.size() + single.points.size());
        for (auto i = 0; i < single.points.size(); ++i) {
            cld.points.emplace_back (single[i]);
        }
        cld.width = static_cast<uint32_t>(cld.points.size());
        cld.height = 1;
        cld.is_dense &= single.is_dense;
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_reserve_insert(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        cld.header.stamp = std::max(cld.header.stamp, single.header.stamp);

        std::size_t nr_points = cld.size();
        cld.points.reserve(cld.size() + single.points.size());
        cld.points.insert(cld.points.end(), single.points.begin(), single.points.end());
        cld.width = static_cast<uint32_t>(cld.points.size());
        cld.height = 1;
        cld.is_dense &= single.is_dense;
        benchmark::DoNotOptimize(cld);
    }
}

static void BM_insert(benchmark::State& state) {
    const int count = state.range(0);
    pcl::PointCloud<pcl::PointXYZRGB> cld;
    pcl::PointXYZRGB pt;

    pt.x = pt.y = pt.z = 1;
    pt.r = pt.g = pt.b = 128;

    pcl::PointCloud<pcl::PointXYZRGB> single;
    single.width = count; single.height = 1;
    single.is_dense = false;
    for (int i = 0; i < count; ++i)
    single.points.push_back(pt);
    for (auto _: state) {
        cld.header.stamp = std::max(cld.header.stamp, single.header.stamp);

        std::size_t nr_points = cld.size();
        cld.points.insert(cld.points.end(), single.points.begin(), single.points.end());
        cld.width = static_cast<uint32_t>(cld.points.size());
        cld.height = 1;
        cld.is_dense &= single.is_dense;
        benchmark::DoNotOptimize(cld);
    }
}

BENCHMARK(BM_emplace)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_old)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_concat)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_resize_raw)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_resize_copy)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_reserve_raw)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_reserve_insert)->RangeMultiplier(10)->Range(1, 10000);
BENCHMARK(BM_insert)->RangeMultiplier(10)->Range(1, 10000);

BENCHMARK_MAIN();
