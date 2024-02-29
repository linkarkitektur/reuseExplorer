#pragma once
#include <string>
#include <types/PointCloud.hh>

namespace linkml {
    PointCloud::Ptr merge_files(const std::string& input_dir, const std::string& output_file, int chunk_size=500);
}