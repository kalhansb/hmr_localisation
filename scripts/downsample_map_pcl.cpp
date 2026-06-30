// Voxel-downsample a localization map cloud (x,y,z,intensity) using PCL's own
// filters. Produces the lighter .pcd map that gt_ouster_ndt_realtime.yaml loads.
//
//   voxel   (default) -> pcl::VoxelGrid       : one CENTROID (averaged synthetic
//                                                point) per occupied voxel.
//   uniform           -> pcl::UniformSampling : keeps the REAL input point closest
//                                                to each voxel centre (no averaging;
//                                                keeps real points, not centroids).
//
// VoxelGrid indexes voxels with int32, so on a building-scale map a very small
// leaf overflows (dx*dy*dz > INT32_MAX): PCL then returns the cloud UNFILTERED.
// We pre-compute that product and print an explicit warning so the no-op is
// never silent.
//
// Build (inside the Jazzy container, libpcl-dev + g++ are present):
//   g++ -O2 -std=c++17 scripts/downsample_map_pcl.cpp -o downsample_map_pcl \
//       $(pkg-config --cflags --libs pcl_common pcl_io pcl_filters) \
//       -lpcl_kdtree -lpcl_search -lpcl_octree
//
// Usage:
//   ./downsample_map_pcl <in.{ply,pcd}> <out.{ply,pcd}> <leaf_m> [voxel|uniform]
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

static bool ends_with(const std::string& s, const std::string& suf) {
  return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

static int load_cloud(const std::string& path, Cloud& c) {
  if (ends_with(path, ".pcd")) return pcl::io::loadPCDFile(path, c);
  if (ends_with(path, ".ply")) return pcl::io::loadPLYFile(path, c);
  std::cerr << "unsupported input extension (use .pcd or .ply): " << path << "\n";
  return -1;
}

static int save_cloud(const std::string& path, const Cloud& c) {
  if (ends_with(path, ".pcd")) return pcl::io::savePCDFileBinary(path, c);
  if (ends_with(path, ".ply")) return pcl::io::savePLYFileBinary(path, c);
  std::cerr << "unsupported output extension (use .pcd or .ply): " << path << "\n";
  return -1;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0]
              << " <in.{ply,pcd}> <out.{ply,pcd}> <leaf_m> [voxel|uniform]\n";
    return 2;
  }
  const std::string in = argv[1], out = argv[2];
  const float leaf = std::stof(argv[3]);
  const std::string method = (argc > 4) ? argv[4] : "voxel";

  Cloud::Ptr cloud(new Cloud);
  if (load_cloud(in, *cloud) == -1) { std::cerr << "failed to load " << in << "\n"; return 1; }
  const std::size_t n_in = cloud->size();

  pcl::PointXYZI mn, mx;
  pcl::getMinMax3D(*cloud, mn, mx);
  const double ex = mx.x - mn.x, ey = mx.y - mn.y, ez = mx.z - mn.z;

  Cloud::Ptr filtered(new Cloud);
  if (method == "uniform") {
    pcl::UniformSampling<pcl::PointXYZI> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(leaf);
    us.filter(*filtered);
  } else {  // voxel (centroid)
    // Replicate PCL's int32 voxel-index guard so the overflow no-op is explicit.
    const auto i64 = [](double v) { return static_cast<std::int64_t>(v); };
    const std::int64_t dx = i64(ex / leaf) + 1, dy = i64(ey / leaf) + 1, dz = i64(ez / leaf) + 1;
    const std::int64_t cells = dx * dy * dz;
    if (cells > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
      std::cerr << "WARNING: leaf " << leaf << " m over extent "
                << ex << "x" << ey << "x" << ez << " m needs " << dx << "*" << dy << "*"
                << dz << " = " << cells << " voxels > INT32_MAX ("
                << std::numeric_limits<std::int32_t>::max()
                << "). pcl::VoxelGrid will return the cloud UNFILTERED.\n";
    }
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*filtered);
  }

  const std::size_t n_out = filtered->size();
  if (save_cloud(out, *filtered) == -1) { std::cerr << "failed to save " << out << "\n"; return 1; }

  const bool noop = (n_out == n_in);
  std::cout << in << ": " << n_in << " pts -> " << out << ": " << n_out << " pts"
            << "  (method=" << method << ", leaf=" << leaf << " m, "
            << (100.0 * static_cast<double>(n_out) / static_cast<double>(n_in)) << "%"
            << (noop ? ", NO-OP/overflow" : "") << ")\n";
  std::cout << "  extent: " << ex << " x " << ey << " x " << ez << " m\n";
  return 0;
}
