#include "fusion/io/results_csv.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fusion {
namespace {

// Reads a whole text file into a vector of lines.
std::vector<std::string> read_lines(const std::string& path) {
  std::ifstream file(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    lines.push_back(line);
  }
  return lines;
}

std::string temp_path(const std::string& name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

TEST(ResultsCsv, WritesHeaderAndOneLinePerRow) {
  const std::string path = temp_path("fusion_test_rows.csv");
  const std::vector<ResultRow> rows(3);  // three rows of zeros
  ASSERT_TRUE(write_results_csv(path, rows));

  const auto lines = read_lines(path);
  ASSERT_EQ(lines.size(), 4u);  // header + 3 rows
  EXPECT_EQ(lines[0], kResultsCsvHeader);
}

TEST(ResultsCsv, WritesValuesInHeaderOrder) {
  const std::string path = temp_path("fusion_test_values.csv");
  ResultRow row;
  row.t = 1.5;
  row.true_x = 2;
  row.true_y = 3;
  row.has_gnss = true;
  row.gnss_x = 4;
  row.gnss_y = 5;
  row.est_x = 6;
  row.est_y = 7;
  row.est_vx = 8;
  row.est_vy = 9;
  row.sigma_x = 10;
  row.sigma_y = 11;
  ASSERT_TRUE(write_results_csv(path, {row}));

  const auto lines = read_lines(path);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[1], "1.5,2,3,1,4,5,6,7,8,9,10,11");
}

TEST(ResultsCsv, WritesZeroForMissingGnss) {
  const std::string path = temp_path("fusion_test_nognss.csv");
  ResultRow row;
  row.has_gnss = false;
  ASSERT_TRUE(write_results_csv(path, {row}));
  EXPECT_EQ(read_lines(path)[1], "0,0,0,0,0,0,0,0,0,0,0,0");
}

TEST(ResultsCsv, ReturnsFalseWhenFileCannotBeOpened) {
  EXPECT_FALSE(write_results_csv("/this/folder/does/not/exist/out.csv", {}));
}

}  // namespace
}  // namespace fusion
