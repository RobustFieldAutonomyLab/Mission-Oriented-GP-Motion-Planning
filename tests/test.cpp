#include "Planning3DUUV.h"
#include "matplot/matplot.h"
void read_draw_path(string path_file, int num_lines_to_skip, string name,
                    std::vector<matplot::axes_object_handle> &handles,
                    std::vector<std::string> &names) {
    ifstream file(path_file);
    if (!file.is_open()) {
        cout << "Error opening file." << endl;
        return;
    }
    // Read the header information
    string line;
    for (int i = 0; i < num_lines_to_skip + 1; i++) {
        std::getline(file, line);
    }
    vector<double> X, Y, Z;
    while (getline(file, line)) {
        double tpx, tpy, tpz;
        std::sscanf(line.c_str(), "%lf, %lf, %lf", &tpx, &tpy, &tpz);
        X.push_back(tpx);
        Y.push_back(tpy);
        Z.push_back(tpz);
    }
    file.close();
    auto l = matplot::plot3(X, Y, Z);
    l->line_width(2);
    l->display_name(name);
    handles.push_back(l);
    names.push_back(name);
}

void draw_1(){
    // Open the file for reading
    string stomp10_file = "/home/rfal/darpa/GPMP_STR/data/str/no_mission/stomp/result_no_mission_10/2.txt";
    string stomp_file30 = "/home/rfal/darpa/GPMP_STR/data/str/no_mission/stomp/result_no_mission_30/2.txt";
    string GP_file = "/home/rfal/darpa/GPMP_STR/data/str/no_mission/gpmp/result_no_mission_10.txt";
    string GP_file30 = "/home/rfal/darpa/GPMP_STR/data/str/no_mission/gpmp/result_no_mission_30.txt";
    string RRT_file = "/home/rfal/darpa/GPMP_STR/data/str/no_mission/ompl/result_no_mission_no_object/0.txt";
    string RRT_file2 = "/home/rfal/darpa/GPMP_STR/data/str/no_mission/ompl/result_no_mission_object/0.txt";

//    string GP_file30 = "/home/rfal/darpa/GPMP_STR/data/lower_bay/cr/gpmp/result_50.txt";
//    string stomp_file30 = "/home/rfal/darpa/GPMP_STR/data/lower_bay/cr/stomp/result_mission_50/27.txt";
//    string stomp10_file = "/home/rfal/darpa/GPMP_STR/data/lower_bay/cr/stomp/result_mission_200/5.txt";
//    string GP_file = "/home/rfal/darpa/GPMP_STR/data/lower_bay/cr/gpmp/result_200.txt";
////    string RRT_file = "/home/rfal/darpa/GPMP_STR/data/NYC/sub_area/no_mission/ompl/result_no_mission_no_object/1.txt";
//    string RRT_file2 = "/home/rfal/darpa/GPMP_STR/data/lower_bay/cr/ompl/2.txt";

//    string GP_file30 = "/home/rfal/darpa/GPMP_STR/data/NYC/100/gpmp/result_30.txt";
//    string stomp_file30 = "/home/rfal/darpa/GPMP_STR/data/NYC/100/stomp/result_mission_30/27.txt";
//    string stomp10_file = "/home/rfal/darpa/GPMP_STR/data/NYC/100/stomp/result_mission_100/5.txt";
//    string GP_file = "/home/rfal/darpa/GPMP_STR/data/NYC/100/gpmp/result_100.txt";
//    string RRT_file2 = "/home/rfal/darpa/GPMP_STR/data/NYC/100/ompl/2.txt";

    // Read the trajectory points
//    Matrix data = loadSeaFloorData("/home/rfal/darpa/GPMP_STR/data/NYC/100/depth_grid_NYC_small.csv");
//    Matrix data = loadSeaFloorData("/home/rfal/darpa/GPMP_STR/data/lower_bay/cr/depth_grid_lower_baysub.csv");
    Matrix data = loadSeaFloorData("/home/rfal/darpa/GPMP_STR/data/str/depth_grid2.csv");
    std::vector<std::string> newcolors = {"#1c8c6b","#0f4c3a","#d8768b","#c73a59",
                                          "#fea64d","#fc8002","#0f4c3a","#c73a59"};

    matplot::colororder(newcolors);

    plotEvidenceMap3D(data, 0, 0, 1, MESH);
//    plotEvidenceMap3D(data, -200, -100, 1, DOWNSIZE_MESH);

    std::vector<matplot::axes_object_handle> lines;
    std::vector<std::string> legend_labels;

    read_draw_path(RRT_file, 4, "RRT* single", lines, legend_labels);
    read_draw_path(RRT_file2, 4, "RRT* multi", lines, legend_labels);
    read_draw_path(stomp_file30, 5, "STOMP 30", lines, legend_labels);
    read_draw_path(stomp10_file, 5, "STOMP 10", lines, legend_labels);
    read_draw_path(GP_file30, 4, "MGPMP 30", lines, legend_labels);
    read_draw_path(GP_file, 4, "MGPMP 10", lines, legend_labels);

    std::vector<double> X{5}, Y{5}, Z{-4220};
//    std::vector<double> X{20}, Y{20}, Z{-2};
//    std::vector<double> X{40}, Y{20}, Z{-12};
    auto s = matplot::scatter3(X, Y, Z, "filled");
    s->line_width(1.5);
    s->display_name("start");
    s->marker_style("^");
    s->marker_size(20);

    std::vector<double> x{45}, y{45}, z{-4182};
//    std::vector<double> x{980}, y{980}, z{-2};
//    std::vector<double> x{50}, y{100}, z{-12};
    auto s1 = matplot::scatter3(x, y, z, "filled");
    s1->line_width(1.5);
    s1->display_name("goal");
    s1->marker_style("^");
    s1->marker_size(20);
    // show a partial legend with only one line
    matplot::legend(lines, legend_labels);
    matplot::yticks({0, 5, 10, 15, 20, 25, 30, 35, 40, 45});
    matplot::show();
}

int main() {
    draw_1();
    return 0;
}

//int main(){

//}