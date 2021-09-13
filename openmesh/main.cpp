#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <cxxopts.hpp>
#include <ranges>
#include<functional>
#include<algorithm>
#include "mesh.h"

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>

void check_for_holes(const std::string& input_file) {
    auto a_mesh = mesh::load_mesh(input_file);
    SPDLOG_INFO("mesh read.");
    using Hole = std::vector<mesh::mesh_type::EdgeHandle>;
    std::vector<Hole> result;
    SPDLOG_INFO("faces: {0}; vertices: {1}", a_mesh.n_faces(), a_mesh.n_vertices());
    OpenMesh::EPropHandleT< bool > boundary;
    a_mesh.add_property(boundary, "boundary");
    // mark all edges as not part of a boundary
    std::for_each(a_mesh.edges_begin(), a_mesh.edges_end(),
                  [&a_mesh, boundary](const mesh::mesh_type::EdgeHandle& edge){
                      a_mesh.property(boundary, edge) = false;
                  });

    for(auto a_edge : a_mesh.edges()) {
        if(a_mesh.property(boundary, a_edge))
            continue;
        if(!a_mesh.is_boundary(a_edge))
            continue;
        mesh::mesh_type::HalfedgeHandle heh = a_mesh.halfedge_handle(a_edge,0);
        if(!a_mesh.is_boundary(heh))
            heh = a_mesh.opposite_halfedge_handle(heh);
        mesh::mesh_type::HalfedgeHandle current_heh = heh;
        Hole current_hole;
        do {
            mesh::mesh_type::EdgeHandle eh = a_mesh.edge_handle(current_heh);
            current_hole.push_back(eh);
            a_mesh.property(boundary,a_mesh.edge_handle(current_heh)) = true;
            int cnt = 0;
            mesh::mesh_type::VertexHandle vh = a_mesh.to_vertex_handle(current_heh);
            for (mesh::mesh_type::VertexOHalfedgeIter vhe_it(a_mesh, vh); vhe_it.is_valid(); ++vhe_it) {
                if(a_mesh.is_boundary(*vhe_it))
                    cnt++;
            }

            if(cnt > 1) {
                mesh::mesh_type::HalfedgeHandle opposit = a_mesh.opposite_halfedge_handle(current_heh);
                mesh::mesh_type::VertexOHalfedgeIter vohe_it(a_mesh, opposit);
                current_heh = *(vohe_it);
            } else
                current_heh = a_mesh.next_halfedge_handle(current_heh);

        } while(current_heh != heh);
        result.push_back(current_hole);
    }
    a_mesh.remove_property(boundary);
    std::sort(result.begin(), result.end(), [](const Hole& hole_one, const Hole& hole_two) { return hole_one.size() > hole_two.size();});
    SPDLOG_INFO("holes identified.");
    auto result_strings = std::ranges::transform_view(result, [&a_mesh](const auto& hole){
        std::string hole_string{};
        for(auto e : hole) {
            auto vh = a_mesh.from_vertex_handle(a_mesh.halfedge_handle(e,0));
            auto point = a_mesh.point(vh);
            hole_string += fmt::format("{0} {1} {2}\n",point[0], point[1], point[2]);
        }
        return hole_string;
    });
    SPDLOG_INFO("writing hole to stdout.");
    std::ranges::for_each(result_strings, [](const auto& item){ std::cout << item << "\n";});
}

void get_point_cloud(const std::string& input_file, const std::string& output_file) {
    auto a_mesh = mesh::load_mesh(input_file);
    SPDLOG_INFO("mesh read.");
    std::vector<mesh::mesh_type::Point> pts;
    std::ranges::transform(a_mesh.vertices(),
                           std::back_inserter(pts),
                          [&a_mesh](const auto& item){return a_mesh.point(item);});

    //auto v = std::ranges::transform_view(a_mesh.vertices(),
    //                       [&a_mesh](const auto& item)->mesh::mesh_type::Point {return a_mesh.point(item);});

    auto res = pts | std::views::transform([](const auto& item) {return pcl::PointXYZ(item[0],item[1],item[2]);});
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::ranges::copy(res,std::back_inserter(cloud));
    SPDLOG_INFO("cloud created.");
    pcl::io::savePLYFile(output_file, cloud);
}

void mesh_convert(const std::string& input_file, const std::string& output_file) {
    auto a_mesh = mesh::load_mesh(input_file);
    SPDLOG_INFO("mesh read.");
    mesh::store_mesh(a_mesh, output_file);
    SPDLOG_INFO("mesh written.");
}

int main(int argc, char** argv) {
    static auto console = spdlog::stdout_color_mt("console");
    spdlog::set_pattern("[%H:%M:%S.%e][%s][%!][%#] %v");
    SPDLOG_INFO("Application started.");
    cxxopts::Options options(argv[0], "see help");
    options.add_options()
            ("i,input","input file", cxxopts::value<std::string>())
            ("o,output","output file", cxxopts::value<std::string>())
            ("c,convert","convert file", cxxopts::value<bool>()->default_value("false"))
            ("p,pointcloud","extract point cloud", cxxopts::value<bool>()->default_value("false"))
            ("x,holes","get information about mesh holes", cxxopts::value<bool>()->default_value("false"))
            ("h,help", "Print help")
            ;

    auto result = options.parse(argc,argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        SPDLOG_INFO("done.");
        exit(0);
    }
    if(!result["convert"].has_default()) {
        mesh_convert(result["input"].as<std::string>(), result["output"].as<std::string>());
    }
    if(!result["pointcloud"].has_default()) {
        get_point_cloud(result["input"].as<std::string>(), result["output"].as<std::string>());
    }
    if(!result["holes"].has_default()) {
        check_for_holes(result["input"].as<std::string>());
    }
    SPDLOG_INFO("done.");
    return 0;
}
