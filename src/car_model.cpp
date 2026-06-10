#include "car_model.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace cgp;

namespace {

struct obj_index {
    int position = -1;
    int uv = -1;
    int normal = -1;
};

obj_index parse_obj_index(std::string const& token)
{
    obj_index index;
    size_t const slash0 = token.find('/');
    if (slash0 == std::string::npos) {
        index.position = std::stoi(token) - 1;
        return index;
    }

    index.position = std::stoi(token.substr(0, slash0)) - 1;
    size_t const slash1 = token.find('/', slash0 + 1);
    if (slash1 == std::string::npos) {
        std::string const uv = token.substr(slash0 + 1);
        if (!uv.empty())
            index.uv = std::stoi(uv) - 1;
        return index;
    }

    std::string const uv = token.substr(slash0 + 1, slash1 - slash0 - 1);
    std::string const normal = token.substr(slash1 + 1);
    if (!uv.empty())
        index.uv = std::stoi(uv) - 1;
    if (!normal.empty())
        index.normal = std::stoi(normal) - 1;
    return index;
}

void append_obj_triangle(mesh& target, std::array<obj_index, 3> const& triangle, std::vector<vec3> const& positions, std::vector<vec2> const& uvs, std::vector<vec3> const& normals)
{
    uint3 connectivity;
    for (int k = 0; k < 3; ++k) {
        obj_index const& index = triangle[k];
        connectivity[k] = int(target.position.size());
        target.position.push_back(positions[index.position]);
        target.uv.push_back(index.uv >= 0 ? uvs[index.uv] : vec2{0.0f, 0.0f});
        target.normal.push_back(index.normal >= 0 ? normals[index.normal] : vec3{0.0f, 1.0f, 0.0f});
    }
    target.connectivity.push_back(connectivity);
}

void transform_car_mesh(mesh& car_mesh, vec3 const& center, float scale)
{
    car_mesh.translate(-center);
    car_mesh.scale(scale);
    car_mesh.rotate({0.0f, 1.0f, 0.0f}, Pi / 2.0f);
    car_mesh.fill_empty_field();
}

}

car_model_meshes create_car_model_meshes(std::string const& obj_path, float visual_length)
{
    std::ifstream stream(obj_path);
    assert_cgp(stream.is_open(), "Cannot open car.obj");

    std::vector<vec3> positions;
    std::vector<vec2> uvs;
    std::vector<vec3> normals;
    car_model_meshes model;

    bool use_window_mesh = false;
    std::string line;
    while (std::getline(stream, line)) {
        std::istringstream line_stream(line);
        std::string command;
        line_stream >> command;

        if (command == "v") {
            vec3 p;
            line_stream >> p.x >> p.y >> p.z;
            positions.push_back(p);
        }
        else if (command == "vt") {
            vec2 uv;
            line_stream >> uv.x >> uv.y;
            uvs.push_back(uv);
        }
        else if (command == "vn") {
            vec3 n;
            line_stream >> n.x >> n.y >> n.z;
            normals.push_back(n);
        }
        else if (command == "usemtl") {
            std::string material;
            line_stream >> material;
            use_window_mesh = material == "Window";
        }
        else if (command == "f") {
            std::vector<obj_index> face;
            std::string token;
            while (line_stream >> token)
                face.push_back(parse_obj_index(token));

            mesh& target = use_window_mesh ? model.windows : model.body;
            for (size_t k = 1; k + 1 < face.size(); ++k)
                append_obj_triangle(target, {face[0], face[k], face[k + 1]}, positions, uvs, normals);
        }
    }

    mesh full_mesh;
    full_mesh.push_back(model.body);
    full_mesh.push_back(model.windows);
    full_mesh.fill_empty_field();

    vec3 p_min;
    vec3 p_max;
    full_mesh.get_bounding_box_position(p_min, p_max);
    vec3 const extent = p_max - p_min;
    float const source_length = std::max(extent.z, 1e-5f);
    vec3 const center = 0.5f * (p_min + p_max);
    float const scale = visual_length / source_length;

    transform_car_mesh(model.body, center, scale);
    transform_car_mesh(model.windows, center, scale);
    return model;
}
