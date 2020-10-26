/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OPENGL_URDF_VISUALIZER_H
#define OPENGL_URDF_VISUALIZER_H


#include <iostream>
#include <map>
#include <string>

#include "multi_body.hpp"
#include "urdf/urdf_structures.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "stb_image/stb_image.h"

template <typename Algebra>
struct OpenGLUrdfVisualizer {
  typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;
  typedef ::tds::UrdfLink<Algebra> TinyUrdfLink;
  typedef ::tds::UrdfVisual<Algebra> UrdfVisual;
  
  typedef ::tds::MultiBody<Algebra> TinyMultiBody;
  typedef tds::Transform<Algebra> Transform;
  using TinyVector3 = typename Algebra::Vector3;
  using TinyMatrix3 = typename Algebra::Matrix3;
  using Quaternion = typename Algebra::Quaternion;
  

  struct TinyVisualLinkInfo {
    std::string vis_name;
    int link_index;
    TinyVector3 origin_rpy;
    TinyVector3 origin_xyz;
    TinyVector3 inertia_xyz;
    TinyVector3 inertia_rpy;
    std::vector<int> instance_ids;
  };

  std::map<std::string, int> m_link_name_to_index;
  std::map<int, TinyVisualLinkInfo> m_b2vis;
  
  
  int m_uid;
  std::string m_texture_data;
  std::string m_texture_uuid;
  std::string m_path_prefix;

  TinyOpenGL3App m_opengl_app;

  OpenGLUrdfVisualizer(int width=1024, int height=768)
      : m_uid(1234) ,
      m_opengl_app("test", width, height)
  {
      
      m_opengl_app.m_renderer->init();
      m_opengl_app.set_up_axis(2);
      m_opengl_app.m_renderer->get_active_camera()->set_camera_distance(4);
      m_opengl_app.m_renderer->get_active_camera()->set_camera_pitch(-30);
      m_opengl_app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);
  }

  void delete_all() {
    //todo
  }

  void load_obj(const std::string& obj_filename, const ::TINY::TinyVector3f& pos, const ::TINY::TinyQuaternionf& orn, const ::TINY::TinyVector3f& scaling, std::vector<int>& instance_ids)
  {
      tinyobj::attrib_t attrib;
      std::vector<tinyobj::shape_t> shapes;
      std::vector<tinyobj::material_t> materials;

      //test Wavefront obj loading
      std::string warn;
      std::string err;
      char basepath[1024];
      bool triangulate = true;
      
      ::tds::FileUtils::extract_path(obj_filename.c_str(), basepath, 1024);
      bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, obj_filename.c_str(),
          basepath, triangulate);

      
      for (int i = 0; i < shapes.size(); i++)
      {
          std::vector<int> indices;
          std::vector<GfxVertexFormat1> vertices;
          int textureIndex = -1;
          TinyMeshUtils::extract_shape(attrib, shapes[i], materials, indices, vertices, textureIndex);
          textureIndex = -1;
          ::TINY::TinyVector3f color(1, 1, 1);
          if (shapes[i].mesh.material_ids.size())
          {
              const tinyobj::material_t& mat = materials[shapes[i].mesh.material_ids[0]];
              color.setValue(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
              if (mat.diffuse_texname.length())
              {
                  std::string texture_file_name = std::string(basepath) + mat.diffuse_texname;
                  std::vector< unsigned char> buffer;
                  int width, height, n;
                  unsigned char* image = stbi_load(texture_file_name.c_str(), &width, &height, &n, 3);

                  textureIndex = m_opengl_app.m_renderer->register_texture(image, width, height);
                  free(image);
              }
          }
          int shape = m_opengl_app.m_renderer->register_shape(&vertices[0].x, vertices.size(), &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex);
          int instance_id = m_opengl_app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);
          instance_ids.push_back(instance_id);
      }

      m_opengl_app.m_renderer->write_transforms();
  }
  
  void convert_link_visuals(TinyUrdfLink &link, int link_index,
                            bool useTextureUuid) {
    for (int vis_index = 0; vis_index < (int)link.urdf_visual_shapes.size();
         vis_index++) {
      UrdfVisual &v =
          link.urdf_visual_shapes[vis_index];

      printf("v.geom_type=%d", v.geometry.geom_type);
      std::string vis_name =
          std::string("/opengl/") + link.link_name + std::to_string(m_uid);
      TinyVisualLinkInfo b2v;
      b2v.vis_name = vis_name;
      b2v.link_index = link_index;
      b2v.origin_rpy = v.origin_rpy;
      b2v.origin_xyz = v.origin_xyz;
      b2v.inertia_xyz = link.urdf_inertial.origin_xyz;
      b2v.inertia_rpy = link.urdf_inertial.origin_rpy;
      int color_rgb = 0xffffff;
      double world_pos[3] = {0, 0, 0};
      if (v.geometry.geom_type == ::tds::TINY_MESH_TYPE) {
        // printf("mesh filename=%s\n", v.geom_meshfilename.c_str());
        std::string obj_filename;
        std::string org_obj_filename = m_path_prefix + v.geometry.mesh.file_name;
        if (::tds::FileUtils::find_file(org_obj_filename, obj_filename))
        {
            ::TINY::TinyVector3f pos(0, 0, 0);
            ::TINY::TinyVector3f scaling(v.geometry.mesh.scale[0], v.geometry.mesh.scale[1], v.geometry.mesh.scale[2]);
            ::TINY::TinyQuaternionf orn(0, 0, 0, 1);
            load_obj(obj_filename, pos, orn, scaling, b2v.instance_ids);
        }
      }
      v.sync_visual_body_uid1 = m_uid;
      m_b2vis[m_uid++] = b2v;
    }
  }

  void convert_visuals(TinyUrdfStructures &urdf,
                       const std::string &texture_path) {
    m_link_name_to_index.clear();
    {
      int link_index = -1;
      std::string link_name = urdf.base_links[0].link_name;
      m_link_name_to_index[link_name] = link_index;
      convert_link_visuals(urdf.base_links[0], link_index, false);
    }

    for (int link_index = 0; link_index < (int)urdf.links.size();
         link_index++) {
      std::string link_name = urdf.links[link_index].link_name;
      m_link_name_to_index[link_name] = link_index;
      convert_link_visuals(urdf.links[link_index], link_index, false);
    }
  }

  void sync_visual_transforms(const TinyMultiBody* body) {
      // sync base transform
      for (int v = 0; v < body->visual_ids().size(); v++) {
          int visual_id = body->visual_ids()[v];
          if (m_b2vis.find(visual_id) != m_b2vis.end()) {
              Quaternion rot;
              Transform geom_X_world =
                  body->base_X_world() * body->X_visuals()[v];

              const TinyMatrix3& m =
                  geom_X_world.rotation;
              m.getRotation(rot);
              const TinyVisualLinkInfo& viz = m_b2vis.at(visual_id);
              for (int i = 0; i < viz.instance_ids.size(); i++)
              {
                  int instance_id = viz.instance_ids[i];
                  ::TINY::TinyVector3f pos(geom_X_world.translation[0], geom_X_world.translation[1], geom_X_world.translation[2]);
                  ::TINY::TinyQuaternionf orn(rot[0], rot[1], rot[2], rot[3]);
                  m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, instance_id);
              }
          }
      }

      for (int l = 0; l < body->links().size(); l++) {
          for (int v = 0; v < body->links()[l].visual_ids.size(); v++) {
              int visual_id = body->links()[l].visual_ids[v];
              if (m_b2vis.find(visual_id) != m_b2vis.end()) {
                  Quaternion rot;
                  Transform geom_X_world =
                      body->links()[l].X_world * body->links()[l].X_visuals[v];
                  
                  TinyMatrix3& m =
                      geom_X_world.rotation;
                  const TinyVisualLinkInfo& viz = m_b2vis.at(visual_id);
                  m.getRotation(rot);
                  for (int i = 0; i < viz.instance_ids.size(); i++)
                  {
                      int instance_id = viz.instance_ids[i];
                      ::TINY::TinyVector3f pos(geom_X_world.translation[0], geom_X_world.translation[1], geom_X_world.translation[2]);
                      ::TINY::TinyQuaternionf orn(rot[0], rot[1], rot[2], rot[3]);
                      m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, instance_id);
                  }

              }
          }
      }
  }
  void render() {
    int upAxis = 2;
    m_opengl_app.m_renderer->write_transforms();
    m_opengl_app.m_renderer->update_camera(upAxis);
    double lightPos[3] = { -50, 30, 40 };
    m_opengl_app.m_renderer->set_light_position(lightPos);
    float specular[3] = { 1,1,1 };
    m_opengl_app.m_renderer->set_light_specular_intensity(specular);
    DrawGridData data;
    data.upAxis = 2;
    m_opengl_app.draw_grid(data);
    //const char* bla = "3d label";
    //m_opengl_app.draw_text_3d(bla, 0, 0, 1, 1);
    m_opengl_app.m_renderer->render_scene();
    m_opengl_app.swap_buffer();
 
  }
};

#endif  // OPENGL_URDF_VISUALIZER_H
