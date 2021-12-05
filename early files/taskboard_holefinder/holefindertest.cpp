
#include "STL/stl_reader.h"
#include <iostream>
std::vector<float> coords, normals;
std::vector<unsigned int> tris, solids;

int main(){
  //stl_reader::ReadStlFile ("IROS2020_Practice - IROS2020_Practice_Base-1.stl", coords, normals, tris, solids);
    
  stl_reader::StlMesh <float, unsigned int> mesh ("IROS2020_Practice - IROS2020_Practice_Base-1.STL");



  for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
      std::cout << "coordinates of triangle " << itri << ": ";
      for(size_t icorner = 0; icorner < 3; ++icorner) {
          const float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
          const float* d = mesh.tri_corner_coords (itri, icorner);
          // or alternatively:
          // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
          std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
      }
      std::cout << std::endl;
  
      const float* n = mesh.tri_normal (itri);
      std::cout   << "normal of triangle " << itri << ": "
                  << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
  }
return 0;
}