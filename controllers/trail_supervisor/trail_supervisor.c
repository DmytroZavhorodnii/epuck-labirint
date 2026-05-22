#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#define TIME_STEP  64
#define TRAIL_STEP 8   /* co ile kroków stawiać kropkę */
#define DOT_R      0.005

int main(void) {
  wb_robot_init();

  WbNodeRef root = wb_supervisor_node_get_root();
  WbFieldRef root_children = wb_supervisor_node_get_field(root, "children");

  WbNodeRef r1 = wb_supervisor_node_get_from_def("EPUCK1");
  WbNodeRef r2 = wb_supervisor_node_get_from_def("EPUCK2");

  WbFieldRef f1 = wb_supervisor_node_get_field(r1, "translation");
  WbFieldRef f2 = wb_supervisor_node_get_field(r2, "translation");

  int step = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    if (++step % TRAIL_STEP != 0) continue;

    const double *p1 = wb_supervisor_field_get_sf_vec3f(f1);
    const double *p2 = wb_supervisor_field_get_sf_vec3f(f2);

    char buf[512];

    snprintf(buf, sizeof(buf),
      "Solid { translation %.4f %.4f 0.001 "
      "children [ Shape { appearance PBRAppearance { "
      "baseColor 1 0 0 metalness 0 roughness 1 } "
      "geometry Sphere { radius %.3f } } ] }",
      p1[0], p1[1], DOT_R);
    wb_supervisor_field_import_mf_node_from_string(root_children, -1, buf);

    snprintf(buf, sizeof(buf),
      "Solid { translation %.4f %.4f 0.001 "
      "children [ Shape { appearance PBRAppearance { "
      "baseColor 0 0.4 1 metalness 0 roughness 1 } "
      "geometry Sphere { radius %.3f } } ] }",
      p2[0], p2[1], DOT_R);
    wb_supervisor_field_import_mf_node_from_string(root_children, -1, buf);
  }

  wb_robot_cleanup();
  return 0;
}
