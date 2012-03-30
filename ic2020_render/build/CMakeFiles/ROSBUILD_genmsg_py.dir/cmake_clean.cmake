FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ic2020_render/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ic2020_render/msg/__init__.py"
  "../src/ic2020_render/msg/_rendupdate.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
