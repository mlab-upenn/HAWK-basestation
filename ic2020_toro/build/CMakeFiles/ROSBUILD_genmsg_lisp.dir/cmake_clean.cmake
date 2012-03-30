FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ic2020_toro/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/newedge.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_newedge.lisp"
  "../msg_gen/lisp/loopnotice.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_loopnotice.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
