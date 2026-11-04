# thirdparty/sp_vision — vendored detection core

Copied from `rmUAV/ob_ws/src/sp_vision_25` at commit **`bd9f5e7`**
("Update star history chart sources for improved token security").

`sp_vision_25` is an untracked directory in this repo, and it is a complete
RoboMaster gimbal application: its top-level `CMakeLists.txt` sets a global
`CMAKE_BUILD_TYPE`, hard-requires OpenVINO at a fixed path, pulls in vendor
camera SDKs (`io/`), and links `tinympc`/Ceres through `auto_aim` and
`auto_buff`. It cannot be embedded as a catkin subdirectory.

So only the stateless detection core is copied here, and the copies are patched
to compile as a ROS 1 package. This file is the record of the fork: if upstream
moves, the diffs below are what has to be re-applied.

## What is copied

| File | Why |
|---|---|
| `tasks/auto_aim/armor.{hpp,cpp}` | `Lightbar` / `Armor` data structures and geometry |
| `tasks/auto_aim/detector.{hpp,cpp}` | Traditional light-bar detector (threshold → contours → pairing) |
| `tasks/auto_aim/classifier.{hpp,cpp}` | 32×32 digit classifier via `cv::dnn` |
| `tools/math_tools.{hpp,cpp}` | `eulers`, `limit_rad`, `xyz2ypd`, … |
| `tools/img_tools.{hpp,cpp}` | Debug drawing |
| `tools/extended_kalman_filter.{hpp,cpp}` | EKF used by `armor_perception::StaticTarget` |

## What is deliberately not copied

`solver.*`, `target.*`, `tracker.*` — all three assume a gimbal bolted to the
field:

- `solver.cpp` computes `xyz_in_world = R_gimbal2world_ * xyz_in_gimbal` with no
  translation term, which is silently wrong on a translating vehicle;
- `target.cpp` models a plate rotating on a robot's yaw axis (state
  `[x vx y vy z vz a w r l h]`) and declares divergence when the unobservable
  radius `r` leaves `[0.05, 0.5]` m, which for a static plate fires at random;
- `tracker.cpp` drags in `omniperception` → `io/usbcamera` → vendor SDKs.

`armor_perception` provides `ArmorSolver`, `StaticTarget` and `Tracker`
replacements. `Tracker` keeps upstream's acquire/lose state machine, which is
the part that was worth having.

Also not copied: `aimer`, `shooter`, `planner/`, `multithread/`, `voter`,
`yolo*`, `omniperception/`, `io/`, `auto_buff/`, `calibration/`, `src/`.

## Patches, file by file

Every patch is marked `PATCH(armor_perception)` in the source. Summary:

### `detector.cpp` / `detector.hpp`

1. **`cv::imshow("binary_img", …)` removed.** Upstream called it unconditionally
   inside `detect()`, outside the `debug_` branch — a ROS node would pop a GUI
   window. Also `show_result()` (which called `cv::imshow` + `cv::resize` for
   display) is replaced by `draw_result()`, a pure function returning the
   annotated image; `detect()` optionally writes it to an out-parameter.
2. **`std::filesystem::create_directory("patterns")` made opt-in.** Upstream
   created a directory relative to CWD on every construction. Now driven by the
   `save_patterns` key, empty (disabled) by default.
3. **`use_classifier` added.** Upstream always ran the digit classifier and both
   `check_name()` and `check_type()` depended on `armor.name`. With no model
   configured the classifier returns `not_armor` for everything, so upstream's
   checks would reject every detection. The flag skips those two checks and lets
   geometry decide; it defaults to "on only if `classify_model` is set".
4. **`fmt` / `spdlog` removed**, replaced by `std::ostringstream` and
   `ROS_*` logging, so the package does not need those two libraries.
5. `#include <numeric>` added — upstream relied on a transitive include for
   `std::accumulate`.

### `classifier.cpp` / `classifier.hpp`

6. **Lazy loading.** Upstream's constructor did `cv::dnn::readNetFromONNX` *and*
   `ov::Core::compile_model`, so every `Detector` paid the OpenVINO cost and
   needed OpenVINO headers even on the traditional light-bar path. Now the
   constructor only records the model path; `classify()` loads `cv::dnn` on first
   use. `ovclassify()` is compiled only under `ARMOR_USE_OPENVINO`.

### `armor.hpp` / `armor.cpp`

7. **Members default-initialized, `Armor()` default constructor added.**
   Upstream left `type/name/priority/class_id/confidence/box` uninitialized in
   the traditional `Armor(Lightbar, Lightbar)` constructor and relied on
   `classifier_.classify()` to fill `name`. With the classifier disabled those
   fields were read uninitialized — UB, and `check_type()` plus priority sorting
   became nondeterministic. `Lightbar`'s default constructor had the same issue.
8. **`R_armor2world` added.** Upstream kept only `ypr_in_world`, which discards
   roll and cannot be turned back into a rotation without re-imposing the
   "armor on a vertical cylinder, pitch ±15°" assumption. The observation point
   needs the true plate normal, so the full rotation is stored.

### `tools/extended_kalman_filter.cpp`

9. **Residual indexing guarded.** Upstream wrote `residual[0..3]` unconditionally
   because its only user had a 4-D measurement. `StaticTarget` uses a 3-D
   position measurement, where `residual[3]` is out of bounds.
10. **NIS/NEES threshold derived from measurement dimension.** Upstream used a
    hardcoded `0.711` (documented as "χ², 4 DOF, 95%", which is wrong for 4 DOF
    anyway). Now the χ² 95% table is indexed by the residual size.

### `tools/math_tools.cpp`

11. **Brace-initialised matrices replaced with comma initialisers.** Eigen 3.3
    (Ubuntu 20.04 / Noetic) does not support row-brace initialisation; that
    arrived in Eigen 3.4. Affects `rotation_matrix`, `xyz2ypd_jacobian`,
    `ypd2xyz_jacobian`.

## Merging upstream changes

There is no automated path. Diff the upstream files against these copies, and
re-apply each `PATCH(armor_perception)` hunk by hand. The detector and the armor
structures are the only files likely to change; the rest are utilities that have
been stable.
