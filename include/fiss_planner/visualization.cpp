/** visualization.cpp
 *
 * Methods for visualization in rviz
 */

#include <tf/tf.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "frenet.h"
#include "vehicle_state.h"
// #include "behaviour.h"


namespace fiss
{
double MARKER_DISPLAY_DURATION = 0.1;

/* --------------------- Collision Detector Marker Scale -------------------- */

double BUGGY_RECT_MARKER_SCALE = 0.1;
double PREDICTED_TRAJECTORY_MARKER_SCALE = 0.03;
double OBSTACLE_HULL_MARKER_SCALE = 0.1;
double OBSTACLE_CENTROID_MARKER_SCALE = 0.2;
double OBSTACLE_LABEL_MARKER_SCALE = 0.5;
double OBSTACLE_LABEL_MARKER_HEIGHT = 2.0;

/* ----------------------- Local Planner Marker Scale ----------------------- */

double CANDIDATE_PATH_MARKER_SCALE = 0.05;

/* --------------------- Behaviour Planner Marker Scale --------------------- */

double BEHAVIOUR_MARKER_SCALE = 0.3;
double BEHAVIOUR_MARKER_HEIGHT = 2.5;

/* ----------------------- Lane Publisher Marker Scale ---------------------- */

double LANE_MARKER_SCALE = 0.05;
double SPECIAL_WAYPOINT_MARKER_SCALE = 0.5;
double END_POINT_MARKER_SCALE = 0.02;
double ARROW_MARKER_LENGTH = 2.0;
class Visualization
{
public:
  enum COLOR
  {
    RED,
    BLUE,
    GREEN,
    ORANGE,
    DARK_GREEN,
    DARK_RED,
    TURQUOISE,
    BLACK,
    WHITE,
    YELLOW,
    FAINT_WHITE,
    TRANSLUCENT_RED,
    TRANSPARENT,

    /* ----------------------------- for buggy mesh ----------------------------- */

    BUGGY_BLUE,
    BUGGY_RED,
    BUGGY_WHITE,
    BUGGY_BLACK,
    BUGGY_YELLOW,
  };

  static std_msgs::ColorRGBA getColor(COLOR colorName)
  {
    std_msgs::ColorRGBA color;

    switch (colorName)
    {
      case RED:
        color = parseColor(1, 0, 0);
        break;
      case GREEN:
        color = parseColor(0, 1, 0);
        break;
      case BLUE:
        color = parseColor(0, 0, 1);
        break;
      case ORANGE:
        color = parseColor(1, 0.65, 0);
        break;
      case DARK_GREEN:
        color = parseColor(0, 0.5, 0);
        break;
      case DARK_RED:
        color = parseColor(0.5, 0, 0);
        break;
      case TURQUOISE:
        color = parseColor(0.25, 0.93, 0.82);
        break;
      case BLACK:
        color = parseColor(0, 0, 0);
        break;
      case WHITE:
        color = parseColor(1, 1, 1);
        break;
      case YELLOW:
        color = parseColor(1, 1, 0);
        break;
      case FAINT_WHITE:
        color = parseColor(1, 1, 1, 0.25);
        break;
      case TRANSLUCENT_RED:
        color = parseColor(1, 0.2, 0.1, 0.5);
        break;
      case TRANSPARENT:
        color = parseColor(0, 0, 0, 0);
        break;

        /* ------------------------------ buggy colors ------------------------------ */

      case BUGGY_BLUE:
        color = parseColor(0, 0.466, 0.784, 0.75);
        break;
      case BUGGY_BLACK:
        color = parseColor(0, 0, 0, 0.75);
        break;
      case BUGGY_RED:
        color = parseColor(1.0, 0, 0, 0.75);
        break;
      case BUGGY_WHITE:
        color = parseColor(1, 1, 1, 0.75);
        break;
      case BUGGY_YELLOW:
        color = parseColor(1, 0.8, 0, 0.75);
        break;
    }

    return color;
  }

  static std_msgs::ColorRGBA parseColor(double r, double g, double b, double a)
  {
    std_msgs::ColorRGBA color;

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
  }

  static std_msgs::ColorRGBA parseColor(double r, double g, double b)
  {
    std_msgs::ColorRGBA color;

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;

    return color;
  }

  static visualization_msgs::MarkerArray concatMarkerArrays(visualization_msgs::MarkerArray array_1,
                                                            visualization_msgs::MarkerArray array_2)
  {
    array_1.markers.insert(array_1.markers.end(), array_2.markers.begin(), array_2.markers.end());
    return array_1;
  }

  static visualization_msgs::Marker initializeMarker(int& marker_id, std::string ns, std_msgs::ColorRGBA color, double marker_scale,
                                                     int32_t marker_type, bool infinite_duration = false)
  {
    visualization_msgs::Marker marker;

    marker.lifetime = infinite_duration ? ros::Duration() : ros::Duration(MARKER_DISPLAY_DURATION);
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = marker_type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.scale.x = marker_scale;
    marker.scale.y = marker_scale;
    marker.scale.z = marker_scale;
    marker.color = color;

    return marker;
  }

  static visualization_msgs::Marker initializeMarker(int& marker_id, std::string ns, COLOR color,
                                                     geometry_msgs::Vector3 marker_scale, int32_t marker_type,
                                                     bool infinite_duration = false)
  {
    visualization_msgs::Marker marker;
    marker.lifetime = infinite_duration ? ros::Duration() : ros::Duration(MARKER_DISPLAY_DURATION);
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = marker_type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.scale = marker_scale;
    marker.color = getColor(color);
    return marker;
  }

  static visualization_msgs::Marker polygonToMarker(const geometry_msgs::Polygon polygon, int& marker_id,
                                                    double marker_scale, std::string ns, COLOR color,
                                                    bool infinite_duration = false, bool flatten = false)
  {
    visualization_msgs::Marker polygon_marker =
        initializeMarker(marker_id, ns, getColor(color), marker_scale, visualization_msgs::Marker::LINE_STRIP, infinite_duration);

    for (auto const& point : polygon.points)
    {
      geometry_msgs::Point tmp_point;
      tmp_point.x = point.x;
      tmp_point.y = point.y;
      tmp_point.z = flatten ? 0 : point.z;
      polygon_marker.points.push_back(tmp_point);
    }
    return polygon_marker;
  }

  static visualization_msgs::Marker pointToMarker(const geometry_msgs::Point point, int& marker_id, COLOR color,
                                                  std::string ns, double marker_scale, bool infinite_duration = false)
  {
    visualization_msgs::Marker point_marker =
        initializeMarker(marker_id, ns, getColor(color), marker_scale, visualization_msgs::Marker::SPHERE, infinite_duration);

    point_marker.pose.position = point;

    return point_marker;
  }

  static visualization_msgs::MarkerArray pointsToMarkers(const std::vector<geometry_msgs::Point> points, int& marker_id,
                                                         COLOR color, std::string ns, double marker_scale,
                                                         bool infinite_duration = false)
  {
    visualization_msgs::MarkerArray point_markers;

    for (auto point : points)
    {
      visualization_msgs::Marker point_marker =
          pointToMarker(point, marker_id, color, ns, marker_scale, infinite_duration);
      point_markers.markers.push_back(point_marker);
    }

    return point_markers;
  }

  static visualization_msgs::Marker trajToMarker(const nav_msgs::Path traj, int& marker_id, COLOR color, std::string ns,
                                                 double marker_scale, bool infinite_duration = false)
  {
    visualization_msgs::Marker traj_marker =
        initializeMarker(marker_id, ns, getColor(color), marker_scale, visualization_msgs::Marker::LINE_STRIP, infinite_duration);

    for (auto pose_stamped : traj.poses)
    {
      traj_marker.points.push_back(pose_stamped.pose.position);
    }

    return traj_marker;
  }

  static visualization_msgs::MarkerArray posesToMarkers(const geometry_msgs::PoseArray pose_arr, int& marker_id,
                                                        COLOR color, std::string ns, double arrow_length,
                                                        bool infinite_duration = false)
  {
    visualization_msgs::MarkerArray pose_markers;

    geometry_msgs::Vector3 arrow_scale;
    arrow_scale.x = arrow_length;
    arrow_scale.y = 0.2 * arrow_length;
    arrow_scale.z = 0.1 * arrow_length;

    for (auto pose : pose_arr.poses)
    {
      visualization_msgs::Marker arrow_marker =
          initializeMarker(marker_id, ns, color, arrow_scale, visualization_msgs::Marker::ARROW, infinite_duration);
      arrow_marker.pose = pose;

      pose_markers.markers.push_back(arrow_marker);
    }

    return pose_markers;
  }

  static visualization_msgs::Marker stringToMarker(const std::string str, const geometry_msgs::Point marker_pos,
                                                   int& marker_id, COLOR color, std::string ns, double marker_scale,
                                                   bool infinite_duration = false)
  {
    visualization_msgs::Marker text_marker = initializeMarker(
        marker_id, ns, getColor(color), marker_scale, visualization_msgs::Marker::TEXT_VIEW_FACING, infinite_duration);

    text_marker.text = str;
    text_marker.pose.position = marker_pos;

    return text_marker;
  }

  static visualization_msgs::Marker meshToMarker(const std::string mesh_file_traj,
                                                 const geometry_msgs::Pose marker_pose, int& marker_id, COLOR color,
                                                 std::string ns, double marker_scale, bool infinite_duration = false)
  {
    visualization_msgs::Marker mesh_marker = initializeMarker(
        marker_id, ns, getColor(color), marker_scale, visualization_msgs::Marker::MESH_RESOURCE, infinite_duration);

    mesh_marker.mesh_resource = mesh_file_traj;
    mesh_marker.pose = marker_pose;

    return mesh_marker;
  }
};
class LocalPlannerVisualization : public Visualization
{
public:
  static visualization_msgs::MarkerArray visualizeCandidateTrajs(const std::vector<FrenetPath>& candidate_trajs,
                                                                 const double z_map, const double max_speed)
  {
    visualization_msgs::MarkerArray safest_trajs_markers;
    visualization_msgs::MarkerArray close_proximity_trajs_markers;
    visualization_msgs::MarkerArray unsafe_trajs_markers;

    int marker_id = 0;
    visualization_msgs::MarkerArray candidate_trajs_markers;

    // Find the min and max costs to determin the color code
    double min_cost = 10000.0;
    double max_cost = 0.0;
    for (const auto& traj : candidate_trajs)
    {
      min_cost = std::min(min_cost, traj.final_cost);
      max_cost = std::max(max_cost, traj.final_cost);
    }

    for (const auto& traj : candidate_trajs)
    {
      const double R = 1.0*(traj.final_cost - min_cost)/(max_cost - min_cost);
      const double G = 0.7*(1 - R);
      std_msgs::ColorRGBA color = parseColor(R, G, 0.2, 0.8);
      candidate_trajs_markers.markers.emplace_back(frenetPathToMarker(traj, marker_id, "candidate", color, z_map, max_speed));
      
      // if (!traj.constraint_passed)
      // {
      //   candidate_trajs_markers.markers.emplace_back(frenetPathToMarker(traj, marker_id, "unsmooth", getColor(COLOR::RED), z_map, max_speed));
      // }
      // else if (!traj.collision_passed)
      // {
      //   candidate_trajs_markers.markers.emplace_back(frenetPathToMarker(traj, marker_id, "unsafe", getColor(COLOR::DARK_RED), z_map, max_speed));
      // }
      // else
      // {
      //   const double R = 1.0*traj.final_cost/max_cost;
      //   const double G = 0.7*traj.final_cost/min_cost;
      //   std_msgs::ColorRGBA color = parseColor(R, G, 0.2, 0.8);
      //   candidate_trajs_markers.markers.emplace_back(frenetPathToMarker(traj, marker_id, "safe", color, z_map, max_speed));
      // }
    }

    return candidate_trajs_markers;
  }

  static visualization_msgs::Marker frenetPathToMarker(const fiss::FrenetPath& traj, int& marker_id, const std::string ns, const std_msgs::ColorRGBA color, 
                                                       const double z_map, const double max_speed)
  {
    visualization_msgs::Marker traj_marker = initializeMarker(marker_id, ns, color, CANDIDATE_PATH_MARKER_SCALE, visualization_msgs::Marker::LINE_STRIP);

    for (int i = 0; i < traj.x.size(); i++)
    {
      geometry_msgs::Point tmp_point;
      if (!std::isnormal(traj.x[i]) || !std::isnormal(traj.y[i]) || 
          !std::isnormal(traj.s_d[i]) || !std::isnormal(traj.d_d[i]))
      {
        break;
      }
      tmp_point.x = traj.x[i];
      tmp_point.y = traj.y[i];
      tmp_point.z = z_map + 2.0*std::hypot(traj.s_d[i], traj.d_d[i])/max_speed;
      traj_marker.points.push_back(tmp_point);
    }

    return traj_marker;
  }
};

class CollisionDetectorVisualization : public Visualization
{
public:
  static visualization_msgs::MarkerArray visualizeCollisions(
      const std::vector<geometry_msgs::Polygon> buggy_rects, const autoware_msgs::DetectedObjectArray objects,
      const Path front_axle_traj, const Path rear_axle_traj, VehicleState current_state,
      double vehicle_width, double safety_margin, geometry_msgs::Polygon dynamic_bumper_straight, int slowdown_mode)
  {
    int marker_id = 0;

    visualization_msgs::MarkerArray buggy_prediction_markers;
    visualization_msgs::MarkerArray obstacle_hull_markers;
    visualization_msgs::MarkerArray obstacle_centroid_markers;
    visualization_msgs::MarkerArray obstacle_label_markers;
    visualization_msgs::MarkerArray visualization_markers;
    visualization_msgs::Marker front_axle_traj_marker;
    visualization_msgs::Marker rear_axle_traj_marker;
    visualization_msgs::Marker dynamic_bumper_straight_marker;

    /* -------------------------------- Obstacles ------------------------------- */

    obstacle_hull_markers = objectsToHulls(objects, marker_id, "obstacles/hulls", YELLOW, RED, DARK_RED, WHITE, GREEN,
                                           DARK_GREEN, BLUE, TURQUOISE);
    obstacle_centroid_markers = ObjectsToCentroids(objects, marker_id, "obstacles/centroids", BLUE);
    obstacle_label_markers = objectsToLabels(objects, marker_id, "obstacles/labels", WHITE);

    /* ---------------------------------- Buggy --------------------------------- */

    buggy_prediction_markers = buggyPredictionsToMarkers(buggy_rects, marker_id, "buggy_predicted_positions", GREEN);

    front_axle_traj_marker = visualizePredictedTrajectory(front_axle_traj, vehicle_width, safety_margin, 0.0, current_state, true,
                                                          marker_id, "predicted_trajectory/front_axle", GREEN, PREDICTED_TRAJECTORY_MARKER_SCALE);

    rear_axle_traj_marker = visualizePredictedTrajectory(rear_axle_traj, vehicle_width, safety_margin, 0.0, current_state, true,
                                                         marker_id, "predicted_trajectory/rear_axle", BLUE, PREDICTED_TRAJECTORY_MARKER_SCALE);

    dynamic_bumper_straight_marker =
        straightBumperToMarker(dynamic_bumper_straight, marker_id, "buggy_prediction_straight", slowdown_mode);

    visualization_markers = concatMarkerArrays(visualization_markers, buggy_prediction_markers);
    visualization_markers = concatMarkerArrays(visualization_markers, obstacle_hull_markers);
    visualization_markers = concatMarkerArrays(visualization_markers, obstacle_centroid_markers);
    visualization_markers = concatMarkerArrays(visualization_markers, obstacle_label_markers);

    visualization_markers.markers.insert(
        visualization_markers.markers.end(),
        { front_axle_traj_marker, rear_axle_traj_marker, dynamic_bumper_straight_marker });

    return visualization_markers;
  }

  static visualization_msgs::MarkerArray buggyPredictionsToMarkers(const std::vector<geometry_msgs::Polygon> buggy_predicted_rects, 
                                                                   int& marker_id, std::string ns, COLOR color)
  {
    visualization_msgs::MarkerArray polygons_markers;

    for (auto const& polygon : buggy_predicted_rects)
    {
      visualization_msgs::Marker polygon_marker =
          polygonToMarker(polygon, marker_id, BUGGY_RECT_MARKER_SCALE, ns, color);

      polygons_markers.markers.push_back(polygon_marker);
    }

    return polygons_markers;
  }

  static visualization_msgs::Marker straightBumperToMarker(const geometry_msgs::Polygon straight_bumper_polygon,
                                                           int& marker_id, std::string ns, int slowdown_mode)
  {
    COLOR bumper_color;

    if (slowdown_mode == 0)
    {
      bumper_color = TURQUOISE;
    }
    else if (slowdown_mode == 1 || slowdown_mode == 2)
    {
      bumper_color = ORANGE;
    }
    else
    {
      bumper_color = FAINT_WHITE;
    }

    return polygonToMarker(straight_bumper_polygon, marker_id, PREDICTED_TRAJECTORY_MARKER_SCALE, ns, bumper_color);
  }

  static visualization_msgs::MarkerArray objectsToHulls(const autoware_msgs::DetectedObjectArray& in_objects,
                                                        int& marker_id, std::string ns, COLOR color_slow,
                                                        COLOR color_stop, COLOR color_emergency, COLOR color_map,
                                                        COLOR color_lidar_out_fov, COLOR color_lidar_in_fov_unfused,
                                                        COLOR color_vision_unfused, COLOR color_fused)
  {
    visualization_msgs::MarkerArray polygon_hulls;

    for (auto const& object : in_objects.objects)
    {
      visualization_msgs::Marker hull;
      COLOR hull_color;

      if (object.user_defined_info.size() == 0)
      {
        hull_color = BLACK;
      }
      else if (object.user_defined_info.size() == 2)
      {
        if (object.user_defined_info.at(1) == "EMERGENCY_STOP")
        {
          hull_color = color_emergency;
        }
        else if (object.user_defined_info.at(1) == "STOP")
        {
          hull_color = color_stop;
        }
        else if (object.user_defined_info.at(1) == "SLOW")
        {
          hull_color = color_slow;
        }
      }
      else if (object.user_defined_info.size() == 1)
      {
        if (object.user_defined_info.at(0) == "MAP_OBSTACLE")
        {
          hull_color = color_map;
        }
        else if (object.user_defined_info.at(0) == "VISION_UNFUSED")
        {
          hull_color = color_vision_unfused;
        }
        else if (object.user_defined_info.at(0) == "LIDAR_OUT_FOV")
        {
          hull_color = color_lidar_out_fov;
        }
        else if (object.user_defined_info.at(0) == "LIDAR_IN_FOV_UNFUSED")
        {
          hull_color = color_lidar_in_fov_unfused;
        }
        else if (object.user_defined_info.at(0) == "FUSED")
        {
          hull_color = color_fused;
        }
      }

      hull = polygonToMarker(object.convex_hull.polygon, marker_id, OBSTACLE_HULL_MARKER_SCALE, ns, hull_color, false,
                             true);

      polygon_hulls.markers.push_back(hull);
    }
    return polygon_hulls;
  }

  static visualization_msgs::MarkerArray ObjectsToCentroids(const autoware_msgs::DetectedObjectArray& in_objects,
                                                            int& marker_id, std::string ns, COLOR color)
  {
    visualization_msgs::MarkerArray centroid_markers;
    for (auto const& object : in_objects.objects)
    {
      visualization_msgs::Marker centroid_marker =
          pointToMarker(object.pose.position, marker_id, color, ns, OBSTACLE_CENTROID_MARKER_SCALE);

      centroid_markers.markers.push_back(centroid_marker);
    }
    return centroid_markers;
  }

  static visualization_msgs::MarkerArray objectsToLabels(const autoware_msgs::DetectedObjectArray& in_objects,
                                                         int& marker_id, std::string ns, COLOR color)
  {
    visualization_msgs::MarkerArray label_markers;

    for (auto const& object : in_objects.objects)
    {
      if (object.label.empty() || object.label == "unknown")
      {
        continue;
      }

      geometry_msgs::Point marker_pos = object.pose.position;
      marker_pos.z = OBSTACLE_LABEL_MARKER_HEIGHT;

      visualization_msgs::Marker label_marker =
          stringToMarker(object.label, marker_pos, marker_id, color, ns, OBSTACLE_LABEL_MARKER_SCALE);

      label_markers.markers.push_back(label_marker);
    }
    return label_markers;
  }

  static visualization_msgs::Marker visualizePredictedTrajectory(const Path& traj, const double vehicle_width, const double safety_margin, 
                                                                 const double z, const VehicleState current_state, const bool use_current_state,
                                                                 int& marker_id, const std::string ns, const COLOR color, const double scale)
  {
    visualization_msgs::Marker predicted_trajectory_marker = initializeMarker(marker_id, ns, getColor(color), 
                                                                              scale, visualization_msgs::Marker::LINE_STRIP);

    std::vector<geometry_msgs::Point32> left_bound;
    std::vector<geometry_msgs::Point32> right_bound;
    std::vector<geometry_msgs::Point32> left_right_points;

    if (use_current_state)
    {
      left_right_points = getBothSidesPoints(current_state.x, current_state.y, z, current_state.yaw, vehicle_width, safety_margin);
      left_bound.push_back(left_right_points.at(0));
      right_bound.push_back(left_right_points.at(1));
    }
    
    for (int i = 0; i < traj.x.size(); i++)
    {
      left_right_points = getBothSidesPoints(traj.x[i], traj.y[i], z, traj.yaw[i], vehicle_width, safety_margin);
      left_bound.push_back(left_right_points.at(0));
      right_bound.push_back(left_right_points.at(1));
    }

    for (int i = 0; i < left_bound.size(); i++)
    {
      geometry_msgs::Point tmp_point;
      tmp_point.x = left_bound[i].x;
      tmp_point.y = left_bound[i].y;
      tmp_point.z = left_bound[i].z;

      predicted_trajectory_marker.points.push_back(tmp_point);
    }

    for (int i = right_bound.size(); i > 0; i--)
    {
      geometry_msgs::Point tmp_point;
      tmp_point.x = right_bound.at(i - 1).x;
      tmp_point.y = right_bound.at(i - 1).y;
      tmp_point.z = right_bound.at(i - 1).z;

      predicted_trajectory_marker.points.push_back(tmp_point);
    }

    return predicted_trajectory_marker;
  }

  static std::vector<geometry_msgs::Point32> getBothSidesPoints(double x, double y, double z, double yaw, 
                                                                double width, double margin)
  {
    geometry_msgs::Point32 left_point, right_point;

    // left being positive
    double left_direction = yaw + M_PI / 2.0;

    left_point.x = x + cos(left_direction) * (0.5 * width + margin);
    left_point.y = y + sin(left_direction) * (0.5 * width + margin) ;
    left_point.z = z;

    right_point.x = x - cos(left_direction) * (0.5 * width + margin);
    right_point.y = y - sin(left_direction) * (0.5 * width + margin);
    right_point.z = z;

    return { left_point, right_point };
  }
};

// class BehaviourPlannerVisualization : public Visualization
// {
// public:
//   static visualization_msgs::MarkerArray
//   visualizeBehaviour(std::vector<Behaviour> candidate_behaviours,
//                      Behaviour final_behaviour, VehicleState current_state)
//   {
//     int marker_id = 0;

//     double marker_height = BEHAVIOUR_MARKER_HEIGHT;

//     visualization_msgs::MarkerArray behaviour_markers;

//     for (auto behaviour : candidate_behaviours)
//     {
//       behaviour_markers.markers.push_back(
//           behaviourToMarker(behaviour, current_state, marker_id, marker_height, "behaviour/candidates", false));
//     }

//     behaviour_markers.markers.push_back(
//         behaviourToMarker(final_behaviour, current_state, marker_id, marker_height, "behaviour/final", true));

//     behaviour_markers.markers.push_back(
//         generateStopWallMarker(marker_id, final_behaviour.getTargetSpeed() == 0, current_state));

//     return behaviour_markers;
//   }

//   static visualization_msgs::Marker behaviourToMarker(Behaviour behaviour,
//                                                       VehicleState current_state, int& marker_id, double& marker_height,
//                                                       std::string ns, bool is_final)
//   {
//     COLOR marker_color;
//     double marker_scale;
//     std::string marker_text;
//     geometry_msgs::Point marker_pos;

//     if (behaviour.isStopping())
//     {
//       marker_color = RED;
//     }
//     else if (behaviour.isSlowing())
//     {
//       marker_color = YELLOW;
//     }
//     else
//     {
//       marker_color = WHITE;
//     }

//     marker_text = (is_final ? "[FINAL] " : "") + behaviour.toString();
//     marker_scale = (is_final ? 1.5 : 1) * BEHAVIOUR_MARKER_SCALE;
//     marker_pos.x = current_state.x;
//     marker_pos.y = current_state.y;
//     marker_pos.z = marker_height;

//     visualization_msgs::Marker behaviour_marker =
//         stringToMarker(marker_text, marker_pos, marker_id, marker_color, ns, marker_scale);

//     marker_height += BEHAVIOUR_MARKER_SCALE;

//     return behaviour_marker;
//   }

//   /**
//    * @brief This method seems quite useless, but leaving it here anyway
//    *
//    * @param marker_id
//    * @param stopwall_flag
//    * @param current_state
//    * @return visualization_msgs::Marker
//    */
//   static visualization_msgs::Marker generateStopWallMarker(int& marker_id, bool stopwall_flag,
//                                                            VehicleState current_state)
//   {
//     double marker_height = 1.5;

//     geometry_msgs::Vector3 marker_scale;
//     marker_scale.x = 0.1;
//     marker_scale.y = marker_height;
//     marker_scale.z = 3.0;

//     COLOR marker_color = stopwall_flag ? TRANSLUCENT_RED : TRANSPARENT;

//     visualization_msgs::Marker stopwall_marker =
//         initializeMarker(marker_id, "stopwall", marker_color, marker_scale, visualization_msgs::Marker::CUBE);

//     double front_x = current_state.x + cos(current_state.yaw) * 2.7;
//     double front_y = current_state.y + sin(current_state.yaw) * 2.7;

//     stopwall_marker.pose.position.x = front_x;
//     stopwall_marker.pose.position.y = front_y;
//     stopwall_marker.pose.position.z = marker_height / 2;

//     stopwall_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, current_state.yaw);

//     return stopwall_marker;
//   }
// };

class LanePublisherVisualization : public Visualization
{
private:
  static constexpr const char* turning_points_ns = "turning_points";
  static constexpr const char* slope_points_ns = "slope_points";
  static constexpr const char* current_traj_ns = "current_traj";
  static constexpr const char* lane_boundary_ns = "lane_boundary";
  static constexpr const char* lane_center_ns = "lane_center";

public:
  static visualization_msgs::MarkerArray visualizeLanes(
      const nav_msgs::Path lane_boundary_1, const nav_msgs::Path lane_boundary_2, const nav_msgs::Path lane_center,
      const geometry_msgs::PoseArray current_traj, std::vector<geometry_msgs::Point> slope_points,
      std::vector<geometry_msgs::Point> turning_points, const geometry_msgs::Point end_point)
  {
    int marker_id = 0;
    visualization_msgs::MarkerArray lane_markers;
    visualization_msgs::MarkerArray turning_point_markers;
    visualization_msgs::MarkerArray slope_point_markers;
    visualization_msgs::MarkerArray current_traj_markers;
    visualization_msgs::Marker lane_boundary_1_marker;
    visualization_msgs::Marker lane_boundary_2_marker;
    visualization_msgs::Marker lane_center_marker;
    visualization_msgs::Marker current_traj_marker;
    visualization_msgs::Marker end_point_marker;

    end_point_marker = endPointToMarker(end_point, marker_id);
    lane_boundary_1_marker = trajToMarker(lane_boundary_1, marker_id, BLACK, lane_boundary_ns, LANE_MARKER_SCALE, true);
    lane_boundary_2_marker = trajToMarker(lane_boundary_2, marker_id, BLACK, lane_boundary_ns, LANE_MARKER_SCALE, true);
    lane_center_marker = trajToMarker(lane_center, marker_id, WHITE, lane_center_ns, LANE_MARKER_SCALE, true);

    for (int i = 0; i < turning_points.size(); i++)
    {
      turning_points[i].z = 1.0;
    }

    for (int i = 0; i < slope_points.size(); i++)
    {
      slope_points[i].z = 2.0;
    }

    turning_point_markers =
        pointsToMarkers(turning_points, marker_id, RED, turning_points_ns, SPECIAL_WAYPOINT_MARKER_SCALE, true);
    slope_point_markers =
        pointsToMarkers(slope_points, marker_id, BLUE, slope_points_ns, SPECIAL_WAYPOINT_MARKER_SCALE, true);
    current_traj_markers = posesToMarkers(current_traj, marker_id, RED, current_traj_ns, ARROW_MARKER_LENGTH, true);

    lane_markers = concatMarkerArrays(lane_markers, turning_point_markers);
    lane_markers = concatMarkerArrays(lane_markers, slope_point_markers);
    lane_markers = concatMarkerArrays(lane_markers, current_traj_markers);

    lane_markers.markers.insert(lane_markers.markers.end(), { end_point_marker, lane_boundary_1_marker,
                                                              lane_boundary_2_marker, lane_center_marker });

    return lane_markers;
  }

  static visualization_msgs::Marker endPointToMarker(geometry_msgs::Point end_point, int& marker_id)
  {
    geometry_msgs::Pose marker_pose;
    marker_pose.position = end_point;
    marker_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, -2.7);

    visualization_msgs::Marker end_point_marker = meshToMarker(
        "package://fiss_planner/meshes/endpoint.stl", marker_pose, marker_id, RED, "end_point", END_POINT_MARKER_SCALE, true);

    return end_point_marker;
  }

  static visualization_msgs::MarkerArray deletePrevMarkers(visualization_msgs::MarkerArray marker_arr)
  {
    for (auto& marker : marker_arr.markers)
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }

    return marker_arr;
  }
};
}  // namespace fiss