#ifndef SAT_COLLISION_CHECKER_H_
#define SAT_COLLISION_CHECKER_H_

#include <math.h>
#include <vector>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>



namespace fiss
{
class SATCollisionChecker
{
public:
  SATCollisionChecker(){};
  virtual ~SATCollisionChecker(){};
  
  //Check for collision between the object and the obstacle
  bool check_collision(geometry_msgs::Polygon rect, geometry_msgs::Polygon obstacle_poly);
  
  //Construct a rectangle with coordinates in the map frame.
  geometry_msgs::Polygon construct_rectangle(const double centre_x, const double centre_y, const double yaw, 
                                             const double length, const double width, 
                                             const double margin_lon, const double margin_lat);

  geometry_msgs::Polygon construct_straight_bumper(double centre_x, double centre_y, double yaw,
                                                                    double length, double width, double margin);
  
  //Removes the top layer of a 3D convex hull.
  geometry_msgs::Polygon remove_top_layer(geometry_msgs::Polygon poly);



private:
  //Get the directions of the edges of a polygon
  std::vector<geometry_msgs::Vector3> vertices_to_edges(geometry_msgs::Polygon poly);
  
  //Get the direction of a single edge
  geometry_msgs::Vector3 get_edge_direction(geometry_msgs::Point32 p0, geometry_msgs::Point32 p1);
  
  //Get the normal of a vector
  geometry_msgs::Vector3 get_orthogonal(geometry_msgs::Vector3 vec);
  
  // Normalize a vector to get a unit vector
  geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 vec);
  
  //Project a polygon onto an axis
  std::vector<double> project(geometry_msgs::Polygon poly, geometry_msgs::Vector3 axis);
  
  //Get the dot product between a point and an axis
  double get_dot_product(geometry_msgs::Point32 point, geometry_msgs::Vector3 axis);
  
  //Check for collision between the object and the obstacle
  bool is_overlapping(std::vector<double> projection1, std::vector<double> projection2);
  
  //Rotate a rectangle centered about (0, 0) by the yaw angle given and translate it to its actual position
  geometry_msgs::Polygon rotate_and_translate_rect(geometry_msgs::Polygon rect, double centre_x, double centre_y, double yaw);

};
}  // namespace fiss


#endif  // SAT_COLLISION_CHECKER_H_