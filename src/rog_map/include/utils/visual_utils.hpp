#pragma once

#include "utils/common_lib.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath> 

namespace rog_map {
    class Color : public std_msgs::msg::ColorRGBA {
    public:
        Color() : std_msgs::msg::ColorRGBA() {}

        Color(int hex_color) {
            int _r = (hex_color >> 16) & 0xFF;
            int _g = (hex_color >> 8) & 0xFF;
            int _b = hex_color & 0xFF;
            r = static_cast<double>(_r) / 255.0;
            g = static_cast<double>(_g) / 255.0;
            b = static_cast<double>(_b) / 255.0;
        }

        Color(const Color& c, double alpha) {
            r = c.r;
            g = c.g;
            b = c.b;
            a = alpha;
        }

        Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {
            r = red > 1.0 ? red / 255.0 : red;
            g = green > 1.0 ? green / 255.0 : green;
            b = blue > 1.0 ? blue / 255.0 : blue;
        }

        Color(double red, double green, double blue, double alpha) : Color() {
            r = red > 1.0 ? red / 255.0 : red;
            g = green > 1.0 ? green / 255.0 : green;
            b = blue > 1.0 ? blue / 255.0 : blue;
            a = alpha;
        }

        static Color White() { return Color(1.0, 1.0, 1.0); }
        static Color Black() { return Color(0.0, 0.0, 0.0); }
        static Color Gray() { return Color(0.5, 0.5, 0.5); }
        static Color Red() { return Color(1.0, 0.0, 0.0); }
        static Color Green() { return Color(0.0, 0.96, 0.0); }
        static Color Blue() { return Color(0.0, 0.0, 1.0); }
        static Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
        static Color Yellow() { return Color(1.0, 1.0, 0.0); }
        static Color Orange() { return Color(1.0, 0.5, 0.0); }
        static Color Purple() { return Color(0.5, 0.0, 1.0); }
        static Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
        static Color Teal() { return Color(0.0, 1.0, 1.0); }
        static Color Pink() { return Color(1.0, 0.0, 0.5); }
    };


    /* Type A, directly publish marker in publisher */
    inline void visualizeText(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub,
                              const std::string &ns,
                              const std::string &text,
                              const Vec3f &position,
                              const Color &c,
                              double size,
                              int id = -1) {
        visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = "world";
        marker.header.frame_id = "camera_init";
        marker.header.stamp = rclcpp::Clock().now(); 
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns;
        if (id >= 0) {
            marker.id = id;
        } else {
            static int static_id = 0;
            marker.id = static_id++;
        }
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(marker);
        pub->publish(arr); 
    }

    inline void visualizePoint(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub_,
                               const Vec3f &pt,
                               Color color = Color::Pink(),
                               const std::string& ns = "pt",
                               double size = 0.1, 
                               int id = -1,
                               bool print_ns = true) {
        visualization_msgs::msg::MarkerArray mkr_arr;
        visualization_msgs::msg::Marker marker_ball;
        static int cnt = 0;
        Vec3f cur_pos = pt;
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
            return;
        }
        marker_ball.header.frame_id = "camera_init";
        marker_ball.header.stamp = rclcpp::Clock().now();
        marker_ball.ns = ns;
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::msg::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::msg::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // add text
        if (print_ns) {
            visualization_msgs::msg::Marker marker;
            // marker.header.frame_id = "world";
            marker.header.frame_id = "camera_init";
            marker.header.stamp = rclcpp::Clock().now();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int static_id = 0;
                marker.id = static_id++;
            }
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }

        pub_->publish(mkr_arr); 
    }

    inline void visualizeBoundingBox(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub,
                                     const Vec3f &box_min,
                                     const Vec3f &box_max,
                                     const std::string &ns,
                                     const Color &color,
                                     double size_x = 0.1,
                                     double alpha = 1.0,
                                     bool print_ns = true) {
        (void)print_ns; // Suppress unused parameter warning
        
        Vec3f size = (box_max - box_min) / 2.0;
        Vec3f vis_pos_world = (box_min + box_max) / 2.0;
        double width = size.x();
        double length = size.y();
        double height = size.z();
        visualization_msgs::msg::MarkerArray mkrarr;
        //Publish Bounding box
        int id = 0;
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.stamp = rclcpp::Clock().now();
        // line_strip.header.frame_id = "world";
        line_strip.header.frame_id = "camera_init";
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++; //unique id, useful when multiple markers exist.
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP; //marker type
        line_strip.scale.x = size_x;

        line_strip.color = color;
        line_strip.color.a = alpha; 
        geometry_msgs::msg::Point p[8];

        //vis_pos_world是目标物的坐标
        p[0].x = vis_pos_world(0) - width;
        p[0].y = vis_pos_world(1) + length;
        p[0].z = vis_pos_world(2) + height;
        p[1].x = vis_pos_world(0) - width;
        p[1].y = vis_pos_world(1) - length;
        p[1].z = vis_pos_world(2) + height;
        p[2].x = vis_pos_world(0) - width;
        p[2].y = vis_pos_world(1) - length;
        p[2].z = vis_pos_world(2) - height;
        p[3].x = vis_pos_world(0) - width;
        p[3].y = vis_pos_world(1) + length;
        p[3].z = vis_pos_world(2) - height;
        p[4].x = vis_pos_world(0) + width;
        p[4].y = vis_pos_world(1) + length;
        p[4].z = vis_pos_world(2) - height;
        p[5].x = vis_pos_world(0) + width;
        p[5].y = vis_pos_world(1) - length;
        p[5].z = vis_pos_world(2) - height;
        p[6].x = vis_pos_world(0) + width;
        p[6].y = vis_pos_world(1) - length;
        p[6].z = vis_pos_world(2) + height;
        p[7].x = vis_pos_world(0) + width;
        p[7].y = vis_pos_world(1) + length;
        p[7].z = vis_pos_world(2) + height;
        //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
        for (int i = 0; i < 8; i++) {
            line_strip.points.push_back(p[i]);
        }
        //为了保证矩形框的八条边都存在：
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[3]);
        line_strip.points.push_back(p[2]);
        line_strip.points.push_back(p[5]);
        line_strip.points.push_back(p[6]);
        line_strip.points.push_back(p[1]);
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[7]);
        line_strip.points.push_back(p[4]);
        mkrarr.markers.push_back(line_strip);
        pub->publish(mkrarr); // 指针调用
    }

    /* Type B: Add marker to given marker_arr for later publish */
    inline void visualizeText(visualization_msgs::msg::MarkerArray &mkr_arr,
                              const std::string &ns,
                              const std::string &text,
                              const Vec3f &position,
                              const Color &c = Color::White(),
                              double size = 0.6,
                              int id = -1) {
        visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = "world";
        marker.header.frame_id = "camera_init";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns;
        if (id >= 0) {
            marker.id = id;
        } else {
            static int static_id = 0;
            marker.id = static_id++;
        }
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        mkr_arr.markers.push_back(marker);
    }

    inline void visualizePoint(visualization_msgs::msg::MarkerArray &mkr_arr,
                               const Vec3f &pt,
                               Color color = Color::Pink(),
                               const std::string& ns = "pt",
                               double size = 0.1, 
                               int id = -1,
                               bool print_ns = true) {
        visualization_msgs::msg::Marker marker_ball;
        static int cnt = 0;
        Vec3f cur_pos = pt;
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
            return;
        }
        // marker_ball.header.frame_id = "world";
        marker_ball.header.frame_id = "camera_init";
        marker_ball.header.stamp = rclcpp::Clock().now();
        marker_ball.ns = ns;
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::msg::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::msg::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // add text
        if (print_ns) {
            visualization_msgs::msg::Marker marker;
            // marker.header.frame_id = "world";
            marker.header.frame_id = "camera_init";
            marker.header.stamp = rclcpp::Clock().now();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int static_id = 0;
                marker.id = static_id++;
            }
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }
    }

    inline void visualizeBoundingBox(visualization_msgs::msg::MarkerArray &mkrarr,
                                     const Vec3f &box_min,
                                     const Vec3f &box_max,
                                     const std::string &ns,
                                     const Color &color,
                                     double size_x = 0.1,
                                     double alpha = 1.0,
                                     bool print_ns = true) {
        (void)print_ns; // Suppress unused parameter warning
        
        Vec3f size = (box_max - box_min) / 2.0;
        Vec3f vis_pos_world = (box_min + box_max) / 2.0;
        double width = size.x();
        double length = size.y();
        double height = size.z();

        //Publish Bounding box
        int id = 0;
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.stamp = rclcpp::Clock().now();
        // line_strip.header.frame_id = "world";
        line_strip.header.frame_id = "camera_init";
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++; //unique id, useful when multiple markers exist.
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP; //marker type
        line_strip.scale.x = size_x;


        line_strip.color = color;
        line_strip.color.a = alpha; //不透明度，设0则全透明
        geometry_msgs::msg::Point p[8];

        //vis_pos_world是目标物的坐标
        p[0].x = vis_pos_world(0) - width;
        p[0].y = vis_pos_world(1) + length;
        p[0].z = vis_pos_world(2) + height;
        p[1].x = vis_pos_world(0) - width;
        p[1].y = vis_pos_world(1) - length;
        p[1].z = vis_pos_world(2) + height;
        p[2].x = vis_pos_world(0) - width;
        p[2].y = vis_pos_world(1) - length;
        p[2].z = vis_pos_world(2) - height;
        p[3].x = vis_pos_world(0) - width;
        p[3].y = vis_pos_world(1) + length;
        p[3].z = vis_pos_world(2) - height;
        p[4].x = vis_pos_world(0) + width;
        p[4].y = vis_pos_world(1) + length;
        p[4].z = vis_pos_world(2) - height;
        p[5].x = vis_pos_world(0) + width;
        p[5].y = vis_pos_world(1) - length;
        p[5].z = vis_pos_world(2) - height;
        p[6].x = vis_pos_world(0) + width;
        p[6].y = vis_pos_world(1) - length;
        p[6].z = vis_pos_world(2) + height;
        p[7].x = vis_pos_world(0) + width;
        p[7].y = vis_pos_world(1) + length;
        p[7].z = vis_pos_world(2) + height;
        //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
        for (int i = 0; i < 8; i++) {
            line_strip.points.push_back(p[i]);
        }
        //为了保证矩形框的八条边都存在：
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[3]);
        line_strip.points.push_back(p[2]);
        line_strip.points.push_back(p[5]);
        line_strip.points.push_back(p[6]);
        line_strip.points.push_back(p[1]);
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[7]);
        line_strip.points.push_back(p[4]);
        mkrarr.markers.push_back(line_strip);
    }
}
