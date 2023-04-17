/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-17 14:07:37
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-04-17 21:03:40
 * @FilePath: /RRTVisualization/src/rrt.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include "rrt.h"

#include "collision.hpp"

vector<tuple<double, double, double>> CircleObstacles;
cv::Mat visualMap(map_h, map_w, CV_8UC3, cv::Scalar(255, 255, 255));

/**
 * @description: 建立障碍物地图
 * @return {*}
 */
void GridMap(Node* start_point, Node* end_point) {
  //起点、终点
  cv::Point start_point_(start_point->x, start_point->y);
  cv::Point end_point_(end_point->x, end_point->y);
  cv::circle(visualMap, start_point_, 10, cv::Scalar(0, 255, 0), -1);
  cv::circle(visualMap, end_point_, 10, cv::Scalar(0, 255, 0), -1);

  //障碍物
  for (auto& co : CircleObstacles) {
    cv::Point obstacle_tmp(get<0>(co), get<1>(co));
    cv::circle(visualMap, obstacle_tmp, get<2>(co), cv::Scalar(0, 0, 0), -1);
  }
}
/**
 * @description: 树结点构造函数
 * @param {double} x_
 * @param {double} y_
 * @return {*}
 */
Node::Node(double x_, double y_) : x(x_), y(y_), parent(nullptr) {}

/**
 * @description: RRT构造函数
 * @param {Node*} start_
 * @param {Node*} end_
 * @param {int} step_size_
 * @param {int} max_iter_
 * @return {*}
 */
RRT::RRT(Node* start_, Node* end_, int step_size_, int max_iter_) {
  start = start_;
  end = end_;
  step_size = step_size_;
  max_iter = max_iter_;
  tree.push_back(start_);
}

/**
 * @description: 在[0,map_w],[0,map_h]的地图上随机采一个生长点
 * @param {Node*} node
 * @return {*}
 */
Node* RRT::SampleNode(Node* node) {
  /** 在当前点的1.9step_size单位圆附近采样
  double scaled_x, scaled_y;
  bool flag = true;
  while (flag) {
    std::default_random_engine generator;  // 随机数生成器
    std::random_device rd;                 // 真随机种子
    if (rd.entropy() > 0.0) {
      generator.seed(rd());  // 如果有足够的熵，使用真随机种子
    } else {
      generator.seed(std::chrono::system_clock::now()
                         .time_since_epoch()
                         .count());  // 否则使用系统时钟作为种子
    }

    std::uniform_real_distribution<double> distribution(
        -1.0, 1.0);  // 均匀分布的随机数生成器
    double circle_radius = 1.9 * (double)step_size;  //采样圆的半径
    double x, y;
    do {
      x = distribution(generator);
      y = distribution(generator);
    } while (std::sqrt(x * x + y * y) >
             1.0);  // 只有当在单位圆内部才认为该点在圆内部

    scaled_x = x * circle_radius + node->x;
    scaled_y = y * circle_radius + node->y;
    if (scaled_x >= 0 && scaled_x <= map_w && scaled_y >= 0 &&
        scaled_y <= map_h)
      flag = false;
  }
  **/

  // 在[0,map_w],[0,map_h]的地图上随机采样
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> disX(0, 0 + map_w);
  uniform_int_distribution<> disY(0, 0 + map_h);
  int randomX = disX(gen);
  int randomY = disY(gen);
  Node* sampleNode = new Node(randomX, randomY);
  sampleNode->parent = node;
  return sampleNode;
}

/**
 * @description: 遍历所有结点找到最近的树结点
 * @param {Node*} node
 * @return {*}
 */
Node* RRT::NearNode(Node* node) {
  int flag = 0;
  double flag_distance = 10000;

  for (int i = 0; i < tree.size(); i++) {
    double distance = hypot(tree[i]->x - node->x, tree[i]->y - node->y);
    if (distance < flag_distance) {
      flag = i;
      flag_distance = distance;
    }
  }
  return tree[flag];
}

/**
 * @description: 往采样的方向挪动step_size形成edge
 * @param {Node*} near_node
 * @param {Node*} sample_node
 * @return {*}
 */
Node* RRT::EdgeNode(Node* near_node, Node* sample_node) {
  double vec_x = sample_node->x - near_node->x;
  double vec_y = sample_node->y - near_node->y;
  double length = hypot(vec_x, vec_y);
  double edge_x = near_node->x + step_size * vec_x / length;
  double edge_y = near_node->y + step_size * vec_y / length;
  Node* edgeNode = new Node(edge_x, edge_y);
  edgeNode->parent = near_node;
  return edgeNode;
}

/**
 * @description: 遍历障碍物，检查新线段是否发生碰撞
 * @param {Node*} near_node
 * @param {Node*} edge_node
 * @return {*}
 */
bool RRT::CollisionFree(Node* near_node, Node* edge_node) {
  // TODO：添加多边形障碍物，并进行碰撞检测
  // for (auto po : PolygonObstacles) {
  //   if (isLineIntersectPolygon(near_node->x, near_node->y, edge_node->x,
  //                              edge_node->y, po)) {
  //     return false;
  //   }
  // }
  for (auto& co : CircleObstacles) {
    if (isLineIntersectCircle(near_node->x, near_node->y, edge_node->x,
                              edge_node->y, get<0>(co), get<1>(co),
                              get<2>(co))) {
      return false;
    }
  }
  return true;
}

/**
 * @description: RRT算法执行
 * @return {*}
 */
vector<Node*> RRT::SolvePath() {
  cur = start;
  cv::namedWindow("RRT", cv::WINDOW_AUTOSIZE);
  GridMap(start, end);
  cv::imshow("RRT", visualMap);
  Node* sample_node_(new Node(0, 0));
  Node* near_node_(new Node(0, 0));
  Node* edge_node_(new Node(0, 0));
  for (int i = 0; i < max_iter; i++) {
    sample_node_ = SampleNode(cur);
    near_node_ = NearNode(sample_node_);
    edge_node_ = EdgeNode(near_node_, sample_node_);
    if (CollisionFree(near_node_, edge_node_)) {
      cur = edge_node_;
      tree.push_back(edge_node_);
      cv::Point edge_start(near_node_->x, near_node_->y);
      cv::Point edge_end(edge_node_->x, edge_node_->y);
      cv::line(visualMap, edge_start, edge_end, cv::Scalar(0, 0, 255), 2);
      cv::imshow("RRT", visualMap);
      cv::waitKey(1);
      //到达终点附近
      if (isLineIntersectCircle(near_node_->x, near_node_->y, edge_node_->x,
                                edge_node_->y, end->x, end->y, 10)) {
        cout << "寻找路径成功：" << endl;
        vector<Node*> globalPath;
        while (edge_node_ && edge_node_->parent) {
          globalPath.push_back(edge_node_);
          edge_node_ = edge_node_->parent;
        }
        globalPath.push_back(start);
        reverse(globalPath.begin(), globalPath.end());
        for (int i = 1; i < globalPath.size(); i++) {
          cv::Point edge_start1(globalPath[i - 1]->x, globalPath[i - 1]->y);
          cv::Point edge_end1(globalPath[i]->x, globalPath[i]->y);
          cv::line(visualMap, edge_start1, edge_end1, cv::Scalar(255, 0, 0), 3);
        }
        cv::imshow("RRT", visualMap);
        cv::waitKey(5000);
        cv::destroyWindow("RRT");
        return globalPath;
      }
    } else
      continue;
  }
  cout << "寻找路径失败： " << endl;
  return vector<Node*>();
}
