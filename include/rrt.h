/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-17 14:07:37
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-04-17 20:59:13
 * @FilePath: /RRTVisualization/include/rrt.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */

#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <tuple>
#include <vector>

using namespace std;

//地图
constexpr int map_w = 500;
constexpr int map_h = 500;
//矩形障碍物列表，假设障碍物都是多边形
// extern vector<vector<pair<double, double>>> PolygonObstacles;
//圆形障碍物列表
extern vector<tuple<double, double, double>> CircleObstacles;

//树结点
class Node {
 public:
  Node(double x_, double y_);
  ~Node() = default;
  double x;
  double y;
  Node* parent;
};

// RRT算法
class RRT {
 public:
  RRT(Node* start_, Node* end_, int step_size_, int max_iter_);
  ~RRT() = default;
  vector<Node*> SolvePath();

 private:
  Node* SampleNode(Node* node);  //在当前点附近随机采样一个点
  Node* NearNode(Node* node);    //在树中找到距离采样点最近的点
  Node* EdgeNode(Node* near_node, Node* sample_node);    //边的端点
  bool CollisionFree(Node* near_node, Node* edge_node);  //边的碰撞检测
  Node* start;
  Node* end;
  Node* cur;
  vector<Node*> tree;
  int step_size;
  int max_iter;
};