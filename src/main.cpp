/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-17 14:07:37
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-04-17 21:19:31
 * @FilePath: /RRTVisualization/src/main.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include "rrt.h"

void GenerateObstacle() {
  auto obstacle1 = make_tuple(50, 50, 30);
  auto obstacle2 = make_tuple(150, 250, 60);
  auto obstacle3 = make_tuple(350, 150, 70);
  auto obstacle4 = make_tuple(320, 350, 40);
  auto obstacle5 = make_tuple(400, 400, 40);
  auto obstacle6 = make_tuple(100, 100, 50);
  CircleObstacles.push_back(obstacle1);
  CircleObstacles.push_back(obstacle2);
  CircleObstacles.push_back(obstacle3);
  CircleObstacles.push_back(obstacle4);
  CircleObstacles.push_back(obstacle5);
  CircleObstacles.push_back(obstacle6);
}

int main(int argc, const char* argv[]) {
  GenerateObstacle();
  Node* startPoint(new Node(10, 10));
  Node* endPoint(new Node(map_w - 10, map_h - 10));
  RRT rrt(startPoint, endPoint, 5, 100000);
  vector<Node*> final_path = rrt.SolvePath();

  // 释放指针空间
  delete startPoint;
  delete endPoint;
  const int vecLength = final_path.size();
  for (int i = 1; i < vecLength; i++) {
    delete final_path[vecLength - i];
  }
  final_path.clear();
  return 0;
}