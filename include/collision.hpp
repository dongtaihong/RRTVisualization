/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-17 16:23:39
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-04-17 20:20:35
 * @FilePath: /RRTVisualization/include/collision.hpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

constexpr double EPS = 1e-10;

bool isEqual(double x, double y) { return fabs(x - y) < EPS; }

/**
 * @description: 计算点 p 到线段 a-b 的距离
 * @return {*}
 */
double distanceToSegment(double px, double py, double x1, double y1, double x2,
                         double y2) {
  double cross = (x2 - x1) * (px - x1) + (y2 - y1) * (py - y1);
  if (cross <= 0) return sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));

  double length2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (cross >= length2)
    return sqrt((px - x2) * (px - x2) + (py - y2) * (py - y2));

  double r = cross / length2;
  double pxNear = x1 + (x2 - x1) * r;
  double pyNear = y1 + (y2 - y1) * r;

  return sqrt((px - pxNear) * (px - pxNear) + (py - pyNear) * (py - pyNear));
}

/**
 * @description: 计算点 p 到线段 a-b 的投影点
 * @return {*}
 */
pair<double, double> projectPointToSegment(double px, double py, double ax,
                                           double ay, double bx, double by) {
  double abx = bx - ax, aby = by - ay, apx = px - ax, apy = py - ay;
  double r = apx * abx + apy * aby;
  if (r <= 0) {
    return make_pair(ax, ay);
  } else if (r >= abx * abx + aby * aby) {
    return make_pair(bx, by);
  } else {
    double t = r / (abx * abx + aby * aby);
    return make_pair(ax + t * abx, ay + t * aby);
  }
}

/**
 * @description: 判断线段 AB 是否与圆心为 C、半径为
 * r的圆相交,原理：假设给定的线段为 AB，圆心坐标为 C，半径为 r。则可以先求出线段
 * AB 的长度，以及圆心到线段 AB 的垂线段 CD 的长度 h。若 h 大于圆的半径
 * r，则说明线段与圆不相交；否则，若点 D 在线段 AB
 * 上，则线段与圆相交；否则，若点 D 不在线段 AB 上，但点 C 到线段 AB
 * 的距离小于等于半径 r，则线段与圆相交；否则，线段与圆不相交。
 * @return {*}
 */
bool isLineIntersectCircle(double Ax, double Ay, double Bx, double By,
                           double Cx, double Cy, double r) {
  double AB = hypot(Bx - Ax, By - Ay);  // 线段 AB 的长度
  double AC = hypot(Cx - Ax, Cy - Ay);  // 点 C 到点 A 的距离
  double BC = hypot(Cx - Bx, Cy - By);  // 点 C 到点 B 的距离
  if (AC < r ||
      BC < r) {  // 点 A 或点 B 在圆内，或线段 AB 在圆内，则直接返回 true
    return true;
  }
  double h =
      distanceToSegment(Cx, Cy, Ax, Ay, Bx, By);  // 点 C 到线段 AB 的距离
  if (h > r) {                                    // 线段 AB 与圆不相交
    return false;
  }
  return true;
}
