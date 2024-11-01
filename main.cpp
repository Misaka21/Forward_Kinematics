//
// Created by david on 24-9-3.
//
#include <vector>
#include <iostream>
#include <stdexcept>
#include "1.hpp"



int main() {
	// 创建一个 3D 双精度浮点数坐标转换系统
	auto ct = GenericCoordinateTransform<double, 3>::create();


	// 添加转换矩阵 u -> a
	Eigen::Matrix4d transform_ua;
	transform_ua << 0.866, -0.5, 0, 11,
					0.5, 0.866, 0, -1,
					0, 0, 1, 8,
					0, 0, 0, 1;
	ct->add_transform("u", "a", transform_ua);
	ct->print_transform("a", "u");

	// 添加转换矩阵 b -> a
	Eigen::Matrix4d transform_ba;
	transform_ba<<1,0,0,0,
					0,0.866,-0.5,10,
					0,0.5,0.866,-20,
					0,0,0,1;

	ct->add_transform("b", "a", transform_ba);

	Eigen::Matrix4d transform_cu;
	transform_cu<<0.866,-0.5,0,-3,
					0.433,0.75,-0.5,-3,
					0.25,0.433,0.866,3,
					0,0,0,1;
	ct->add_transform("c", "u", transform_cu);
	// 故意设置错误的 A -> C 直接转换矩阵
	//Eigen::Matrix4d transform_ac_wrong = Eigen::Matrix4d::Identity();
	//transform_ac_wrong(0, 3) = 5.0; // 人为设置错误值
	//ct->addTransform("A", "C", transform_ac_wrong);

//	// 创建一些点
//	std::vector<Eigen::Vector3d> points = {
//			Eigen::Vector3d(0, 0, 0)
//	};
//// 转换点云从 A 到 C
//	auto transformed_points = ct->transform_point_cloud("b", "c", points);
//	std::cout << "Transformed points from A to C:" << std::endl;
//
	ct->print_transform("b", "c");
//
//// 打印转换链
	ct->print_transform_chain("b", "c");
//
//// 检查直接转换
//	std::cout << "Direct transform from A to C: " << (ct->has_direct_transform("A", "C") ? "Yes" : "No") << std::endl;
//
//// 获取所有坐标系
//	auto systems = ct->get_all_coordinate_systems();
//	std::cout << "All coordinate systems: ";
//	for (const auto& sys : systems) {
//		std::cout << sys << " ";
//	}
//	std::cout << std::endl;
//
//// 移除转换
//	ct->remove_transform("A", "B");

	return 0;
}