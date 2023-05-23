#include <iostream>
#include <string>
#include <vector>

// gurobi
#include "gurobi_c++.h"
#pragma comment(lib, "D:\\program\\gurobi950\\win64\\lib\\gurobi95.lib")
#pragma comment(lib, "D:\\program\\gurobi950\\win64\\lib\\gurobi_c++mdd2019.lib")

// easyx
#include <easyx.h>

class BGA
{
public:
	BGA(int width, int height, int pinNum, std::vector<std::vector<int>>& pins) : width_(width), height_(height), pinNum_(pinNum), pins_(pins)
	{
	}

public:
	int width_;
	int height_;
	int pinNum_;
	std::vector<std::vector<int>> pins_; // pin:pins_[w][h] = 1; no-pin:pins_[w][h] = 0

};


// 点
class Point
{
public:
	Point(int x, int y, int layer = 0) : x_(x), y_(y), layer_(layer)
	{}

public:
	int x_;
	int y_;

	int layer_; // 层

};

// 节点
class Node : public Point
{
public:
	Node(int x, int y, const std::string& name, int cap = 2, int flag = 0)
		: Point(x, y), name_(name), cap_(cap), flag_(flag)
	{
	}

public:
	std::string name_;

	int cap_;  // 1:起点/终点    2:其它
	int flag_; // 1:起点 -1:终点 0:其它

};

// 边
class Edge
{
public:
	Edge(Node n1, Node n2, double cost = 1.0) : n1_(n1), n2_(n2), cost_(cost)
	{}

public:
	Node n1_;
	Node n2_;
	double cost_;

};

// 有序逃逸
class OER
{
public:
	OER(int width, int height, int pinNum, std::vector<std::vector<int>>& pins) 
		: bga_(BGA(width, height, pinNum, pins))
	{
		initGraph();
		initGraphImg();
	}

	int run()
	{}

public:
	void initGraph()
	{

		for (int w = 0; w < bga_.width_; ++w)
		{
			for (int h = 0; h < bga_.height_; ++h)
			{
				Node newNode(w, h, "node_" + std::to_string(w + 1) + "_" + std::to_string(h + 1));
				if (bga_.pins_[w][h] == 1)
				{
					newNode.cap_ = 1;
					newNode.flag_ = 1;
				}
				nodes_.push_back(newNode);
			}
		}
	}
	void initGraphImg()
	{}

public:
	BGA bga_;

	std::vector<Node> nodes_;
	std::vector<Node> escNode_;
	std::vector<Edge> edges_;
	std::vector<std::vector<int>> route_;

};

class Painter
{
public:


};


int main()
{
}
