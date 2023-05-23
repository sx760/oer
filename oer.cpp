#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

// gurobi
#include "gurobi_c++.h"
#pragma comment(lib, "D:\\program\\gurobi950\\win64\\lib\\gurobi95.lib")
#pragma comment(lib, "D:\\program\\gurobi950\\win64\\lib\\gurobi_c++mdd2019.lib")

// easyx
#include <easyx.h>

// 工具
class Utils
{
public:
	static wchar_t *stringToLPCWSTR(std::string orig)
	{
		wchar_t *wcstring = 0;
		try
		{
			size_t origsize = orig.length() + 1;
			const size_t newsize = 100;
			size_t convertedChars = 0;
			if (orig == "")
			{
				wcstring = (wchar_t *)malloc(0);
				mbstowcs_s(&convertedChars, wcstring, origsize, orig.c_str(), _TRUNCATE);
			}
			else
			{
				wcstring = (wchar_t *)malloc(sizeof(wchar_t) * (orig.length() - 1));
				mbstowcs_s(&convertedChars, wcstring, origsize, orig.c_str(), _TRUNCATE);
			}
		}
		catch (std::exception e)
		{
		}
		return wcstring;
	}
};

class BGA
{
public:
	BGA(int width, int height, int pinNum, std::vector<std::vector<int>> &pins) : width_(width), height_(height), pinNum_(pinNum), pins_(pins)
	{
	}

public:
	int width_;
	int height_;
	int pinNum_;
	std::vector<std::vector<int>> pins_; // pin:pins_[w][h] = pinNet; no-pin:pins_[w][h] = -1
};

// 点
class Point
{
public:
	Point(int x, int y, int layer = 0) : x_(x), y_(y), layer_(layer)
	{
	}

	bool operator==(const Point &other) const
	{
		return layer_ == other.layer_ && x_ == other.x_ && y_ == other.y_;
	}

public:
	int x_; // -1无意义
	int y_; // -1无意义

	int layer_; // 层, -1无意义, 0顶层
};

// 节点
class Node : public Point
{
public:
	Node() : Point(-1, -1, -1), name_(""), cap_(0), net_(-1), flag_(0) {}
	Node(int x, int y, int layer, const std::string &name, int cap = 2, int net = -1, int flag = 0)
		: Point(x, y, layer), name_(name), cap_(cap), net_(net), flag_(flag)
	{
	}

	bool operator==(const Node& other) const
	{
		return name_ == other.name_;
	}

	bool isImg() const
	{
		return x_ == -1 && y_ == -1;
	}

public:
	std::string name_;

	int cap_;  // 1:起点/终点    2:其它
	int net_;  // 线网, -1无意义
	int flag_; // 1:起点 -1:终点 0:其它
};

// 边
class Edge
{
public:
	Edge(Node n1, Node n2, int cap = 1, double cost = 1.0) : n1_(n1), n2_(n2), cap_(cap), cost_(cost)
	{
	}

	bool isVia() const
	{
		return n1_.layer_ != n2_.layer_;
	}

	bool isImg() const
	{
		return n1_.x_ == -1 && n2_.x_ == -1;
	}

public:
	Node n1_;
	Node n2_;
	int cap_;
	double cost_;
};

// 有序逃逸
class OER
{
public:
	enum State
	{
		sSuccess = 0,
		sFail = 1
	};

public:
	OER(int width, int height, int pinNum, std::vector<std::vector<int>> &pins, int maxLayer = 1)
		: bga_(BGA(width, height, pinNum, pins)), maxLayer_(maxLayer)
	{
		state_ = sFail;
		// 初始化nodeTable
		nodeTable_.resize(width);
		for (int w = 0; w < width; ++w)
		{
			nodeTable_[w].resize(height);
			for (int h = 0; h < height; ++h)
			{
				nodeTable_[w][h].resize(maxLayer);
			}
		}
		initGraph();
		initFlow();
	}

	void run()
	{
		//return;
		try
		{
			GRBEnv env = GRBEnv(true);
			std::string logFile = "oer.log";
			env.set(GRB_StringParam_LogFile, logFile);
			env.start();
			GRBModel model = GRBModel(env);

			model.set(GRB_DoubleParam_TimeLimit, 60 * 60 * 1);
			model.set(GRB_IntParam_Threads, 10);
			model.set(GRB_DoubleParam_MIPGap, 0.05);
			model.set(GRB_IntParam_LogToConsole, 0);

			// 决策变量
			std::vector<std::vector<GRBVar>> x;
			x.resize(bga_.pinNum_);
			for (int p = 0; p < bga_.pinNum_; ++p)
			{
				x[p].resize(edges_.size());
				for (int e = 0; e < edges_.size(); ++e)
				{
					x[p][e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + std::to_string(p) + "_" + std::to_string(e));
				}
			}

			// 优化目标
			GRBLinExpr obj = 0;
			for (int p = 0; p < bga_.pinNum_; ++p)
			{
				for (int e = 0; e < edges_.size(); ++e)
				{
					obj += x[p][e] * edges_[e].cost_;
				}
			}
			model.setObjective(obj, GRB_MINIMIZE);

			// 约束条件
			// 1,2,3
			std::vector<Edge> es1, es2;
			std::vector<int> es1Indexs, es2Indexs;
			for (int p = 0; p < bga_.pinNum_; ++p)
			{
				for (int n = 0; n < nodes_.size(); ++n)
				{
					GRBLinExpr netOpen = 0;
					edgeOutAndIn(nodes_[n], es1, es1Indexs, es2, es2Indexs);
					for (int m = 0; m < es1.size(); ++m)
					{
						netOpen += x[p][es1Indexs[m]];
					}
					for (int m = 0; m < es2.size(); ++m)
					{
						netOpen -= x[p][es2Indexs[m]];
					}
					if (netOpen.size())
					{
						model.addConstr(netOpen == flow_[p][n], "netOpen_" + std::to_string(p) + "_" + std::to_string(n));
					}
				}
			}
			// 5
			for (int n = 0; n < nodes_.size(); ++n)
			{
				GRBLinExpr nodeShort = 0;
				for (int p = 0; p < bga_.pinNum_; ++p)
				{
					edgeOutAndIn(nodes_[n], es1, es1Indexs, es2, es2Indexs);
					for (int m = 0; m < es1.size(); ++m)
					{
						//nodeShort += x[p][edgeIndex(es1[m])];
						nodeShort += x[p][es1Indexs[m]];
					}
					for (int m = 0; m < es2.size(); ++m)
					{
						nodeShort += x[p][es2Indexs[m]];
					}
				}
				if (nodeShort.size())
				{
					model.addConstr(nodeShort <= nodes_[n].cap_, "nodeShort_" + std::to_string(n));
				}
			}
			// 4
			for (int e = 0; e < edges_.size(); ++e)
			{
				GRBLinExpr edgeCapacity = 0;
				for (int p = 0; p < bga_.pinNum_; ++p)
				{
					edgeCapacity += x[p][e];
				}
				if (edgeCapacity.size())
				{
					model.addConstr(edgeCapacity <= 1/*edges_[e].cap_*/, "edgeCapacity_" + std::to_string(e));
				}
			}

			//model.write("oer.lp");

			model.optimize(); // 求解
			// 有可行解
			if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL ||
				((model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) && model.get(GRB_IntAttr_SolCount) > 0))
			{
				state_ = sSuccess;
				route_.resize(bga_.pinNum_);
				for (int p = 0; p < bga_.pinNum_; ++p)
				{
					route_[p].resize(edges_.size());
					for (int e = 0; e < edges_.size(); ++e)
					{
						if (x[p][e].get(GRB_DoubleAttr_X) == 1)
						{
							route_[p][e] = 1;
						}
						else
						{
							route_[p][e] = 0;
						}
					}
				}
			}
			else // 无可行解
			{
				state_ = sFail;
			}
		}
		catch (GRBException e)
		{
			std::cout << "Error code = " << e.getErrorCode() << std::endl;
			std::cout << e.getMessage() << std::endl;
		}
		catch (...)
		{
			std::cout << "Exception during optimization" << std::endl;
		}
	}

public:
	void initGraph()
	{
		for (int l = 0; l < maxLayer_; ++l)
		{
			initGraph(l);
		}
		initVia(); // 通孔

		if (escNodeTop_.size() + escNodeRight_.size() + escNodeBottom_.size() + escNodeLeft_.size() > bga_.pinNum_)
		{
			initGraphImg(); // 虚拟节点
		}
	}

	// 图
	void initGraph(int layer)
	{
		for (int w = 0; w < bga_.width_; ++w)
		{
			for (int h = 0; h < bga_.height_; ++h)
			{
				Node newNode(w, h, layer, "node_" + std::to_string(layer) + "_" + std::to_string(w) + "_" + std::to_string(h));
				if (layer == 0 && bga_.pins_[w][h] != -1) // pin起点
				{
					newNode.cap_ = 1;
					newNode.net_ = bga_.pins_[w][h];
					newNode.flag_ = 1;
				}
				nodeTable_[w][h][layer] = newNode;
				nodes_.push_back(newNode);

				// 同一层边
				if (isIndexLegal(w - 1, h, layer)) // 1
				{
					edges_.push_back(Edge(nodeTable_[w - 1][h][layer], nodeTable_[w][h][layer]));
					edges_.push_back(Edge(nodeTable_[w][h][layer], nodeTable_[w - 1][h][layer]));
				}
				if (isIndexLegal(w, h - 1, layer)) // 2
				{
					edges_.push_back(Edge(nodeTable_[w][h - 1][layer], nodeTable_[w][h][layer]));
					edges_.push_back(Edge(nodeTable_[w][h][layer], nodeTable_[w][h - 1][layer]));
				}

				// 边界
				if (w == 0)
				{
					escNodeLeft_.push_back(newNode);
				}
				if (w == bga_.width_ - 1)
				{
					escNodeRight_.push_back(newNode);
				}
				if (h == 0)
				{
					escNodeTop_.push_back(newNode);
				}
				if (h == bga_.height_ - 1)
				{
					escNodeBottom_.push_back(newNode);
				}
			}
		}
	}

	// 通孔
	void initVia()
	{
		for (int l = 1; l < maxLayer_; ++l)
		{
			std::vector<std::vector<int>> vis(bga_.width_, std::vector<int>(bga_.height_, 0));
			for (int w = 0; w < bga_.width_; ++w)
			{
				for (int h = 0; h < bga_.height_; ++h)
				{
					if (bga_.pins_[w][h] == 1 && vis[w][h] == 0) // pin
					{
						if (isIndexLegal(w - 1, h, l - 1) && isIndexLegal(w - 1, h, l)) // 1
						{
							vis[w - 1][h] = 1;
							Edge viaEdge(nodeTable_[w - 1][h][l - 1], nodeTable_[w - 1][h][l], 1, cViaCost);
							edges_.push_back(viaEdge);
						}
						if (isIndexLegal(w + 1, h, l - 1) && isIndexLegal(w + 1, h, l)) // 2
						{
							vis[w + 1][h] = 1;
							Edge viaEdge(nodeTable_[w + 1][h][l - 1], nodeTable_[w + 1][h][l], 1, cViaCost);
							edges_.push_back(viaEdge);
						}
						if (isIndexLegal(w, h - 1, l - 1) && isIndexLegal(w, h - 1, l)) // 3
						{
							vis[w][h - 1] = 1;
							Edge viaEdge(nodeTable_[w][h - 1][l - 1], nodeTable_[w][h - 1][l], 1, cViaCost);
							edges_.push_back(viaEdge);
						}
						if (isIndexLegal(w, h + 1, l - 1) && isIndexLegal(w, h + 1, l)) // 4
						{
							vis[w][h + 1] = 1;
							Edge viaEdge(nodeTable_[w][h + 1][l - 1], nodeTable_[w][h + 1][l], 1, cViaCost);
							edges_.push_back(viaEdge);
						}
					}
				}
			}
		}
		
	}

	// 虚拟节点、边
	void initGraphImg()
	{
		sortEscNode();									// 排序
		int count = 0;									// 当前行虚拟节点数量
		std::vector<std::vector<std::vector<Node>>> ns; // ns[l][i][j]第l层第i行第j个虚拟节点
		ns.resize(maxLayer_);
		for (int l = 0; l < maxLayer_; ++l)
		{
			count = escNode_[l].size() - 1;
			ns[l].resize(escNode_[l].size() - bga_.pinNum_);
			while (count >= bga_.pinNum_)
			{
				ns[l][(escNode_[l].size() - 1) - count].resize(count);
				if (count == escNode_[l].size() - 1) // 第一行
				{
					for (int i = 0; i < count; ++i)
					{
						Node newImgNode(-1, -1, l, "imgNode_" + std::to_string(l) + "_" + std::to_string(count) + "_" + std::to_string(i));
						nodes_.push_back(newImgNode);
						edges_.push_back(Edge(escNode_[l][i], newImgNode, 1, cImgEdgeCost));
						edges_.push_back(Edge(escNode_[l][i + 1], newImgNode, 1, cImgEdgeCost));
						// 记录虚拟节点
						ns[l][(escNode_[l].size() - 1) - count][i] = newImgNode;
					}
				}
				else
				{
					for (int i = 0; i < count; ++i)
					{
						Node newImgNode(-1, -1, l, "imgNode_" + std::to_string(l) + "_" + std::to_string(count) + "_" + std::to_string(i));
						nodes_.push_back(newImgNode);
						edges_.push_back(Edge(ns[l][(escNode_[l].size() - 1) - count - 1][i], newImgNode, 1, cImgEdgeCost));
						edges_.push_back(Edge(ns[l][(escNode_[l].size() - 1) - count - 1][i + 1], newImgNode, 1, cImgEdgeCost));
						// 记录虚拟节点
						ns[l][(escNode_[l].size() - 1) - count][i] = newImgNode;
					}
				}
				--count; // 逐行递减
			}
		}

		for (int p = 0; p < bga_.pinNum_; ++p) // 终点的虚拟节点
		{
			Node newImgNode(-1, -1, -1, "lastImgNode_" + std::to_string(p));
			newImgNode.cap_ = 1;
			newImgNode.net_ = p;
			newImgNode.flag_ = -1;
			nodes_.push_back(newImgNode);

			for (int l = 0; l < maxLayer_; ++l)
			{
				edges_.push_back(Edge(ns[l][escNode_[l].size() - bga_.pinNum_ - 1][p], newImgNode, 1, cImgEdgeCost));
			}
		}
	}

	// 逃逸节点排序
	void sortEscNode()
	{
		// 1
		std::sort(escNodeTop_.begin(), escNodeTop_.end(), [](const Node &n1, const Node &n2)
				  {	if (n1.layer_ == n2.layer_)
			{
				return n1.x_ < n2.x_;
			}
			else
			{
				return n1.layer_ < n2.layer_;
			} });
		// 2
		std::sort(escNodeBottom_.begin(), escNodeBottom_.end(), [](const Node &n1, const Node &n2)
				  {	if (n1.layer_ == n2.layer_)
		{
			return n1.x_ > n2.x_;
		}
		else
		{
			return n1.layer_ < n2.layer_;
		} });
		// 3
		std::sort(escNodeLeft_.begin(), escNodeLeft_.end(), [](const Node &n1, const Node &n2)
				  {	if (n1.layer_ == n2.layer_)
		{
			return n1.y_ > n2.y_;
		}
		else
		{
			return n1.layer_ < n2.layer_;
		} });
		// 4
		std::sort(escNodeRight_.begin(), escNodeRight_.end(), [](const Node &n1, const Node &n2)
				  {	if (n1.layer_ == n2.layer_)
		{
			return n1.y_ < n2.y_;
		}
		else
		{
			return n1.layer_ < n2.layer_;
		} });

		escNode_.resize(maxLayer_);
		for (int l = 0; l < maxLayer_; ++l)
		{
			// 1
			for (int n = 0; n < escNodeTop_.size(); ++n)
			{
				if (escNodeTop_[n].layer_ == l)
				{
					escNode_[l].push_back(escNodeTop_[n]);
				}
			}
			// 2
			for (int n = 1; n < escNodeRight_.size(); ++n)
			{
				if (escNodeRight_[n].layer_ == l)
				{
					escNode_[l].push_back(escNodeRight_[n]);
				}
			}
			// 3
			for (int n = 1; n < escNodeBottom_.size(); ++n)
			{
				if (escNodeBottom_[n].layer_ == l)
				{
					escNode_[l].push_back(escNodeBottom_[n]);
				}
			}
			// 4
			for (int n = 1; n < escNodeLeft_.size() - 1; ++n)
			{
				if (escNodeLeft_[n].layer_ == l)
				{
					escNode_[l].push_back(escNodeLeft_[n]);
				}
			}
		}
	}

	// 初始化flow
	void initFlow()
	{
		flow_.resize(bga_.pinNum_);
		for (int p = 0; p < bga_.pinNum_; ++p)
		{
			flow_[p].resize(nodes_.size());
			for (int n = 0; n < nodes_.size(); ++n)
			{
				if (nodes_[n].net_ == p)
				{
					flow_[p][n] = nodes_[n].flag_;
				}
				else
				{
					flow_[p][n] = 0;
				}
			}
		}
	}

	// 索引是否合法
	bool isIndexLegal(int w, int h, int layer) const
	{
		if (w >= 0 && w < bga_.width_ && h >= 0 && h < bga_.height_ && layer >= 0 && layer < maxLayer_)
		{
			return true;
		}
		return false;
	}

	// 以node为起点的边的集合
	void edgeOut(const Node &node, std::vector<Edge> &es) const
	{
		for (auto e : edges_)
		{
			if (node == e.n1_)
			{
				es.push_back(e);
			}
		}
	}

	// 以node为终点的边的集合
	void edgeIn(const Node &node, std::vector<Edge> &es) const
	{
		for (auto e : edges_)
		{
			if (node == e.n2_)
			{
				es.push_back(e);
			}
		}
	}

	void edgeOutAndIn(
		const Node &node, 
		std::vector<Edge> &es1, 
		std::vector<int> &es1Indexs, 
		std::vector<Edge> &es2, 
		std::vector<int> &es2Indexs) const
	{
		es1.clear();
		es1Indexs.clear();
		es2.clear();
		es2Indexs.clear();
		for (int e = 0; e < edges_.size(); ++e)
		{
			if (node == edges_[e].n1_)
			{
				es1.push_back(edges_[e]);
				es1Indexs.push_back(e);
			}
			else if (node == edges_[e].n2_)
			{
				es2.push_back(edges_[e]);
				es2Indexs.push_back(e);
			}
			else
			{
				continue;
			}
		}
	}

	// 获取edge的索引
	int edgeIndex(const Edge &edge) const
	{
		for (int e = 0; e < edges_.size(); e++)
		{
			if (edge.n1_ == edges_[e].n1_ && edge.n2_ == edges_[e].n2_)
			{
				return e;
			}
		}
		return -1;
	}

public:

	const static int cViaCost = 2;
	const static int cImgEdgeCost = 0;

	int maxLayer_;
	BGA bga_;

	std::vector<Node> nodes_;
	std::vector<Node> escNodeTop_; // 四个方向上的逃逸节点
	std::vector<Node> escNodeRight_;
	std::vector<Node> escNodeBottom_;
	std::vector<Node> escNodeLeft_;
	std::vector<std::vector<Node>> escNode_;				// 每一层的逃逸节点
	std::vector<std::vector<std::vector<Node>>> nodeTable_; // nodeTable_[w][h][layer] = Node
	std::vector<Edge> edges_;
	std::vector<std::vector<int>> flow_; // flow_[p][n] = -1,0,1

	State state_;
	std::vector<std::vector<int>> route_;
};

class Painter
{
public:
	void draw(const OER &oer, int zoom)
	{
		zoom_ = zoom;
		Resize(&img_, oer.bga_.width_ * zoom + zoom, oer.bga_.height_ * zoom + zoom);
		SetWorkingImage(&img_);
		setorigin(0, 0);
		setaspectratio(1.0, 1.0);
		setbkcolor(WHITE);
		cleardevice();

		drawBGA(oer.bga_);
		//drawByLayer(oer);
		drawRoute(oer, 0);

		//saveimage(_T("route.bmp"), &img_);
		//ShellExecuteA(NULL, "open", "route.bmp", NULL, ".\\", SW_SHOW);
	}

public:
	void drawByLayer(const OER &oer)
	{
		for (int l = 0; l < oer.maxLayer_; ++l)
		{
			drawRoute(oer, l);
			std::string imgFile = "route_" + std::to_string(l) + ".bmp";
			saveimage(Utils::stringToLPCWSTR(imgFile), &img_);
		}
	}

	void drawBGA(const BGA &bga)
	{
		setlinestyle(PS_SOLID, 2);
		setlinecolor(BLACK);
		//rectangle(zoom_ - 20, zoom_ - 20, bga.width_ * zoom_ + 20, bga.height_ * zoom_ + 20);
		rectangle(zoom_, zoom_, bga.width_ * zoom_, bga.height_ * zoom_);
		setfillcolor(CYAN);
		setlinecolor(CYAN);
		for (int w = 0; w < bga.width_; ++w)
		{
			for (int h = 0; h < bga.height_; ++h)
			{
				fillcircle(w * zoom_ + zoom_, h * zoom_ + zoom_, 10);
			}
		}
		drawText(bga);
	}

	void drawRoute(const OER &oer, int layer)
	{
		if (oer.state_ == OER::sFail)
		{
			return;
		}
		setlinestyle(PS_SOLID, 4);
		for (int p = 0; p < oer.bga_.pinNum_; ++p)
		{
			for (int e = 0; e < oer.edges_.size(); ++e)
			{
				if (oer.edges_[e].isImg() || oer.edges_[e].n1_.isImg() || oer.edges_[e].n2_.isImg())
				{
					continue;
				}
				
				if (oer.route_[p][e] == 1)
				{
					if (oer.edges_[e].isVia())
					{
						drawVia(oer.edges_[e]);
						continue;
					}
					if (oer.edges_[e].n1_.layer_ != layer)
					{
						continue;
					}
					setLayerColor(layer);
					line(
						zoom_ + oer.edges_[e].n1_.x_ * zoom_,
						zoom_ + oer.edges_[e].n1_.y_ * zoom_,
						zoom_ + oer.edges_[e].n2_.x_ * zoom_,
						zoom_ + oer.edges_[e].n2_.y_ * zoom_);
				}
			}
		}
		std::string imgFile = "route.bmp";
		//std::string imgFile = "route_" + std::to_string(layer) + ".bmp";
		saveimage(Utils::stringToLPCWSTR(imgFile), &img_);
	}

	void drawVia(const Edge e)
	{
		setlinecolor(BROWN);
		setfillcolor(BROWN);
		fillcircle(e.n1_.x_ * zoom_ + zoom_, e.n1_.y_ * zoom_ + zoom_, 6);
	}

	void drawText(const BGA& bga)
	{
		settextcolor(BLACK);
		settextstyle(20, 0, _T("Consolas"));
		for (int w = 0; w < bga.width_; ++w)
		{
			for (int h = 0; h < bga.height_; ++h)
			{
				if (bga.pins_[w][h] == -1)
				{
					continue;
				}
				TCHAR s[5];
				swprintf_s(s, _T("%d"), bga.pins_[w][h]);
				outtextxy(w * zoom_ + zoom_, h * zoom_ + zoom_, s);
			}
		}
	}

	void setLayerColor(int layer)
	{
		switch (layer)
		{
		case 0:
			setlinecolor(RED);
			break;
		case 1:
			setlinecolor(BLUE);
			break;
		case 2:
			setlinecolor(YELLOW);
			break;
		case 3:
			setlinecolor(GREEN);
			break;
		default:
			break;
		}
	}

public:
	IMAGE img_;
	int zoom_;
};

int main()
{
	// 单层
	int width = 8, height = 8, pinNum = 6;
	std::vector<std::vector<int>> pins(width);
	for (int w = 0; w < width; ++w)
	{
		pins[w].resize(height);
		std::fill(pins[w].begin(), pins[w].end(), -1);
	}
	pins[1][1] = 0;
	pins[5][2] = 1;
	pins[4][4] = 2;
	pins[3][3] = 3;
	pins[2][6] = 4; 
	pins[3][2] = 5; 

	// 多层, 目前只能做到层内有序, 待解决层间的有序
	/*int width = 8, height = 8, pinNum = 8;
	std::vector<std::vector<int>> pins(width);
	for (int w = 0; w < width; ++w)
	{
		pins[w].resize(height);
		std::fill(pins[w].begin(), pins[w].end(), -1);
	}
	pins[1][1] = 0;
	pins[5][2] = 1;
	pins[4][4] = 2;
	pins[2][3] = 3;
	pins[2][6] = 4; 
	pins[3][2] = 5; 
	pins[5][5] = 6; 
	pins[6][3] = 7; */

	OER oer(width, height, pinNum, pins, 1);
	oer.run();

	Painter p;
	p.draw(oer, 100);

	return 0;
}
