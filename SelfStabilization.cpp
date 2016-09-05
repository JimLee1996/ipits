#include <iostream>
#include <iomanip>
//#include "afx.h"
#include <fstream>
using namespace std;


/////功能函数/////

/*
*
*功能	从大到小排序
*
*用途	换道时，确定2车道中位于1车道n车前车的位置
*
*/

void sort(double arr[], int n)     
{
	int i, j, k;
	double w;
	for (i = 0; i<n - 1; i++)
	{
		k = i;
		for (j = i + 1; j<n; j++)
		if (arr[k]<arr[j])      k = j;
		if (i != k)
		{
			w = arr[i];
			arr[i] = arr[k];
			arr[k] = w;
		}
	}
}

/////车辆类定义与实现/////

/*
*
*功能	定义车辆类
*
*成分	编号 位置 速度 加速度 前车车头距离 状态 车道 unKnown
*
*/

class Vehicle : public CObject
{
public:
	Vehicle(int number, double position, double speed, 
			double acceleration, double headway,bool condition, bool lane, int counts);
	~Vehicle();
	int	   Number;
	double Position[3];
	double Speed[301];
	double Headway;
	double Acceleration;
	bool   FastorSlow;//fast=1,slow=0
	bool   LaneNumber;//left=0,right=1
	int    Countsum;
};

Vehicle::Vehicle(int number, double position, double speed,
				double acceleration, double headway, bool condition, bool lane, int counts)
{
	Number = number;
	Position[0] = position;
	Position[1] = position;
	Position[2] = position;
	for (int i = 0; i < 301; i++)
		Speed[i] = speed;
	Acceleration = acceleration;
	Headway = headway;
	FastorSlow = condition;
	LaneNumber = lane;
	Countsum=counts;
}

Vehicle::~Vehicle()
{
}

/////车道类定义与实现/////

class Road : public CObject
{
public:
	Road();
	~Road();

	void AddVehicle(CObList* &plist, const int &carSum, const double &Length, const double &carSpeedF, const double &carHeadway, const bool &lanenumber);
	void Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao);
	void Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao);
	void InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar);
	void InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i);
	void LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, const double &xfc, const double L, int i);
	
	CObList queue;
};

Road::Road()
{
}

Road::~Road()
{
}

void Road::AddVehicle(CObList* &plist, const int &carSum, const double &Length, 
					  const double &carSpeedF, const double &carHeadway, const bool &lanenumber)
{
	for (int i = 1; i<carSum + 1; i++)
	{
		if (lanenumber == 0)
		{
			Vehicle* vehicle = new Vehicle(i, Length + carHeadway / 2.0 - i*carHeadway, carSpeedF, 0, carHeadway, 1, lanenumber, 0);
			plist->AddTail(vehicle);
		}
		else
		{
			Vehicle* vehicle = new Vehicle(i + 1000, Length - (i - 1)*carHeadway, carSpeedF, 0, carHeadway, 1, lanenumber, 0);
			plist->AddTail(vehicle);
		}
	}
}

void Road::Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao)
{
	ofstream outfile1("left1.txt", ofstream::out | ofstream::app);
	ofstream outfile2("right1.txt", ofstream::out | ofstream::app);
	ofstream outfile3("left2.txt", ofstream::out | ofstream::app);
	ofstream outfile4("left3.txt", ofstream::out | ofstream::app);
	ofstream outfile5("right3.txt", ofstream::out | ofstream::app);
	ofstream outfile6("right2.txt", ofstream::out | ofstream::app);

	
	double OptimumV;
	double t = 0.01;
	double v01 = 7.9, m1 = 0.13;
	double bc1 = 7.3, bf1 = 17;
	double lambda1 = 0.3;
	double lambda2 = 0.3;
	double k1, k2, k3, k4;
	double a1 = 1.0;
	double a2 = 1.5;

	POSITION post = plist->GetTailPosition();
	Vehicle* FrontalCar = (Vehicle*)(plist->GetAt(post));  //第一辆车的前车

	Vehicle* thisCar;
	POSITION pos = plist->GetHeadPosition();
	
	int N = planechange->GetCount();

	while (pos)
	{
		thisCar = (Vehicle*)(plist->GetNext(pos));

		if (thisCar->Position[0] >= L)
			thisCar->Position[0] = thisCar->Position[0] - L;
		if (FrontalCar->Position[0] >= L)
			FrontalCar->Position[0] = FrontalCar->Position[0] - L;
		//计算当前车与前车的车间距
		thisCar->Headway = FrontalCar->Position[0] - thisCar->Position[0];
		////???///////
		if (thisCar->Headway<-L / 2)
			thisCar->Headway += L;

		//加入扰动，第25辆车与前车的车间距减至原始车间距的三分之一
		if (i >= 200 && i <= 300)
		{
			if (thisCar->Number == 25)
			{
				thisCar->Headway = thisCar->Headway / 3;
				thisCar->Position[0] = thisCar->Position[0] + 2 * thisCar->Headway;
				//判断是否超出
				if (thisCar->Position[0] >= L)
					thisCar->Position[0] = thisCar->Position[0] - L;
			}
		}



		POSITION posNL = planechange->GetHeadPosition();
		double *position = new double[N + 1];
		int p = 0;
		while (posNL!=NULL)
		{
			Vehicle* car = (Vehicle*)(planechange->GetNext(posNL));
			position[p] = car->Position[0];
			p++;
		}
		position[N] = thisCar->Position[0];
		double frontcarpos, DistanceF;
		sort(position, N + 1);
		for (int k = 0; k< N + 1; k++)
		{
			if (position[k] == thisCar->Position[0])
			{
				if (k == 0)
					frontcarpos = position[N];
				else
					frontcarpos = position[k - 1];
				break;
			}
		}
		delete[] position;
		//计算本车与相邻车道最近前车的车间距离
		DistanceF = frontcarpos - thisCar->Position[0];
		if (DistanceF<0)
			DistanceF+=L;
		//优化速度计算
		OptimumV = v01*(tanh(m1*(ay*thisCar->Headway + aq*DistanceF - bf1)) - tanh(m1*(bc1 - bf1)));
		//加速度计算
		if (thisCar->LaneNumber == 0)
			thisCar->Acceleration = a1*(OptimumV - thisCar->Speed[0]) + lambda1*(thisCar->Speed[0] - thisCar->Speed[tao]);
		else
			thisCar->Acceleration = a2*(OptimumV - thisCar->Speed[0]) + lambda2*(thisCar->Speed[0] - thisCar->Speed[tao]);
		
		//速度计算，四次龙格库塔
		if (thisCar->LaneNumber == 0)
		{
			k1 = t*(a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			k2 = t*(a1*(OptimumV - thisCar->Speed[tao] - 0.5*k1) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			k3 = t*(a1*(OptimumV - thisCar->Speed[tao] - 0.5*k2) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			k4 = t*(a1*(OptimumV - thisCar->Speed[tao] - k3) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			thisCar->Speed[tao] = thisCar->Speed[tao - 1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);
		}
		else
		{
			k1 = t*(a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			k2 = t*(a2*(OptimumV - thisCar->Speed[tao] - 0.5*k1) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			k3 = t*(a2*(OptimumV - thisCar->Speed[tao] - 0.5*k2) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			k4 = t*(a2*(OptimumV - thisCar->Speed[tao] - k3) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			thisCar->Speed[tao] = thisCar->Speed[tao - 1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);
		}

		//更新速度和位置
		if (thisCar->Speed[tao] <= 0)
		{
			thisCar->Position[tao] = thisCar->Position[tao - 1];
			thisCar->Speed[tao] = 0;
		}
		else
			thisCar->Position[1] = thisCar->Position[0] + thisCar->Speed[tao] * t + 0.5*thisCar->Acceleration * t * t;

		if (thisCar->Position[1] >= L)
			thisCar->Position[1] = thisCar->Position[1] - L;

		for (int j = 1; j<tao; j++)
			thisCar->Speed[j] = thisCar->Speed[j + 1];
		thisCar->Position[2] = thisCar->Position[1];


		//输出到.txt文件中
		if (i >= 0 && i <= 1000)
		{
			if (thisCar->LaneNumber == 0)
				outfile4 << i << "   " << DistanceF << "   " << thisCar->Position[0] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "   " << OptimumV << endl;
			else
				outfile5 << i << "   " << DistanceF << "   " << thisCar->Position[0] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "   " << OptimumV << endl;

		}
		if (i > 1 &&i % 10000 == 0)
		{
			if (thisCar->LaneNumber == 0)
				outfile1 << i << "   " << thisCar->Position[1] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "   " << thisCar->LaneNumber << endl;
			else
				outfile2 << i << "   " << thisCar->Position[1] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "   " << thisCar->LaneNumber << endl;
		}
		if (i >= 195000 && i <= 200000 && i % 5 == 0)
		{
			if (thisCar->LaneNumber == 0)
				outfile3 << i << "   " << thisCar->Position[1] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "   " << thisCar->LaneNumber << endl;
			else
				outfile6 << i << "   " << thisCar->Position[1] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "   " << thisCar->LaneNumber << endl;

		}
		FrontalCar = thisCar;
	}
	outfile1.close();
	outfile2.close();
	outfile3.close();
	outfile4.close();
	outfile5.close();
	outfile6.close();
}

void Road::Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L,const int &tao)
{
	POSITION pos = plist->GetHeadPosition();
	if (plist->GetCount()>0)
	{
		while (pos)
		{
			//更新所有车辆的位置与速度信息
			Vehicle* car = (Vehicle*)(plist->GetNext(pos));
			if (time != 0)
			{
				car->Position[0] = car->Position[2];
				car->Speed[0] = car->Speed[tao];
			}
			//换道
			LaneChange(plist, planechange, car ,xfc, L, time);
		}
	}
}

void Road::InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar)
{
	if (!porigin->IsEmpty())
	{
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;//换道，车道号码 0->1，1->0
		thisCar->Countsum = 200;
		ptarget->AddTail(thisCar);
		POSITION pos = porigin->Find(thisCar);
		porigin->RemoveAt(pos);
	}
}

void Road::InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i)
{
	ofstream outfile6("huandao1.txt", ofstream::out | ofstream::app);
	ofstream outfile7("huandao2.txt", ofstream::out | ofstream::app);
	if (!porigin->IsEmpty())
	{
		//本车换道，车道号由0变为1，或者由1变为0
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;
		POSITION pos1 = ptarget->Find(carLeading);
		ptarget->InsertAfter(pos1, thisCar);
		POSITION posDe = porigin->Find(thisCar);
		porigin->RemoveAt(posDe);
		if (thisCar->LaneNumber == 0)
			outfile6 << i << "   " << thisCar->Position[0] << "     " << thisCar->Speed[100] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "     " << thisCar->LaneNumber << endl;
		else
			outfile7 << i << "   " << thisCar->Position[0] << "     " << thisCar->Speed[100] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Number << "     " << thisCar->LaneNumber << endl;

	}
	outfile6.close();
	outfile7.close();
}



void Road::LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, const double &xfc, const double L, int i)
{
	if (planechange->GetCount() == 0)
		InsertAtTail(plist, planechange, thisCar);
	else
	{
		//将相邻车道所有车辆的位置信息全部放置于position[N+1]数组中
		int N = planechange->GetCount();
		POSITION PosNL = planechange->GetHeadPosition();
		double *position = new double[N + 1];
		int j = 0;
		while (PosNL != NULL)
		{
			Vehicle* CarNL = (Vehicle*)(planechange->GetNext(PosNL));
			position[j] = CarNL->Position[0];
			j = j + 1;
		}
		//最后一个数组元素存放本车位置信息
		position[N] = thisCar->Position[0];

		//位置从大到小排序，本车位置也参与了排序
		sort(position, N + 1);

		//前车位置，后车位置
		double frontcarpos, followcarpos;

		//与相邻前车距离，与相邻后车距离
		double DistanceF, DistanceB;

		//计算当前车辆与相邻车道最近前车与后车的距离
		for (int k = 0; k < N + 1; k++)
		{
			if (position[k] == thisCar->Position[0])
			{
				if (k == 0)
				{
					frontcarpos = position[N];
					followcarpos = position[1];
					DistanceF = frontcarpos - thisCar->Position[0] + L;
					DistanceB = thisCar->Position[0] - followcarpos;
				}
				else if (k == N)
				{
					frontcarpos = position[N - 1];
					followcarpos = position[0];
					DistanceF = frontcarpos - thisCar->Position[0];
					DistanceB = thisCar->Position[0] - followcarpos + L;
				}
				else
				{
					frontcarpos = position[k - 1];
					followcarpos = position[k + 1];
					DistanceF = frontcarpos - thisCar->Position[0];
					DistanceB = thisCar->Position[0] - followcarpos;
				}
			}

		}//else
		delete[] position;


		PosNL = planechange->GetHeadPosition();
		Vehicle* FrontCar = (Vehicle*)(planechange->GetAt(PosNL));
		while (PosNL != NULL)
		{
			Vehicle* car = (Vehicle*)(planechange->GetNext(PosNL));
			if (car->Position[0] == frontcarpos)
				FrontCar = car;
		}

		//满足条件就【变道】
		if ((thisCar->Headway<2 * xfc) && (DistanceF>thisCar->Headway) && (DistanceB>xfc))
		{
			InsertAtMiddle(plist, planechange, thisCar, FrontCar, i);
			thisCar->Headway = DistanceF;
		}
	}
}
	



/////主函数/////
void SelfStabilization()
{
	Road LeftLane, RightLane;
	CObList* LeftQueue = &LeftLane.queue;
	CObList* RightQueue = &RightLane.queue;
	ofstream outfile6("lane1.txt", ofstream::out | ofstream::app);
	ofstream outfile7("lane2.txt", ofstream::out | ofstream::app);
	int circleTimes = 200000; //循环进行次数
	double d=0.06;      //初始化车流密度
	double xfc = 0.5;

	while (d<=0.0601)
	{
		double Speed1 = 0;
		double Speed2 = 0;
		double AverageSpeed = 0;
		int LeftVehicleSum = 50;			//左车道总车辆数
		int RightVehicleSum = 50;			//右车道总车辆数
		int tao = 150;						//延时
		double iniHeadway = (1.0) / d;		//初始车间距
		const double L = 50 * iniHeadway;	//车道总长度
		double SafeDistanceL = 4;						//左车道安全间距
		double SafeDistanceR = 4;						//右车道安全间距

		//参数初始化
		double v0 = 7.9, m = 0.13;			//左车道安全间距
		double bc = 7.3, bf = 17;			//右车道安全间距

		double iniHeadwayL = 0.7*iniHeadway + 0.3*iniHeadway / 2.0;				//初始左车道车间距
		double iniHeadwayR = 0.8*iniHeadway + 0.2*iniHeadway / 2.0;				//初始右车道车间距
		double iniSpeedL = v0*(tanh(m*(iniHeadwayL - bf)) - tanh(m*(bc - bf))); //左车道初始速度
		double iniSpeedR = v0*(tanh(m*(iniHeadwayR - bf)) - tanh(m*(bc - bf)));	//右车道初始速度

		//初始化，左右车道加车
		LeftLane.AddVehicle(LeftQueue, LeftVehicleSum, L, iniSpeedL, iniHeadway, 0);
		RightLane.AddVehicle(RightQueue, RightVehicleSum, L, iniSpeedR, iniHeadway, 1);

		//开始循环
		for (int i = 0; i <= circleTimes; i++)
		{
			//迭代
			LeftLane.Iteration(LeftQueue, RightQueue, L, i, 0.7, 0.3, tao);
			RightLane.Iteration(RightQueue,LeftQueue, L, i, 0.8, 0.2, tao);

			//更新
			LeftLane.Update(i, LeftQueue, RightQueue, xfc , L , tao);
			RightLane.Update(i, RightQueue, LeftQueue, xfc , L, tao);

			cout << i << endl;
		}

		d += 0.005;
	}
	outfile6.close();
	outfile7.close();
}


/////主函数//////
int main()
{
	SelfStabilization();
	return 0;
}


