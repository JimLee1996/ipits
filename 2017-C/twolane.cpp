
#include "stdafx.h"
#include <iostream>
//#include <iomanip>
#define _AFXDLL
#include "afx.h"
#include <fstream>
using namespace std;

//横轴Headway
float hdw = 15.0;
//延时tao
const int tao = 100;
//反馈系数
float lambda1 = 0.4;
float lambda2 = 0.4;

//每步长下所前进的最小时间单位
float t = 0.01;
//循环次数
int circleTimes = 100000

//敏感系数
float a1 = 2.0;
float a2 = 2.0;

//车道长度
float L = 2000.0;
//车数
int carsum = 100;

//安全车距
float xfc = 2.0;
float ay = 0.8;
float aq = 0.2;

//优化速度函数
float OV1(float ay, float aq, float headway, float distf, float hd = 2.0)
{
	return tanh(ay * headway + aq * distf - hd) + tanh(hd);
}
float OV2(float ay, float aq, float headway, float distf)
{
	float delta = 8.0;
	float beta = 1.5;
	float v0 = 15.0;
	float dx = ay * headway + aq * distf;
	return v0 * (1/(1+tanh(beta))) * (tanh(dx/delta-beta)) + tanh(beta);
}

//从大到小排序
void sort(float arr[], int n)
{
	int i, j, k;
	float w;
	for (i = 0; i<n - 1; i++)
	{
		k = i;
		for (j = i + 1; j<n; j++)
			if (arr[k]<arr[j])
				k = j;
		if (i != k)
		{
			w = arr[i];
			arr[i] = arr[k];
			arr[k] = w;
		}
	}
}


//车辆类
class Vehicle : public CObject
{
public:
	Vehicle(int number, bool lane, float position, float speed, float acceleration, float headway, float distf);
	~Vehicle();

	int	   Number;
	bool   LaneNumber;

	float Headway[tao+1];
	float Acceleration[tao+1];
	float Position[tao+1];
	float Speed[tao+1];
	float Distf[tao+1];

};

Vehicle::Vehicle(int number, bool lane, float position, float speed, float acceleration, float headway, float distf)
{
	Number = number;
	LaneNumber = lane;

	for (int i = 0; i <= tao; i++)
	{
		Position[i] = position;
		Speed[i] = speed;
		Acceleration[i] = acceleration;
		Headway[i] = headway;
		Distf[i] = distf;
	}

}
Vehicle::~Vehicle()
{
}


//车道类
class Road : public CObject
{
public:
	Road();
	~Road();

	void AddVehicle(CObList* &plist, int carSum, float Length, float carHeadway, bool lanenumber, float OptimumV, float distf);																																					
	void Iteration(CObList* &plist, CObList* &planechange, const float &ay, const float &aq, int i);
	void Update(CObList* &plist, CObList* &planechange, int time);
	void InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar);
	void InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i);
	void LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, int i);

	CObList queue;

};

Road::Road()
{
}
Road::~Road()
{
}

void Road::AddVehicle(CObList* &plist, int carSum, float Length, float carHeadway, bool lanenumber, float OptimumV, float distf)
{
	if (lanenumber == 0)
	{
		for (int i = 1; i <= carSum; i++)
		{
			Vehicle* vehicle = new Vehicle(i, 0, Length - i*carHeadway, OptimumV, 0, carHeadway, distf);
			plist->AddTail(vehicle);
		}
	}
	else
	{
		for (int i = 1; i <= carSum; i++)
		{
			Vehicle* vehicle = new Vehicle(i + 100, 1, Length + carHeadway / 2.0 - i*carHeadway, OptimumV, 0, carHeadway, distf);
			plist->AddTail(vehicle);
		}
	}
}

void Road::Iteration(CObList* &plist, CObList* &planechange, const float &ay, const float &aq, int i)
{
	float OptimumV;
	float k1, k2, k3, k4;   

	POSITION post = plist->GetTailPosition();
	Vehicle* FrontalCar = (Vehicle*)(plist->GetAt(post)); //第一辆车的前车

	Vehicle* thisCar;
	POSITION pos = plist->GetHeadPosition(); //第一辆车

	int N = planechange->GetCount();

	//对车道车遍历
	while (pos)
	{
		thisCar = (Vehicle*)(plist->GetNext(pos)); // return pos; pos++


		//加入扰动，第25辆车与前车的车间距减小
		if (i == 2000)
		{
			if (thisCar->Number == 25)
			{
				float tempheadway;
				tempheadway = FrontalCar->Position[tao] - thisCar->Position[tao];
				if (tempheadway < 0)
					tempheadway += L;
				thisCar->Position[tao] += 0.5 * tempheadway;
				if (thisCar->Position[tao] >= L)
					thisCar->Position[tao] -= L;
			}
		}

		thisCar->Headway[tao] = FrontalCar->Position[tao] - thisCar->Position[tao];
		if (thisCar->Headway[tao] < 0)
			thisCar->Headway[tao] += L;

		//临车道最近前车车头距
		POSITION posNL = planechange->GetHeadPosition();
		float *position = new float[N + 1];
		int p = 0;
		while (posNL != NULL)
		{
			Vehicle* car = (Vehicle*)(planechange->GetNext(posNL));
			position[p] = car->Position[tao];
			p++;
		}
		position[N] = thisCar->Position[tao];
		float frontcarpos;
		sort(position, N + 1); //从大到小进行排序
		for (int k = 0; k< N + 1; k++)
		{
			if (position[k] == thisCar->Position[tao])
			{
				if (k == 0)
					frontcarpos = position[N];
				else
					frontcarpos = position[k - 1];
				break;
			}
		}
		delete[] position;
		thisCar->Distf[tao] = frontcarpos - thisCar->Position[tao];
		if (thisCar->Distf[tao] < -L/2)
			thisCar->Distf[tao] += L;

		//优化速度计算
		OptimumV = OV2(ay, aq, thisCar->Headway[tao], thisCar->Distf[tao])

		//龙格库塔法计算速度
		if (thisCar->LaneNumber == 0)
		{
			k1 = t*(a1*(OptimumV - thisCar->Speed[tao-1]) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k2 = t*(a1*(OptimumV - thisCar->Speed[tao-1] - 0.5*k1) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k3 = t*(a1*(OptimumV - thisCar->Speed[tao-1] - 0.5*k2) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k4 = t*(a1*(OptimumV - thisCar->Speed[tao-1] - k3) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			thisCar->Speed[tao] = thisCar->Speed[tao-1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);
		}
		else
		{
			k1 = t*(a2*(OptimumV - thisCar->Speed[ta0-1]) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k2 = t*(a2*(OptimumV - thisCar->Speed[tao-1] - 0.5*k1) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k3 = t*(a2*(OptimumV - thisCar->Speed[tao-1] - 0.5*k2) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k4 = t*(a2*(OptimumV - thisCar->Speed[tao-1] - k3) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			thisCar->Speed[tao] = thisCar->Speed[tao-1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);
		}

		//加速度计算
		if (thisCar->LaneNumber == 0)
			thisCar->Acceleration[tao] = a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]);
		else
			thisCar->Acceleration[tao] = a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]);


		//更新位置
		thisCar->Position[0] = thisCar->Position[tao] + thisCar->Speed[tao] * t + 0.5*thisCar->Acceleration[tao] * t * t;
		//超程清零	
		if (thisCar->Position[0] >= L)
			thisCar->Position[0] -= L;

		FrontalCar = thisCar;
	}





	// 输出稳定点
	ofstream outfile1("left1.txt", ofstream::out | ofstream::app);
	ofstream outfile2("right1.txt", ofstream::out | ofstream::app);
	ofstream outfile5("left0.txt", ofstream::out | ofstream::app);
	ofstream outfile6("right0.txt", ofstream::out | ofstream::app);

		//输出到.txt文件中
		if (i < 20000 && i % 100 == 0)
		{
			// 考虑换道之后，车的车道信息被更换
			if (thisCar->LaneNumber == 0)
				outfile5 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration[tao] << "   " << thisCar->Headway[tao] << "   " << thisCar->Distf[tao] << endl;
			else
				outfile6 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration[tao] << "   " << thisCar->Headway[tao] << "   " << thisCar->Distf[tao] << endl;
		}

		if (i > 130000 && i % 100 == 0)
		{
			// 考虑换道之后，车的车道信息被更换
			if (thisCar->LaneNumber == 0)
				outfile1 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration[tao] << "   " << thisCar->Headway[tao] << "   " << thisCar->Distf[tao] << endl;
			else
				outfile2 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration[tao] << "   " << thisCar->Headway[tao] << "   " << thisCar->Distf[tao] << endl;
		}

		//更新信息
		for (int j = 1; j<tao; j++)
		{
			thisCar->Headway[j] = thisCar->Headway[j + 1];
			thisCar->Acceleration[j] = thisCar->Acceleration[j + 1];
			thisCar->Speed[j] = thisCar->Speed[j + 1];
			thisCar->Position[j] = thisCar->Position[j + 1];
			thisCar->Distf[j] = thisCar->Distf[j + 1];
		}






	outfile1.close();
	outfile2.close();
}

void Road::Update(CObList* &plist, CObList* &planechange, int time)
{
	POSITION pos = plist->GetHeadPosition();
	if (plist->GetCount()>0)
	{
		while (pos)
		{
			Vehicle* car = (Vehicle*)(plist->GetNext(pos));
			if (time != 0)
			{
				float newpos;
				newpos = car->Position[0];
				for (int j=0;j<tao;j++)
				{
					car->Headway[j] = car->Headway[j+1];
					car->Acceleration[j] = car->Acceleration[j+1];
					car->Position[j] = car->Position[j+1];
					car->Speed[j] = car->Speed[j+1];
					car->Distf[j] = car->Distf[j+1];

				}
				car->Position[tao] = newpos;
			}
			LaneChange(plist, planechange, car, time);
		}
	}
}
               
void Road::InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar)
{
	if (!porigin->IsEmpty())
	{
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;
		ptarget->AddTail(thisCar);
		POSITION pos = porigin->Find(thisCar);
		porigin->RemoveAt(pos);
	}
}

void Road::InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i)
{

	ofstream outfile3("huandao1.txt", ofstream::out | ofstream::app);
	ofstream outfile4("huandao2.txt", ofstream::out | ofstream::app);

	if (!porigin->IsEmpty())
	{
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;
		POSITION pos1 = ptarget->Find(carLeading);
		ptarget->InsertAfter(pos1, thisCar);
		POSITION posDe = porigin->Find(thisCar);
		porigin->RemoveAt(posDe);

		if (thisCar->LaneNumber == 0)
			outfile3 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration[tao] << "   " << thisCar->Headway[tao] << "   " << thisCar->Distf[tao] << endl;
		else
			outfile4 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration[tao] << "   " << thisCar->Headway[tao] << "   " << thisCar->Distf[tao] << endl;

	}
	outfile3.close();
	outfile4.close();
}

void Road::LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, int i)
{
	if (planechange->GetCount() == 0)
		InsertAtTail(plist, planechange, thisCar);
	else
	{
		int N = planechange->GetCount();
		POSITION PosNL = planechange->GetHeadPosition();
		float *position = new float[N + 1];
		int j = 0;
		while (PosNL != NULL)
		{
			Vehicle* CarNL = (Vehicle*)(planechange->GetNext(PosNL));
			position[j] = CarNL->Position[tao];
			j++;
		}
		//最后一个数组元素存放本车位置信息
		position[N] = thisCar->Position[tao];

		//位置从大到小排序，本车位置也参与了排序
		sort(position, N + 1);

		//前车位置，后车位置
		float frontcarpos, followcarpos;

		//与相邻前车距离，与相邻后车距离
		float DistanceF, DistanceB;

		//计算当前车辆与相邻车道最近前车与后车的距离
		for (int k = 0; k < N + 1; k++)
		{
			if (position[k] == thisCar->Position[tao])
			{
				if (k == 0)
				{
					frontcarpos = position[N];
					followcarpos = position[1];
					DistanceF = frontcarpos - thisCar->Position[tao] + L;
					DistanceB = thisCar->Position[tao] - followcarpos;
				}
				else if (k == N)
				{
					frontcarpos = position[N - 1];
					followcarpos = position[tao];
					DistanceF = frontcarpos - thisCar->Position[tao];
					DistanceB = thisCar->Position[tao] - followcarpos + L;
				}
				else
				{
					frontcarpos = position[k - 1];
					followcarpos = position[k + 1];
					DistanceF = frontcarpos - thisCar->Position[tao];
					DistanceB = thisCar->Position[tao] - followcarpos;
				}
			}

		}
		delete[] position;


		PosNL = planechange->GetHeadPosition();
		Vehicle* FrontCar = (Vehicle*)(planechange->GetAt(PosNL));
		while (PosNL != NULL)
		{
			Vehicle* car = (Vehicle*)(planechange->GetNext(PosNL));
			if (car->Position[tao] == frontcarpos)
				FrontCar = car;
		}

		//满足换道条件就 变道 
		if ((thisCar->Headway<2 * xfc) && (DistanceF>thisCar->Headway[tao]) && (DistanceB>xfc))
		{
			InsertAtMiddle(plist, planechange, thisCar, FrontCar, i);
			thisCar->Distf[tao] = thisCar->Headway[tao]
			thisCar->Headway[tao] = DistanceF;
		}
	}
}




//主函数
void SelfStabilization()
{
	//初始化
	Road LeftLane, RightLane;
	CObList* LeftQueue = &LeftLane.queue;
	CObList* RightQueue = &RightLane.queue;

	float d = 0.5;				//初始化车流密度

	while (d <= 0.501)
	{

		// double iniHeadway = (1.0) / d;		//初始车间距
		// double iniDistf = iniHeadway / 2;   //初始相邻车道车头距
		// const double L = 200;				//车道总长度


		// double iniHeadwayL = 1.7;				//初始左车道车间距
		// double iniHeadwayR = 1.8;				//初始右车道车间距
		// double iniSpeedL = 0.5*2.0*(tanh(iniHeadway - 1.7) + tanh(1.7)); //左车道初始速度
		// double iniSpeedR = 0.5*1.5*(tanh(iniHeadway - 1.8) + tanh(1.8));	//右车道初始速度

																			
		LeftLane.AddVehicle(LeftQueue, carsum, L, hdw, 0, OV2(ay,aq,hdw,hdw/2), hdw/2.0);
		RightLane.AddVehicle(RightQueue, carsum, L, hdw, 1, OV2(ay,aq,hdw,hdw/2), hdw/2.0);

		for (int i = 0; i <= circleTimes; i++)
		{
			LeftLane.Iteration(LeftQueue, RightQueue, ay, aq, i);
			RightLane.Iteration(RightQueue, LeftQueue, ay, aq, i);

			LeftLane.Update(LeftQueue, RightQueue, i);
			RightLane.Update(RightQueue, LeftQueue, i);

			if (i%100 == 0) //计时
				cout << i << endl;	
		}

		d += 0.05;
	}

	cout << "Finish!" << endl;
	getchar();
}


int main()
{
	SelfStabilization();
	return 0;
}
