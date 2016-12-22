

#include "StdAfx.h"
#include <iostream>
//#include <iomanip>
#define _AFXDLL
#include "afx.h"
#include <fstream>
using namespace std;



/////功能函数/////

//从大到小排序
void sort(double arr[], int n)     
{
	int i, j, k;
	double w;
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



/////车辆类定义与实现/////
class Vehicle : public CObject
{
public:
	Vehicle(int number, double position, double speed, double acceleration, double headway, bool lane, double distf); //引入临近最前车信息
	~Vehicle();
	
	int	   Number;
	
	double Position[301];//历史位置
	double Speed[301];//历史速度
	double Distf[301];//历史临车距

	double Headway;
	double Acceleration;
	bool   LaneNumber;//left=0,right=1
};

//  车辆构造函数	   车号	 		位置			速度			加速度			车头距		  车道		临车道车头距
Vehicle::Vehicle(int number, double position, double speed, double acceleration, double headway, bool lane, double distf)    //引入临近最前车信息
{
	Number = number;
//	Position[0] = position;
//	Position[1] = position;
//	Position[2] = position;
	for (int i = 0; i < 301; i++)
	{
		Position[i] = position;
		Speed[i] = speed;
		Distf[i] = distf;
	}
	Acceleration = acceleration;
	Headway = headway;

	LaneNumber = lane;
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

	void AddVehicle(CObList* &plist, const int &carSum, const double &Length, const double &carHeadway, const bool &lanenumber, double OptimumV, double distf); 
	//distf
	void Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao, const double &hd);
	//
	//
	//加入干扰之后，临车距也会改变，后面注意
	//
	//
	//更新状态
	void Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao);

	///加车函数///
	void InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar);
	void InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i);
	///加车函数///
	
	//换道函数
	void LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, const double &xfc, const double L, int i);

	CObList queue;//车的队列

};

Road::Road()
{
}
Road::~Road()
{
}

//   道路加车函数		车道链表			车辆数				车道长度			 理想车头距				车道号				 优化速度          临车道车头距
void Road::AddVehicle(CObList* &plist, const int &carSum, const double &Length, const double &carHeadway, const bool &lanenumber, double OptimumV, double distf) //distf
{
	for (int i = 1; i<carSum + 1; i++)
	{
		if (lanenumber == 0)
		{
			Vehicle* vehicle = new Vehicle(i, Length + carHeadway / 2.0 - i*carHeadway, OptimumV, 0, carHeadway, lanenumber, distf);
					//  车辆构造函数	   车号	 		位置 小号车在前                 速度   加速度 车头距		 车道	临车道车头距			

			plist->AddTail(vehicle);
		}
		else
		{
			Vehicle* vehicle = new Vehicle(i+100,           Length - i*carHeadway        , OptimumV, 0, carHeadway, lanenumber, distf);  //加车易错
			plist->AddTail(vehicle);
		}
	}
}

//      加入干扰函数    车道链表        临车道链表            车道长度        循环次数      权重			权重              延时            安全车距
void Road::Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao, const double &hd)
{

// 输出稳定点

	ofstream outfile1("left1.txt", ofstream::out | ofstream::app);
	ofstream outfile2("right1.txt", ofstream::out | ofstream::app);

	ofstream outfile5("left0.txt", ofstream::out | ofstream::app);
	ofstream outfile6("right0.txt", ofstream::out | ofstream::app);

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


	double OptimumV;
	double t = 0.01;			//每步长下所前进的最小时间单位
	/////////////////////////////////////////////////
	double lambda1 = 0.4;		//反馈系数
	double lambda2 = 0.4;
	////////////////////////////////////////////////
	double k1, k2, k3, k4;      //龙格库塔法系数
	////////////////////////////////////////////////
	double a1 = 2.0;			//驾驶员敏感系数
	double a2 = 2.0;


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

	POSITION post = plist->GetTailPosition();
	Vehicle* FrontalCar = (Vehicle*)(plist->GetAt(post));  //第一辆车的前车

	Vehicle* thisCar;
	POSITION pos = plist->GetHeadPosition();               //第一辆车

	int N = planechange->GetCount();

	while (pos)  //遍历
	{
		thisCar = (Vehicle*)(plist->GetNext(pos));
		//若超出车道长度范围，则修正车辆位置
		if (thisCar->Position[0] >= L)
			thisCar->Position[0] = thisCar->Position[0] - L;
		if (FrontalCar->Position[0] >= L)
			FrontalCar->Position[0] = FrontalCar->Position[0] - L;


		//计算车与前车的车间距
		thisCar->Headway = FrontalCar->Position[0] - thisCar->Position[0];

		///////车头距修正///////
		if (thisCar->Headway< 0)
			thisCar->Headway += L;


		
		//加入扰动，第25辆车与前车的车间距减至原始车间距的30%
		//车头距会有更新，所以多次设定
		if (i >= 100 && i <= 150)
		{
			if (thisCar->Number == 25)
			{
				thisCar->Headway = thisCar->Headway * 0.3;
				thisCar->Position[0] = thisCar->Position[0] + 0.7 * thisCar->Headway;
				thisCar->Distf[0] = thisCar->Distf[0] - 0.7 * thisCar->Headway;
				//判断是否超出
				if (thisCar->Position[0] >= L)
					thisCar->Position[0] = thisCar->Position[0] - L;
				if (thisCar->Distf[0] < 0)
					thisCar->Distf[0] = thisCar->DistF[0] + L;
			}
		}
		
		
		
		//临车道
		POSITION posNL = planechange->GetHeadPosition();
		//用于排序
		double *position = new double[N + 1];
		int p = 0;
		while (posNL != NULL)
		{
			Vehicle* car = (Vehicle*)(planechange->GetNext(posNL));
			position[p] = car->Position[0];
			p++;
		}
		position[N] = thisCar->Position[0];


		///////////临车道前车距离计算///////////

		//相邻车道最邻前车位置，以及本车与其距离
		double frontcarpos, DistanceF;
		//从大到小进行排序
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
			DistanceF += L;



		////////核心部分/////////
		
		//优化速度计算
		OptimumV = tanh(ay*thisCar->Headway + aq*DistanceF - hd) + tanh(hd);
		////////////////////////////////////////////////
		double v0 = 15;
		double beta = 1.5;
		double delta = 8;
		//city way
		//double headway = ay*thisCar->Headway + aq*DistanceF;		
		//OptimumV = v0*(1/(1+tanh(beta)))*(tanh(headway/delta-beta))+tanh(beta);
		
		
		
		////////////////////////////////////////////////
		//加速度计算
		if (thisCar->LaneNumber == 0)
			//thisCar->Acceleration = a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]);
			thisCar->Acceleration = a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]);
		else
			//thisCar->Acceleration = a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]);
			thisCar->Acceleration = a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]);

		//速度计算，四次龙格库塔
		if (thisCar->LaneNumber == 0)
		{
			//k1 = t*(a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//k2 = t*(a1*(OptimumV - thisCar->Speed[tao] - 0.5*k1) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//k3 = t*(a1*(OptimumV - thisCar->Speed[tao] - 0.5*k2) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//k4 = t*(a1*(OptimumV - thisCar->Speed[tao] - k3) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//thisCar->Speed[tao] = thisCar->Speed[tao - 1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);

			k1 = t*(a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k2 = t*(a1*(OptimumV - thisCar->Speed[tao] - 0.5*k1) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k3 = t*(a1*(OptimumV - thisCar->Speed[tao] - 0.5*k2) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k4 = t*(a1*(OptimumV - thisCar->Speed[tao] - k3) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]));
			thisCar->Speed[tao] = thisCar->Speed[tao - 1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);
		}
		else
		{
			//k1 = t*(a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//k2 = t*(a2*(OptimumV - thisCar->Speed[tao] - 0.5*k1) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//k3 = t*(a2*(OptimumV - thisCar->Speed[tao] - 0.5*k2) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//k4 = t*(a2*(OptimumV - thisCar->Speed[tao] - k3) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]));
			//thisCar->Speed[tao] = thisCar->Speed[tao - 1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);

			k1 = t*(a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k2 = t*(a2*(OptimumV - thisCar->Speed[tao] - 0.5*k1) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k3 = t*(a2*(OptimumV - thisCar->Speed[tao] - 0.5*k2) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			k4 = t*(a2*(OptimumV - thisCar->Speed[tao] - k3) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]));
			thisCar->Speed[tao] = thisCar->Speed[tao - 1] + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4);

		}

		
		//////////核心部分//////////
			
		
		//更新速度和位置
		//速度不允许为负，最过停下来
		if (thisCar->Speed[tao] <= 0)
		{
			thisCar->Position[tao] = thisCar->Position[tao - 1];
			thisCar->Distf[tao] = thisCar->Distf[tao-1];
			thisCar->Speed[tao] = 0;
		}
		else
			thisCar->Position[tao] = thisCar->Position[tao-1] + thisCar->Speed[tao] * t + 0.5*thisCar->Acceleration * t * t;
			thisCar->Distf[tao] = DistanceF;	
			//已修改

		//超程清零	
		if (thisCar->Position[tao] >= L)
			thisCar->Position[tao] = thisCar->Position[tao] - L;

		//更新信息
		for (int j = 1; j<tao; j++)
		{
			thisCar->Speed[j] = thisCar->Speed[j + 1];
			thisCar->Position[j] = thisCar->Position[j+1];
			thisCar->Distf[j] = thisCar->Distf[j+1];
		}




		//输出到.txt文件中
		if (i < 20000 && i % 100 == 0)
		{
			// 考虑换道之后，车的车道信息被更换
			if (thisCar->LaneNumber == 0)
				outfile5 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[tao] << endl;
			else
				outfile6 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[tao] << endl;
		}
		
		if (i > 130000 && i % 100 == 0)
		{
			// 考虑换道之后，车的车道信息被更换
			if (thisCar->LaneNumber == 0)
				outfile1 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[tao] << endl;
			else
				outfile2 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[tao] << endl;
		}

		FrontalCar = thisCar;
	}
	outfile1.close();
	outfile2.close();
}


//    道路更新    时刻         车道              临车道              安全距离       车道长度       延时
void Road::Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao)
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
				car->Position[0] = car->Position[tao];
				car->Speed[0] = car->Speed[tao];
				car->Distf[0] = car->Distf[tao];
			}
			//换道函数
			LaneChange(plist, planechange, car, xfc, L, time);
		}
	}
}

//换道1                        源               目标              车车                    
void Road::InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar)
{
	if (!porigin->IsEmpty())
	{
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;//换道，车道号码 0->1，1->0
		//完全没必要变
		ptarget->AddTail(thisCar);
		POSITION pos = porigin->Find(thisCar);
		porigin->RemoveAt(pos);
	}
}

//换道2                        源                目标                车车              车头？           信息输出用
void Road::InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i)
{

	ofstream outfile3("huandao1.txt", ofstream::out | ofstream::app);
	ofstream outfile4("huandao2.txt", ofstream::out | ofstream::app);

	if (!porigin->IsEmpty())
	{
		//本车换道，车道号由0变为1，或者由1变为0
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;
		POSITION pos1 = ptarget->Find(carLeading);
		ptarget->InsertAfter(pos1, thisCar);
		POSITION posDe = porigin->Find(thisCar);
		porigin->RemoveAt(posDe);

		//输出换道信息/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (thisCar->LaneNumber == 0)
			outfile3 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[0] << "   " << thisCar->Speed[0] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[0] << endl;
		else
			outfile4 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[0] << "   " << thisCar->Speed[0] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[0] << endl;

	}
	outfile3.close();
	outfile4.close();
}


//    换道函数			车道				临车道				车车				安全距离			车道长度	信息输出用
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
			j++;
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

		//满足换道条件就 变道 
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
	//初始化
	Road LeftLane, RightLane;
	CObList* LeftQueue = &LeftLane.queue;
	CObList* RightQueue = &RightLane.queue;


	int circleTimes = 150000;  //循环进行次数
	double d = 0.5;				//初始化车流密度
	double xfc = 0.5;			//理想安全车间距



	while (d <= 0.501)
	{
		double Speed1 = 0;
		double Speed2 = 0;
		double AverageSpeed = 0;
		int LeftVehicleSum = 100;			//左车道总车辆数
		int RightVehicleSum = 100;			//右车道总车辆数
/////////////////////////////////////////////////////////////////

		int tao = 100;						//延时
		
/////////////////////////////////////////////////////////////////		
		double iniHeadway = (1.0) / d;		//初始车间距
		double iniDistf = iniHeadway / 2;   //初始相邻车道车头距
		const double L = 200;				//车道总长度
		

		double iniHeadwayL = 1.7;				//初始左车道车间距
		double iniHeadwayR = 1.8;				//初始右车道车间距
		double iniSpeedL = 0.5*2.0*(tanh(iniHeadway - 1.7) + tanh(1.7)); //左车道初始速度
		double iniSpeedR = 0.5*1.5*(tanh(iniHeadway - 1.8) + tanh(1.8));	//右车道初始速度

//初始化，左右车道加车
//   道路加车函数		车道链表			车辆数				车道长度			 理想车头距				车道号				 优化速度          林车道车头距
//    AddVehicle(CObList* &plist, const int &carSum, const double &Length, const double &carHeadway, const bool &lanenumber, double OptimumV, double distf)
		LeftLane.AddVehicle(LeftQueue, LeftVehicleSum, L, iniHeadway, 0, iniSpeedL, iniDistf);//distf
		RightLane.AddVehicle(RightQueue, RightVehicleSum, L, iniHeadway, 1, iniSpeedR, iniDistf);//distf

		//开始循环
		for (int i = 0; i <= circleTimes; i++)
		{
			//迭代
			//      加入干扰函数    车道链表        临车道链表            车道长度        车号     权重				 权重              延时              安全车距
            //    Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao, const double &hd)
			LeftLane.Iteration(LeftQueue, RightQueue, L, i, 0.7, 0.3, tao, 2);
			RightLane.Iteration(RightQueue, LeftQueue, L, i, 0.8, 0.2, tao, 2);

			//更新
			//    道路更新    时刻         车道              临车道              安全距离       车道长度       延时
            //	  Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao)
			LeftLane.Update(i, LeftQueue, RightQueue, xfc, L, tao);
			RightLane.Update(i, RightQueue, LeftQueue, xfc, L, tao);

			cout << i << endl;
			//计时
		}

		d += 0.05;
	}

	cout << "Finish!" << endl;
	getchar();
}


/////主函数//////
int main()
{
	SelfStabilization();
	return 0;
}
