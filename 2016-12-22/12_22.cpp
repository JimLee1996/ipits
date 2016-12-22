

#include "StdAfx.h"
#include <iostream>
//#include <iomanip>
#define _AFXDLL
#include "afx.h"
#include <fstream>
using namespace std;



/////���ܺ���/////

//�Ӵ�С����
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



/////�����ඨ����ʵ��/////
class Vehicle : public CObject
{
public:
	Vehicle(int number, double position, double speed, double acceleration, double headway, bool lane, double distf); //�����ٽ���ǰ����Ϣ
	~Vehicle();
	
	int	   Number;
	
	double Position[301];//��ʷλ��
	double Speed[301];//��ʷ�ٶ�
	double Distf[301];//��ʷ�ٳ���

	double Headway;
	double Acceleration;
	bool   LaneNumber;//left=0,right=1
};

//  �������캯��	   ����	 		λ��			�ٶ�			���ٶ�			��ͷ��		  ����		�ٳ�����ͷ��
Vehicle::Vehicle(int number, double position, double speed, double acceleration, double headway, bool lane, double distf)    //�����ٽ���ǰ����Ϣ
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




/////�����ඨ����ʵ��/////
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
	//�������֮���ٳ���Ҳ��ı䣬����ע��
	//
	//
	//����״̬
	void Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao);

	///�ӳ�����///
	void InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar);
	void InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i);
	///�ӳ�����///
	
	//��������
	void LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, const double &xfc, const double L, int i);

	CObList queue;//���Ķ���

};

Road::Road()
{
}
Road::~Road()
{
}

//   ��·�ӳ�����		��������			������				��������			 ���복ͷ��				������				 �Ż��ٶ�          �ٳ�����ͷ��
void Road::AddVehicle(CObList* &plist, const int &carSum, const double &Length, const double &carHeadway, const bool &lanenumber, double OptimumV, double distf) //distf
{
	for (int i = 1; i<carSum + 1; i++)
	{
		if (lanenumber == 0)
		{
			Vehicle* vehicle = new Vehicle(i, Length + carHeadway / 2.0 - i*carHeadway, OptimumV, 0, carHeadway, lanenumber, distf);
					//  �������캯��	   ����	 		λ�� С�ų���ǰ                 �ٶ�   ���ٶ� ��ͷ��		 ����	�ٳ�����ͷ��			

			plist->AddTail(vehicle);
		}
		else
		{
			Vehicle* vehicle = new Vehicle(i+100,           Length - i*carHeadway        , OptimumV, 0, carHeadway, lanenumber, distf);  //�ӳ��״�
			plist->AddTail(vehicle);
		}
	}
}

//      ������ź���    ��������        �ٳ�������            ��������        ѭ������      Ȩ��			Ȩ��              ��ʱ            ��ȫ����
void Road::Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao, const double &hd)
{

// ����ȶ���

	ofstream outfile1("left1.txt", ofstream::out | ofstream::app);
	ofstream outfile2("right1.txt", ofstream::out | ofstream::app);

	ofstream outfile5("left0.txt", ofstream::out | ofstream::app);
	ofstream outfile6("right0.txt", ofstream::out | ofstream::app);

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


	double OptimumV;
	double t = 0.01;			//ÿ��������ǰ������Сʱ�䵥λ
	/////////////////////////////////////////////////
	double lambda1 = 0.4;		//����ϵ��
	double lambda2 = 0.4;
	////////////////////////////////////////////////
	double k1, k2, k3, k4;      //���������ϵ��
	////////////////////////////////////////////////
	double a1 = 2.0;			//��ʻԱ����ϵ��
	double a2 = 2.0;


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

	POSITION post = plist->GetTailPosition();
	Vehicle* FrontalCar = (Vehicle*)(plist->GetAt(post));  //��һ������ǰ��

	Vehicle* thisCar;
	POSITION pos = plist->GetHeadPosition();               //��һ����

	int N = planechange->GetCount();

	while (pos)  //����
	{
		thisCar = (Vehicle*)(plist->GetNext(pos));
		//�������������ȷ�Χ������������λ��
		if (thisCar->Position[0] >= L)
			thisCar->Position[0] = thisCar->Position[0] - L;
		if (FrontalCar->Position[0] >= L)
			FrontalCar->Position[0] = FrontalCar->Position[0] - L;


		//���㳵��ǰ���ĳ����
		thisCar->Headway = FrontalCar->Position[0] - thisCar->Position[0];

		///////��ͷ������///////
		if (thisCar->Headway< 0)
			thisCar->Headway += L;


		
		//�����Ŷ�����25������ǰ���ĳ�������ԭʼ������30%
		//��ͷ����и��£����Զ���趨
		if (i >= 100 && i <= 150)
		{
			if (thisCar->Number == 25)
			{
				thisCar->Headway = thisCar->Headway * 0.3;
				thisCar->Position[0] = thisCar->Position[0] + 0.7 * thisCar->Headway;
				thisCar->Distf[0] = thisCar->Distf[0] - 0.7 * thisCar->Headway;
				//�ж��Ƿ񳬳�
				if (thisCar->Position[0] >= L)
					thisCar->Position[0] = thisCar->Position[0] - L;
				if (thisCar->Distf[0] < 0)
					thisCar->Distf[0] = thisCar->DistF[0] + L;
			}
		}
		
		
		
		//�ٳ���
		POSITION posNL = planechange->GetHeadPosition();
		//��������
		double *position = new double[N + 1];
		int p = 0;
		while (posNL != NULL)
		{
			Vehicle* car = (Vehicle*)(planechange->GetNext(posNL));
			position[p] = car->Position[0];
			p++;
		}
		position[N] = thisCar->Position[0];


		///////////�ٳ���ǰ���������///////////

		//���ڳ�������ǰ��λ�ã��Լ������������
		double frontcarpos, DistanceF;
		//�Ӵ�С��������
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
		//���㱾�������ڳ������ǰ���ĳ������
		DistanceF = frontcarpos - thisCar->Position[0];
		if (DistanceF<0)
			DistanceF += L;



		////////���Ĳ���/////////
		
		//�Ż��ٶȼ���
		OptimumV = tanh(ay*thisCar->Headway + aq*DistanceF - hd) + tanh(hd);
		////////////////////////////////////////////////
		double v0 = 15;
		double beta = 1.5;
		double delta = 8;
		//city way
		//double headway = ay*thisCar->Headway + aq*DistanceF;		
		//OptimumV = v0*(1/(1+tanh(beta)))*(tanh(headway/delta-beta))+tanh(beta);
		
		
		
		////////////////////////////////////////////////
		//���ٶȼ���
		if (thisCar->LaneNumber == 0)
			//thisCar->Acceleration = a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Speed[tao] - thisCar->Speed[1]);
			thisCar->Acceleration = a1*(OptimumV - thisCar->Speed[tao]) + lambda1*(thisCar->Distf[tao] - thisCar->Distf[1]);
		else
			//thisCar->Acceleration = a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Speed[tao] - thisCar->Speed[1]);
			thisCar->Acceleration = a2*(OptimumV - thisCar->Speed[tao]) + lambda2*(thisCar->Distf[tao] - thisCar->Distf[1]);

		//�ٶȼ��㣬�Ĵ��������
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

		
		//////////���Ĳ���//////////
			
		
		//�����ٶȺ�λ��
		//�ٶȲ�����Ϊ�������ͣ����
		if (thisCar->Speed[tao] <= 0)
		{
			thisCar->Position[tao] = thisCar->Position[tao - 1];
			thisCar->Distf[tao] = thisCar->Distf[tao-1];
			thisCar->Speed[tao] = 0;
		}
		else
			thisCar->Position[tao] = thisCar->Position[tao-1] + thisCar->Speed[tao] * t + 0.5*thisCar->Acceleration * t * t;
			thisCar->Distf[tao] = DistanceF;	
			//���޸�

		//��������	
		if (thisCar->Position[tao] >= L)
			thisCar->Position[tao] = thisCar->Position[tao] - L;

		//������Ϣ
		for (int j = 1; j<tao; j++)
		{
			thisCar->Speed[j] = thisCar->Speed[j + 1];
			thisCar->Position[j] = thisCar->Position[j+1];
			thisCar->Distf[j] = thisCar->Distf[j+1];
		}




		//�����.txt�ļ���
		if (i < 20000 && i % 100 == 0)
		{
			// ���ǻ���֮�󣬳��ĳ�����Ϣ������
			if (thisCar->LaneNumber == 0)
				outfile5 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[tao] << endl;
			else
				outfile6 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[tao] << "   " << thisCar->Speed[tao] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[tao] << endl;
		}
		
		if (i > 130000 && i % 100 == 0)
		{
			// ���ǻ���֮�󣬳��ĳ�����Ϣ������
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


//    ��·����    ʱ��         ����              �ٳ���              ��ȫ����       ��������       ��ʱ
void Road::Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao)
{
	POSITION pos = plist->GetHeadPosition();
	if (plist->GetCount()>0)
	{
		while (pos)
		{
			//�������г�����λ�����ٶ���Ϣ
			Vehicle* car = (Vehicle*)(plist->GetNext(pos));
			if (time != 0)
			{
				car->Position[0] = car->Position[tao];
				car->Speed[0] = car->Speed[tao];
				car->Distf[0] = car->Distf[tao];
			}
			//��������
			LaneChange(plist, planechange, car, xfc, L, time);
		}
	}
}

//����1                        Դ               Ŀ��              ����                    
void Road::InsertAtTail(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar)
{
	if (!porigin->IsEmpty())
	{
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;//�������������� 0->1��1->0
		//��ȫû��Ҫ��
		ptarget->AddTail(thisCar);
		POSITION pos = porigin->Find(thisCar);
		porigin->RemoveAt(pos);
	}
}

//����2                        Դ                Ŀ��                ����              ��ͷ��           ��Ϣ�����
void Road::InsertAtMiddle(CObList* &porigin, CObList* &ptarget, Vehicle* &thisCar, Vehicle* &carLeading, int i)
{

	ofstream outfile3("huandao1.txt", ofstream::out | ofstream::app);
	ofstream outfile4("huandao2.txt", ofstream::out | ofstream::app);

	if (!porigin->IsEmpty())
	{
		//������������������0��Ϊ1��������1��Ϊ0
		thisCar->LaneNumber = ((int)(thisCar->LaneNumber + 1)) % 2;
		POSITION pos1 = ptarget->Find(carLeading);
		ptarget->InsertAfter(pos1, thisCar);
		POSITION posDe = porigin->Find(thisCar);
		porigin->RemoveAt(posDe);

		//���������Ϣ/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (thisCar->LaneNumber == 0)
			outfile3 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[0] << "   " << thisCar->Speed[0] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[0] << endl;
		else
			outfile4 << i << "   " << thisCar->Number << "   " << thisCar->LaneNumber << "   " << thisCar->Position[0] << "   " << thisCar->Speed[0] << "   " << thisCar->Acceleration << "   " << thisCar->Headway << "   " << thisCar->Distf[0] << endl;

	}
	outfile3.close();
	outfile4.close();
}


//    ��������			����				�ٳ���				����				��ȫ����			��������	��Ϣ�����
void Road::LaneChange(CObList* &plist, CObList* &planechange, Vehicle* &thisCar, const double &xfc, const double L, int i)
{
	if (planechange->GetCount() == 0)
		InsertAtTail(plist, planechange, thisCar);
	else
	{
		//�����ڳ������г�����λ����Ϣȫ��������position[N+1]������
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
		//���һ������Ԫ�ش�ű���λ����Ϣ
		position[N] = thisCar->Position[0];

		//λ�ôӴ�С���򣬱���λ��Ҳ����������
		sort(position, N + 1);

		//ǰ��λ�ã���λ��
		double frontcarpos, followcarpos;

		//������ǰ�����룬�����ں󳵾���
		double DistanceF, DistanceB;

		//���㵱ǰ���������ڳ������ǰ����󳵵ľ���
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

		//���㻻�������� ��� 
		if ((thisCar->Headway<2 * xfc) && (DistanceF>thisCar->Headway) && (DistanceB>xfc))
		{
			InsertAtMiddle(plist, planechange, thisCar, FrontCar, i);
			thisCar->Headway = DistanceF;
		}
	}
}




/////������/////
void SelfStabilization()
{
	//��ʼ��
	Road LeftLane, RightLane;
	CObList* LeftQueue = &LeftLane.queue;
	CObList* RightQueue = &RightLane.queue;


	int circleTimes = 150000;  //ѭ�����д���
	double d = 0.5;				//��ʼ�������ܶ�
	double xfc = 0.5;			//���밲ȫ�����



	while (d <= 0.501)
	{
		double Speed1 = 0;
		double Speed2 = 0;
		double AverageSpeed = 0;
		int LeftVehicleSum = 100;			//�󳵵��ܳ�����
		int RightVehicleSum = 100;			//�ҳ����ܳ�����
/////////////////////////////////////////////////////////////////

		int tao = 100;						//��ʱ
		
/////////////////////////////////////////////////////////////////		
		double iniHeadway = (1.0) / d;		//��ʼ�����
		double iniDistf = iniHeadway / 2;   //��ʼ���ڳ�����ͷ��
		const double L = 200;				//�����ܳ���
		

		double iniHeadwayL = 1.7;				//��ʼ�󳵵������
		double iniHeadwayR = 1.8;				//��ʼ�ҳ��������
		double iniSpeedL = 0.5*2.0*(tanh(iniHeadway - 1.7) + tanh(1.7)); //�󳵵���ʼ�ٶ�
		double iniSpeedR = 0.5*1.5*(tanh(iniHeadway - 1.8) + tanh(1.8));	//�ҳ�����ʼ�ٶ�

//��ʼ�������ҳ����ӳ�
//   ��·�ӳ�����		��������			������				��������			 ���복ͷ��				������				 �Ż��ٶ�          �ֳ�����ͷ��
//    AddVehicle(CObList* &plist, const int &carSum, const double &Length, const double &carHeadway, const bool &lanenumber, double OptimumV, double distf)
		LeftLane.AddVehicle(LeftQueue, LeftVehicleSum, L, iniHeadway, 0, iniSpeedL, iniDistf);//distf
		RightLane.AddVehicle(RightQueue, RightVehicleSum, L, iniHeadway, 1, iniSpeedR, iniDistf);//distf

		//��ʼѭ��
		for (int i = 0; i <= circleTimes; i++)
		{
			//����
			//      ������ź���    ��������        �ٳ�������            ��������        ����     Ȩ��				 Ȩ��              ��ʱ              ��ȫ����
            //    Iteration(CObList* &plist, CObList* &planechange, const double &L, int i, const double &ay, const double &aq, const int &tao, const double &hd)
			LeftLane.Iteration(LeftQueue, RightQueue, L, i, 0.7, 0.3, tao, 2);
			RightLane.Iteration(RightQueue, LeftQueue, L, i, 0.8, 0.2, tao, 2);

			//����
			//    ��·����    ʱ��         ����              �ٳ���              ��ȫ����       ��������       ��ʱ
            //	  Update(int time, CObList* &plist, CObList* &planechange, double &xfc, const double &L, const int &tao)
			LeftLane.Update(i, LeftQueue, RightQueue, xfc, L, tao);
			RightLane.Update(i, RightQueue, LeftQueue, xfc, L, tao);

			cout << i << endl;
			//��ʱ
		}

		d += 0.05;
	}

	cout << "Finish!" << endl;
	getchar();
}


/////������//////
int main()
{
	SelfStabilization();
	return 0;
}
