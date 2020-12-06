#include <iostream>
#include <vector>
#include <stdio.h>      /* printf */
#include <math.h> 

using namespace std;


/* TO DO:
1. define a variable for gravity
*/


class Mass
{
public:
	int index = 0;
	float m = 0.1;
	float F[3] = {0, 0, 0};
	float a[3] = {0, 0, 0};
	float v[3] = {0, 0, 0};
	float p[3] = {0, 0, 0}; 
	float miu_s = 1;
	float miu_k = 0.8;
	
	Mass(int i, float mass, float positions[3])
	{
		index = i;
		m = mass;
		for (int i {0}; i<3; ++i)
			p[i] = positions[i] + 0.1f; 
		// p = positions;
		// add F, a, v
	}

	void update_pos(float dt){

		for (int i {0}; i<3; ++i){

			if (p[2] < 0){
				F[2] += 100000 * (-p[2]); // ground vertical force 
			}

			if (i != 2)
				a[i] = F[i] / m;
			else
				a[i] = F[i] / m - 9.81;
			v[i] = v[i] + a[i] * dt * 0.999; // 0.999 for damping
			p[i] = p[i] + v[i] * dt;

		}

			
	}

		
	// ~Mass();	
};

class Spring 
{
public:
	float k = 10000.0f;
	float l_0 = 0.0f;
	int m1_index;
	int m2_index;
	Spring(float length, int index_1, int index_2)
	{
		l_0 = length;
		m1_index = index_1;
		m2_index = index_2;
	}	
};

float distance(Mass m1, Mass m2){

	float result = 0.0f;
	for (int i {0}; i<3; ++i)
		result += pow(m1.p[i]-m2.p[i], 2);
	result = sqrt(result);
	return result;
}
	
vector<float> direction(Mass m1, Mass m2){
	vector<float> dis_dir = {0, 0, 0, 0};

	float dis = 0.0f;
	for (int i {0}; i<3; ++i)
		dis += pow(m1.p[i]-m2.p[i], 2);
	dis = sqrt(dis);

	dis_dir[3] = dis;
	for (int i {0}; i<3; ++i)
		dis_dir[i] = (m1.p[i] - m2.p[i]) / dis;

	return dis_dir;
}

int main(){
	


	// positions?

	float mass_pos[4][3] = {
		{-0.1f, -0.1f,-0.1f},
		{-0.1f,  0.1f,-0.1f},
		{ 0.1f,  0.1f,-0.1f},
		{ 0.1f, -0.1f,-0.1f}
		// {-0.1f, -0.1f, 0.1f},
		// {-0.1f,  0.1f, 0.1f},
		// { 0.1f,  0.1f, 0.1f},
		// { 0.1f, -0.1f, 0.1f} 
	};

	// instancaite every mass on the robot (cube) and store them in mass_ls
	vector<Mass> mass_ls;
	for (int i {0}; i<4; ++i){
		Mass m(i, 0.1, mass_pos[i]);
		mass_ls.push_back(m);
	}
		

	for (int i {0}; i<mass_ls.size(); ++i){
		cout << '\n' << "Mass Index: " << mass_ls[i].index << endl;
		cout << "Mass Positions: ";
		for (int j {0}; j<3; ++j)
			cout << mass_ls[i].p[j] << ' ' ;
	}

	cout << '\n';
	int count = 0;
	vector<Spring> spring_ls;
	for (int i {0}; i<mass_ls.size(); ++i){
		for (int j {i+1}; j<mass_ls.size(); ++j){
			count++;
			// cout << "Mass Indices " << i << " " << j << endl;
			float length = distance(mass_ls[i], mass_ls[j]);
			// cout << "Spring Rest Length " << length << endl;
			Spring spr(length, i, j);
			spring_ls.push_back(spr);
		}
	}
	for (auto spr : spring_ls){
		cout << "Spring inices " << spr.m1_index << " " << spr.m2_index << endl;
	}
	cout << "Num of springs " << count << endl;
	// for (float p : m1.p)
	// 	cout << p << endl;
	// for (float v : m1.v)
	// 	cout << v << endl;
	// return 0;
	// Simulate 
	float t = 0.0f;
	float dt = 0.0001f;

	while (t < 0.01){
		t += dt;

		cout << "T: " << t << endl;
		// zero the force in every mass
		// Later integrate in update_pos
		for (int i {0}; i<<mass_ls.size(); ++i){
			for (int j {0}; j<3; ++j){

				mass_ls[i].F[j] = 0.0f;
			}
		}

		for (auto spr : spring_ls){
			vector<float> dis_dir = direction(mass_ls[spr.m1_index], mass_ls[spr.m2_index]);
			float l = dis_dir[3];
			float F = spr.k * (l - spr.l_0);

			for (int i {0}; i<3; ++i){
				mass_ls[spr.m1_index].F[i] = mass_ls[spr.m1_index].F[i] + F * dis_dir[i];
				mass_ls[spr.m2_index].F[i] = mass_ls[spr.m2_index].F[i] - F * dis_dir[i];
			}			
		}

		for (int i {0}; i<mass_ls.size(); ++i){
			mass_ls[i].update_pos(dt);
		}
		for (int i {0}; i<mass_ls.size(); ++i){
			cout << '\n' << "Mass Index: " << mass_ls[i].index << endl;
			cout << "Mass Positions: ";
			for (int j {0}; j<3; ++j)
				cout << mass_ls[i].p[j] << ' ' ;
		}
	cout << endl;
	

	}
	return 0;
}
	

